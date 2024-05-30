/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Advanced Micro Devices Inc.
 *
 * amdisp.cpp - Pipeline handler for AMD ISP devices
 */

#include <algorithm>
#include <cstring>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <math.h>
#include <memory>
#include <numeric>
#include <sys/stat.h>
#include <tuple>

#include <libcamera/base/file.h>
#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include <libcamera/camera.h>
#include <libcamera/control_ids.h>
#include <libcamera/controls.h>
#include <libcamera/formats.h>
#include <libcamera/property_ids.h>
#include <libcamera/request.h>
#include <libcamera/stream.h>

#include <libcamera/ipa/ipa_interface.h>
#include <libcamera/ipa/ipa_module_info.h>
#include <libcamera/ipa/amd_ipa_interface.h>
#include <libcamera/ipa/amd_ipa_proxy.h>

#include "libcamera/internal/camera.h"
#include "libcamera/internal/camera_sensor.h"
#include "libcamera/internal/device_enumerator.h"
#include "libcamera/internal/ipa_manager.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/pipeline_handler.h"
#include "libcamera/internal/sysfs.h"
#include "libcamera/internal/v4l2_videodevice.h"
#include "libcamera/internal/yaml_parser.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(AmdIsp4)

/**
 * Since the sensor kernel module won't be available in the first version
 * of the device driver, simulate the CameraSensor class, getting the
 * controls and properties from a config file.
 */
class AmdCameraSensor
{
public:
	AmdCameraSensor(const MediaEntity *entity);
	~AmdCameraSensor();

	int init(const YamlObject &sensorRoot);

	const ControlList &properties() const { return properties_; }
	const std::string &id() const { return id_; }
private:
	int configureSensorProperties(const YamlObject &sensorRoot);
	const MediaEntity *entity_;
	ControlList properties_;
	std::string id_;
};

AmdCameraSensor::AmdCameraSensor(const MediaEntity *entity)
	: entity_(entity)
	, properties_(properties::properties)
{
}

AmdCameraSensor::~AmdCameraSensor()
{
}

class AmdIsp4CameraData : public Camera::Private
{
public:
	/* When the AMD ISP sensor kernel module will be implemented, this class will
	 * grow and should eventually be moved to its own file
	 */
	class VideoDevice
	{
	public:
		VideoDevice(std::unique_ptr<V4L2VideoDevice> &&video)
			: video_(std::move(video))
		{
		}
		Stream &stream() { return stream_; }
		std::unique_ptr<V4L2VideoDevice> &video() { return video_; }
		std::map<PixelFormat, std::vector<SizeRange>> pixelFormats() { return pixelFormats_; }
		unsigned int bufferCount() { return bufferCount_; }
	protected:
		friend class AmdIsp4CameraData;
		int populateFormats();
		Stream stream_;
		unsigned int bufferCount_{ 0 };
		Size minResolution_{ 0, 0 };
		Size maxResolution_{ 0, 0 };
	private:
		void addControl(uint32_t cid, const ControlInfo &v4l2info,
				ControlInfoMap::Map *ctrls);

		std::unique_ptr<V4L2VideoDevice> video_;
		std::map<PixelFormat, std::vector<SizeRange>> pixelFormats_;
	};
	AmdIsp4CameraData(PipelineHandler *pipe)
		: Camera::Private(pipe)
	{
	}

	using iterator = std::vector<VideoDevice>::iterator;

	int init(MediaDevice *media);

	int loadIPA();

	void bufferReady(FrameBuffer *buffer);
	void paramsBufferReady(unsigned int id, const Flags<ipa::amd::TestFlag> flags);
	VideoDevice *at(Stream *stream);
	VideoDevice *at(StreamRole role);
	iterator begin() { return videoDevices_.begin(); }
	iterator end() { return videoDevices_.end(); }
	bool empty() const { return videoDevices_.empty(); }
	std::size_t size() const { return videoDevices_.size(); }

	const std::string &id() const { return id_; }

	static const std::vector<std::string> &supportedDeviceNames()
	{
		static const std::vector<std::string> deviceNames{
			"amd_isp_capture"
		};
		return deviceNames;
	}
	static const std::vector<std::string> &supportedEntityNames()
	{
		static const std::vector<std::string> entityNames{ "Preview", "Video", "Still" };
		return entityNames;
	}

	std::unique_ptr<ipa::amd::IPAProxyAmdIsp4> &ipa() { return ipa_; }
	std::unique_ptr<AmdCameraSensor> &sensor() { return sensor_; }

	std::vector<std::unique_ptr<FrameBuffer>> mockIPABufs_;

private:
	int loadPlatformConfiguration();
	int parsePlatformConfiguration(std::unique_ptr<YamlObject> &yamlObject);
	int configureIspProperties(const YamlObject &ispRoot);
	int allocateMockIPABuffers();
	int videoDeviceAdd(MediaDevice *media, const std::string &entityName);
	static StreamRole entityNameToRole(const std::string &entityName)
	{
		static const std::map<const std::string, const StreamRole> converter{
			{ supportedEntityNames().at(0), StreamRole::Viewfinder },
			{ supportedEntityNames().at(1), StreamRole::VideoRecording },
			{ supportedEntityNames().at(2), StreamRole::StillCapture }
		};

		if (converter.count(entityName))
			return converter.at(entityName);
		else {
			LOG(AmdIsp4, Warning)
				<< "Entity name '"
				<< entityName
				<< "' doesn't match any role. Default to "
				<< converter.at(supportedEntityNames().at(0));
			return converter.at(supportedEntityNames().at(0));
		}
	}

	constexpr bool ipaFunctional() { return false; }

	std::string configurationFile_;
	std::string id_;
	std::vector<VideoDevice> videoDevices_;
	std::map<const std::string, const StreamRole> nodeToRoleConverter_;
	std::unique_ptr<ipa::amd::IPAProxyAmdIsp4> ipa_;
	std::unique_ptr<AmdCameraSensor> sensor_;
};

class AmdIsp4CameraConfiguration : public CameraConfiguration
{
public:
	AmdIsp4CameraConfiguration();

	Status validate() override;
	void linkVideoDevice(AmdIsp4CameraData::VideoDevice *videoDevice)
	{
		videoDeviceLinks_.push_back(videoDevice);
	}

	AmdIsp4CameraData::VideoDevice *videoDeviceAt(unsigned int index)
	{
		return videoDeviceLinks_.at(index);
	}

	std::size_t videoDeviceLinkSize() const { return videoDeviceLinks_.size(); }

private:
	static constexpr unsigned int MAX_STREAMS{ 3 };
	std::vector<AmdIsp4CameraData::VideoDevice *> videoDeviceLinks_;
};

class PipelineHandlerAmdIsp4 : public PipelineHandler
{
public:
	PipelineHandlerAmdIsp4(CameraManager *manager);

	std::unique_ptr<CameraConfiguration> generateConfiguration(Camera *camera,
								   Span<const StreamRole> roles) override;
	int configure(Camera *camera, CameraConfiguration *config) override;

	int exportFrameBuffers(Camera *camera, Stream *stream,
			       std::vector<std::unique_ptr<FrameBuffer>> *buffers) override;

	int start(Camera *camera, const ControlList *controls) override;
	void stopDevice(Camera *camera) override;

	int queueRequestDevice(Camera *camera, Request *request) override;

	bool match(DeviceEnumerator *enumerator) override;

	static constexpr const char* name() { return "isp4"; }
private:
	int createCamera(MediaDevice *media);

	AmdIsp4CameraData *cameraData(Camera *camera)
	{
		return dynamic_cast<AmdIsp4CameraData *>(camera->_d());
	}
};

AmdIsp4CameraData::VideoDevice *AmdIsp4CameraData::at(Stream *stream)
{
	auto it = std::find_if(
		videoDevices_.begin(),
		videoDevices_.end(),
		[&](auto &value) {
			return &value.stream() == stream;
		});

	if (it == videoDevices_.end())
		return nullptr;
	else
		return &*it;
}

AmdIsp4CameraData::VideoDevice *AmdIsp4CameraData::at(StreamRole role)
{
	auto it = std::find_if(
		videoDevices_.begin(),
		videoDevices_.end(),
		[&](auto &value) {
			return nodeToRoleConverter_.at(value.video()->deviceNode()) == role;
		});

	if (it == videoDevices_.end())
		return nullptr;
	else
		return &*it;
}

int AmdIsp4CameraData::VideoDevice::populateFormats()
{
	for (const auto &[v4l2Format, sizes] : video_->formats()) {
		const PixelFormat pixelFormat = v4l2Format.toPixelFormat();
		if (!pixelFormat.isValid()) {
			LOG(AmdIsp4, Warning)
				<< "Media pixel format '"
				<< pixelFormat << "'"
				<< " is invalid, skipping";
			continue;
		}

		for (const SizeRange &sizeRange : sizes) {
			if (sizeRange.min < minResolution_)
				minResolution_ = sizeRange.min;
			if (sizeRange.max > maxResolution_)
				maxResolution_ = sizeRange.max;
		}
		pixelFormats_.emplace(
			std::piecewise_construct,
			std::forward_as_tuple(pixelFormat),
			std::forward_as_tuple(sizes.begin(), sizes.end()));
	}

	if (pixelFormats_.empty()) {
		LOG(AmdIsp4, Error)
			<< "'" << video_->deviceNode() << "'"
			<< "doesn't expose any supported format";
		return -EINVAL;
	}

	return 0;
}

int AmdIsp4CameraData::loadPlatformConfiguration() {
	std::string filename("config-default.yaml");

	char const *envVar = utils::secure_getenv("LIBCAMERA_AMD_CONFIG_FILE");
	if (!envVar || !strlen(envVar)) {
		LOG(AmdIsp4, Warning)
			<< "Env var LIBCAMERA_AMD_CONFIG_FILE not found";
		LOG(AmdIsp4, Warning)
			<< "Using default config file '" << filename << "'";
	} else {
		filename = envVar;
	}

	struct stat statbuf;
	envVar = utils::secure_getenv("LIBCAMERA_AMD_PIPELINE_CONFIG_PATH");
	if (envVar && strlen(envVar)) {
		for (const auto &path : utils::split(envVar, ":")) {
			if (path.empty())
				continue;

			std::string configPath = path + "/" + filename;

			int ret = stat(configPath.c_str(), &statbuf);
			if (ret == 0 && (statbuf.st_mode & S_IFMT) == S_IFREG) {
				configurationFile_ = std::move(configPath);
				LOG(AmdIsp4, Debug)
					<< "Found AMD config file '"
					<< configurationFile_
					<< "'";
				return 0;
			}
		}
	}

	/*
	 * If libcamera wasn't installed, check the source directory for the
	 * config file
	 */
	PipelineHandlerAmdIsp4 *pipelineHandler { dynamic_cast<PipelineHandlerAmdIsp4*>(pipe()) };
	std::string root = utils::libcameraSourcePath();
	if (!root.empty()) {
		std::string configPath = root + "src/libcamera/pipeline/amd/" + pipelineHandler->name() + "/data" + "/" + filename;

		LOG(AmdIsp4, Debug)
			<< "libcamera is not installed. Loading pipeline handler configuration from source code directory '"
			<< configPath << "'";
		int ret = stat(configPath.c_str(), &statbuf);
		if (ret == 0 && (statbuf.st_mode & S_IFMT) == S_IFREG) {
			configurationFile_ = std::move(configPath);
			LOG(AmdIsp4, Debug)
				<< "Found AMD config file from the source code directory '"
				<< configurationFile_
				<< "'";
			return 0;
		}
	}

	/*
	 * As a last resort, look in the system location
	 */
	for (const auto &systemDir : utils::split(LIBCAMERA_DATA_DIR, ":")) {
		std::string configPath = systemDir + "/pipeline/amd/" + pipelineHandler->name() + "/" + filename;

		LOG(AmdIsp4, Debug)
			<< "Loading pipeline handler configuration from system directory '"
			<< configPath << "'";
		int ret = stat(configPath.c_str(), &statbuf);
		if (ret == 0 && (statbuf.st_mode & S_IFMT) == S_IFREG) {
			configurationFile_ = std::move(configPath);
			LOG(AmdIsp4, Debug)
				<< "Found AMD config file from the system directory '"
				<< configurationFile_
				<< "'";
			return 0;
		}
	}

	LOG(AmdIsp4, Error)
		<< "Sensor config file not found";
	return -EINVAL;
}

int AmdIsp4CameraData::parsePlatformConfiguration(std::unique_ptr<YamlObject> &yamlObject) {
	File file(configurationFile_);

	if (!file.open(File::OpenModeFlag::ReadOnly)) {
		LOG(AmdIsp4, Error)
			<< "Failed to open configuration file '"
			<< configurationFile_ << "'";
		return -ENOENT;
	}

	std::unique_ptr<YamlObject> root = YamlParser::parse(file);
	if (!root) {
		LOG(AmdIsp4, Error)
			<< "Failed to parse configuration file";
		return -EINVAL;
	}

	std::optional<double> ver = (*root)["version"].get<double>(0);
	if (!ver || *ver != 1.0) {
		LOG(AmdIsp4, Error)
			<< "Unexpected configuration file version reported: "
			<< *ver;
		return -EINVAL;
	}

	
	if (!root->contains(PipelineHandlerAmdIsp4::name())) {
		LOG(AmdIsp4, Error)
			<< "Config file not for target '"
			<< PipelineHandlerAmdIsp4::name() << "'";
		return -EINVAL;
	}

	yamlObject = std::move(root);
	return 0;
}

int AmdIsp4CameraData::configureIspProperties(const YamlObject &ispRoot) {
	if (!ispRoot.isList()) {
		LOG(AmdIsp4, Error)
			<< "Invalid isp root object";		
		return -EINVAL;
	}
	
	for (const auto &entityName : AmdIsp4CameraData::supportedEntityNames()) {
		AmdIsp4CameraData::VideoDevice *videoDevice {at(entityNameToRole(entityName))};

		if (!videoDevice) {
			LOG(AmdIsp4, Error)
				<< "Missing video device '"
				<< entityName << "'";		
			return -EINVAL;
		}
		for (const auto &configData : ispRoot.asList()) {
			if (configData.contains("entity name")) {
				std::string name { configData["entity name"].get<std::string>("") };
				if (name == entityName) {
					if (configData.contains("buffer count")) {
						videoDevice->bufferCount_ = configData["buffer count"].get<unsigned int>(0);
						LOG(AmdIsp4, Debug)
							<< "Buffer count on device '"
							<< entityName << "' is "
							<< videoDevice->bufferCount_;		
					}
					if (configData.contains("minimum resolution")) {
						videoDevice->minResolution_ = configData["minimum resolution"].get<Size>({ 0, 0 });
						LOG(AmdIsp4, Debug)
							<< "Buffer minimum resolution on device '"
							<< entityName << "' is "
							<< videoDevice->minResolution_;		
					}
					if (configData.contains("maximum resolution")) {
						videoDevice->maxResolution_ = configData["maximum resolution"].get<Size>({ 0, 0 });
						LOG(AmdIsp4, Debug)
							<< "Buffer maximum resolution on device '"
							<< entityName << "' is "
							<< videoDevice->maxResolution_;		
					}
					break;
				}
			}
		}
		if (videoDevice->bufferCount_ == 0 || videoDevice->minResolution_ == Size(0,0) || videoDevice->maxResolution_ == Size(0, 0)) {
			LOG(AmdIsp4, Error)
				<< "Failed to set device '"
				<< entityName << "' configuration";		
			return -EINVAL;
		}
	}
	return 0;
}

bool PipelineHandlerAmdIsp4::match(DeviceEnumerator *enumerator)
{
	std::vector<MediaDevice *> medias;
	for (const auto &deviceName : AmdIsp4CameraData::supportedDeviceNames()) {
		DeviceMatch dm(deviceName);
		for (const auto &entityName : AmdIsp4CameraData::supportedEntityNames()) {
			dm.add(entityName);
		}

		MediaDevice *media = acquireMediaDevice(enumerator, dm);
		if (media)
			medias.push_back(media);
		else
			return false;
	}

	if (!medias.empty()) {
		for (auto &media : medias) {
			if (createCamera(media) < 0)
				return false;
		}
	}
	return true;
}

int AmdCameraSensor::init(const YamlObject &sensorRoot) {
	return configureSensorProperties(sensorRoot);
}

int AmdCameraSensor::configureSensorProperties(const YamlObject &sensorRoot) {
	if (!sensorRoot.contains("sensor id")) {
		LOG(AmdIsp4, Error)
			<< "Invalid sensor root object";
		return -EINVAL;
	}

	id_ = sensorRoot["sensor id"].get<std::string>("");
	LOG(AmdIsp4, Info)
		<< "sensor id: '"
		<< id_ << "'";
	
	return 0;
}

int PipelineHandlerAmdIsp4::createCamera(MediaDevice *media)
{
	std::unique_ptr<AmdIsp4CameraData> data = std::make_unique<AmdIsp4CameraData>(this);
	std::set<Stream *> streams;

	/* Init the id since the sensor module isn't implemented yet */
	int ret{ data->init(media) };
	if (ret < 0)
		return ret;

	if (!data->size())
		return -ENODEV;

	for (auto &videoDevice : *data) {
		streams.insert(&videoDevice.stream());
		videoDevice.video()->bufferReady.connect(data.get(), &AmdIsp4CameraData::bufferReady);
	}

	ret = data->loadIPA();
	if (ret < 0)
		return ret;

	/* Create and register the camera. */
	std::string id = data->id();
	std::shared_ptr<Camera> camera =
		Camera::create(std::move(data), id, streams);
	registerCamera(std::move(camera));

	return ret;
}

std::unique_ptr<CameraConfiguration>
PipelineHandlerAmdIsp4::generateConfiguration(Camera *camera,
					  Span<const StreamRole> roles)
{
	AmdIsp4CameraData *data = cameraData(camera);

	if (roles.size() > data->size()) {
		LOG(AmdIsp4, Error)
			<< "Requested more roles than device supports";
		return nullptr;
	}

	std::unique_ptr<CameraConfiguration> cameraConfig =
		std::make_unique<AmdIsp4CameraConfiguration>();

	if (roles.empty())
		return cameraConfig;

	for (const StreamRole role : roles) {
		AmdIsp4CameraData::VideoDevice *videoDevice{ data->at(role) };
		if (videoDevice) {
			StreamFormats formats(videoDevice->pixelFormats());
			StreamConfiguration streamConfig(formats);

			LOG(AmdIsp4, Debug) << "For now, just use the first available format and size:";
			streamConfig.pixelFormat = formats.pixelformats().front();
			streamConfig.size = formats.sizes(streamConfig.pixelFormat).back();
			streamConfig.bufferCount = videoDevice->bufferCount();
			LOG(AmdIsp4, Debug)
				<< "pixel format: " << streamConfig.pixelFormat
				<< " size: " << streamConfig.size
				<< " buffer count " << streamConfig.bufferCount;

			cameraConfig->addConfiguration(streamConfig);
			dynamic_cast<AmdIsp4CameraConfiguration *>(cameraConfig.get())->linkVideoDevice(videoDevice);
		} else {
			LOG(AmdIsp4, Error)
				<< "Requested stream role '"
				<< role
				<< "'not supported.";
			return nullptr;
		}
	}

	if (cameraConfig->validate() == CameraConfiguration::Status::Invalid) {
		LOG(AmdIsp4, Error)
			<< "Failed to generate config, invalid configuration.";
		return nullptr;
	}

	return cameraConfig;
}

int AmdIsp4CameraData::init(MediaDevice *media)
{
	if (loadPlatformConfiguration()) {
		LOG(AmdIsp4, Error)
			<< "Failed to load platform configuration file.";
		return -ENOENT;
	}

	std::unique_ptr<YamlObject> root;
	if (parsePlatformConfiguration(root)) {
		LOG(AmdIsp4, Error)
			<< "Failed to parse configuration file.";
		return -ENOENT;
	}

	if (!root->contains("sensor")) {
		LOG(AmdIsp4, Error)
			<< "Missing sensor configurations.";
		return -ENOENT;
	}
	
	std::unique_ptr<AmdCameraSensor> sensor = std::make_unique<AmdCameraSensor>(nullptr);

	if (sensor && !sensor->init((*root)["sensor"])) {
		sensor_ = std::move(sensor);
	} else {
		LOG(AmdIsp4, Error)
			<< "Failed to create sensor.";
		return -ENOENT;
	}

	if (id_.empty() && sensor_)
		id_ = sensor_->id();

	PipelineHandlerAmdIsp4 *pipelineHandler { dynamic_cast<PipelineHandlerAmdIsp4*>(pipe()) };

	if (!root->contains(pipelineHandler->name())) {
		LOG(AmdIsp4, Error)
			<< "Missing isp4 configurations.";
		return -ENOENT;
	}

	for (const auto &entityName : AmdIsp4CameraData::supportedEntityNames()) {
		int ret = videoDeviceAdd(media, entityName);
		if (ret < 0) {
			LOG(AmdIsp4, Error)
				<< "Failed to add device '"
				<< entityName << "'";			
			return ret;
		}		
	}

	if (configureIspProperties((*root)[pipelineHandler->name()])) {
		LOG(AmdIsp4, Error)
			<< "Failed to configure isp4 devices";
		return -ENOENT;
	}

	return 0;
}

int AmdIsp4CameraData::videoDeviceAdd(MediaDevice *media, const std::string &entityName)
{
	std::unique_ptr<V4L2VideoDevice> video = V4L2VideoDevice::fromEntityName(media, entityName);

	if (!video)
		return -ENODEV;

	int ret = video->open();
	if (ret < 0)
		return ret;

	AmdIsp4CameraData::VideoDevice videoDevice(std::move(video));

	ret = videoDevice.populateFormats();
	if (ret < 0)
		return ret;

	nodeToRoleConverter_.emplace(videoDevice.video()->deviceNode(), entityNameToRole(entityName));
	videoDevices_.push_back(std::move(videoDevice));

	return ret;
}

AmdIsp4CameraConfiguration::AmdIsp4CameraConfiguration()
	: CameraConfiguration()
{
}

CameraConfiguration::Status AmdIsp4CameraConfiguration::validate()
{
	Status status = Status::Valid;

	if (config_.empty())
		return Status::Invalid;

	if (config_.size() != videoDeviceLinks_.size()) {
		LOG(AmdIsp4, Error)
			<< "Count mismatch between stream config and video devices";
		return Status::Invalid;
	}

	if (config_.size() > MAX_STREAMS) {
		LOG(AmdIsp4, Debug)
			<< "Restrict the config count to '"
			<< MAX_STREAMS
			<< "', the maximum supported concurrent streams.";
		config_.resize(MAX_STREAMS);
		videoDeviceLinks_.resize(MAX_STREAMS);
		status = Status::Adjusted;
	}

	std::vector<unsigned int> indexes(config_.size());
	std::iota(indexes.begin(), indexes.end(), 0);
	for (unsigned int index : indexes) {
		StreamConfiguration &streamConfig{ config_.at(index) };
		AmdIsp4CameraData::VideoDevice &videoDevice{ *videoDeviceLinks_.at(index) };
		V4L2DeviceFormat format;
		format.fourcc = videoDevice.video()->toV4L2PixelFormat(streamConfig.pixelFormat);
		format.size = streamConfig.size;

		if (videoDevice.video()->tryFormat(&format))
			return Status::Invalid;

		streamConfig.stride = format.planes[0].bpl;
		streamConfig.frameSize = format.planes[0].size;

		streamConfig.colorSpace = format.colorSpace;
		streamConfig.setStream(const_cast<Stream *>(&videoDevice.stream()));
	}

	return status;
}

PipelineHandlerAmdIsp4::PipelineHandlerAmdIsp4(CameraManager *manager)
	: PipelineHandler(manager)
{
}

int PipelineHandlerAmdIsp4::configure(Camera *camera, CameraConfiguration *config)
{
	AmdIsp4CameraConfiguration &cameraConfig{ *dynamic_cast<AmdIsp4CameraConfiguration *>(config) };
	AmdIsp4CameraData *data = cameraData(camera);
	PixelFormat ipaPixelFormat;
	Size ipaSize;

	if (config->size() != cameraConfig.videoDeviceLinkSize()) {
		LOG(AmdIsp4, Error)
			<< "Count mismatch between stream config and video devices";
		return -EINVAL;
	}

	std::vector<unsigned int> indexes(config->size());
	std::iota(indexes.begin(), indexes.end(), 0);

	for (auto index : indexes) {
		StreamConfiguration &streamConfig{ cameraConfig.at(index) };
		AmdIsp4CameraData::VideoDevice &videoDevice{ *cameraConfig.videoDeviceAt(index) };

		V4L2DeviceFormat format;
		format.fourcc = videoDevice.video()->toV4L2PixelFormat(streamConfig.pixelFormat);
		format.size = streamConfig.size;

		int ret = videoDevice.video()->setFormat(&format);
		if (ret)
			return ret;

		if (format.size != streamConfig.size ||
		    format.fourcc != videoDevice.video()->toV4L2PixelFormat(streamConfig.pixelFormat))
			return -EINVAL;

		LOG(AmdIsp4, Debug) << "For now use any config since the IPA is not used";
		ipaPixelFormat = streamConfig.pixelFormat;
		ipaSize = streamConfig.size;
	}

	if (data->ipa()) {
		std::map<unsigned int, IPAStream> ipaStreams;
		ipaStreams.emplace(std::piecewise_construct,
				   std::forward_as_tuple(0),
				   std::forward_as_tuple(ipaPixelFormat, ipaSize));

		std::map<unsigned int, ControlInfoMap> entityControls;

		if (data->ipa())
			data->ipa()->configure({}, ipaStreams, entityControls);
	}

	return 0;
}

int PipelineHandlerAmdIsp4::exportFrameBuffers(Camera *camera, Stream *stream,
					       std::vector<std::unique_ptr<FrameBuffer>> *buffers)
{
	AmdIsp4CameraData *data = cameraData(camera);
	unsigned int count = stream->configuration().bufferCount;

	AmdIsp4CameraData::VideoDevice *videoDevice{ data->at(stream) };

	if (!videoDevice)
		return -EINVAL;

	return videoDevice->video()->exportBuffers(count, buffers);
}

int PipelineHandlerAmdIsp4::start(Camera *camera, [[maybe_unused]] const ControlList *controls)
{
	AmdIsp4CameraData *data = cameraData(camera);
	for (auto &stream : camera->streams()) {
		AmdIsp4CameraData::VideoDevice *videoDevice{ data->at(stream) };

		if (!videoDevice)
			return -ENODEV;

		unsigned int bufferCount = videoDevice->stream().configuration().bufferCount;

		if (!bufferCount) {
			LOG(AmdIsp4, Error)
				<< "buffer count zero, skip non-configured stream";
			continue;
		}

		int ret = videoDevice->video()->importBuffers(bufferCount);
		if (ret < 0)
			return ret;

		if (data->ipa()) {
			/* Map the mock IPA buffers to Amd Isp4 IPA to exercise IPC code paths. */
			std::vector<IPABuffer> ipaBuffers;
			for (auto [i, buffer] : utils::enumerate(data->mockIPABufs_)) {
				buffer->setCookie(i + 1);
				ipaBuffers.emplace_back(buffer->cookie(), buffer->planes());
			}
			data->ipa()->mapBuffers(ipaBuffers);

			ret = data->ipa()->start();
			if (ret) {
				videoDevice->video()->releaseBuffers();
				return ret;
			}
		}

		ret = videoDevice->video()->streamOn();
		if (ret < 0) {
			if (data->ipa())
				data->ipa()->stop();
			videoDevice->video()->releaseBuffers();
			return ret;
		}
	}

	return 0;
}

void PipelineHandlerAmdIsp4::stopDevice(Camera *camera)
{
	AmdIsp4CameraData *data = cameraData(camera);

	for (auto &videoDevice : *data) {
		videoDevice.video()->streamOff();

		std::vector<unsigned int> ids;
		for (const std::unique_ptr<FrameBuffer> &buffer : data->mockIPABufs_)
			ids.push_back(buffer->cookie());
		if (data->ipa()) {
			data->ipa()->unmapBuffers(ids);
			data->ipa()->stop();
		}

		videoDevice.video()->releaseBuffers();
	}
}

void AmdIsp4CameraData::bufferReady(FrameBuffer *buffer)
{
	Request *request = buffer->request();

	LOG(AmdIsp4, Debug)
		<< "Use the buffer time stamp '"
		<< buffer->metadata().timestamp
		<< "' for now until the real timestamp from the frame metadata is implemented";
	request->metadata().set(controls::SensorTimestamp,
				buffer->metadata().timestamp);

	pipe()->completeBuffer(request, buffer);
	pipe()->completeRequest(request);

	if (ipa_)
		ipa_->fillParamsBuffer(request->sequence(), mockIPABufs_[0]->cookie());
}

int AmdIsp4CameraData::allocateMockIPABuffers()
{
	std::vector<FrameBuffer::Plane> planes;
	mockIPABufs_.push_back(std::make_unique<FrameBuffer>(planes));

	return 0;
}

void AmdIsp4CameraData::paramsBufferReady([[maybe_unused]] unsigned int id,
					  [[maybe_unused]] const Flags<ipa::amd::TestFlag> flags)
{
	LOG(AmdIsp4, Debug)
		<< "Do nothing for now since the IPA is not used";
}

int PipelineHandlerAmdIsp4::queueRequestDevice(Camera *camera, Request *request)
{
	AmdIsp4CameraData *data = cameraData(camera);
	bool requestQueued{ false };

	for (auto &videoDevice : *data) {
		FrameBuffer *buffer = request->findBuffer(&videoDevice.stream());
		if (!buffer)
			continue;

		int ret = videoDevice.video()->queueBuffer(buffer);
		if (ret < 0) {
			LOG(AmdIsp4, Error)
				<< "Failed to queue buffer";
			return ret;
		}
		requestQueued = true;
	}

	if (!requestQueued) {
		LOG(AmdIsp4, Error)
			<< "Attempt to queue request with invalid stream";
		return -ENOENT;
	}

	if (data->ipa())
		data->ipa()->queueRequest(request->sequence(), request->controls());

	return 0;
}

int AmdIsp4CameraData::loadIPA()
{
	if (!ipaFunctional()) {
		LOG(AmdIsp4, Warning)
			<< "IPA not function yet, skip loading it";
		return 0;
	}

	std::unique_ptr<ipa::amd::IPAProxyAmdIsp4> ipa { IPAManager::createIPA<ipa::amd::IPAProxyAmdIsp4>(pipe(), 0, 0) };
	if (!ipa) {
		LOG(AmdIsp4, Error) << "no matching IPA found";
		return -EINVAL;
	}

	ipa->paramsBufferReady.connect(this, &AmdIsp4CameraData::paramsBufferReady);

	std::string tuningFilename;
	char const *envVar { utils::secure_getenv("LIBCAMERA_AMD_TUNING_FILE") };
	if (envVar && std::strlen(envVar))
		tuningFilename = ipa->configurationFile(envVar);

	if (tuningFilename.empty()) {
		LOG(AmdIsp4, Debug)
			<< "Tuning file from env not found, trying default for the camera model '"
			<< id()
			<< "'";
		tuningFilename = ipa->configurationFile(id() + ".json");
	}

	if (tuningFilename.empty()) {
		LOG(AmdIsp4, Debug)
			<< "Default tuning file for the camera model '"
			<< id()
			<< "' not found, trying the generinc tuning file";
		tuningFilename = ipa->configurationFile("generic-tuning.json");
	}

	if (tuningFilename.empty())
		return -EINVAL;

	Flags<ipa::amd::TestFlag> inFlags = ipa::amd::TestFlag::Flag2;
	Flags<ipa::amd::TestFlag> outFlags;
	int ret{ ipa->init(IPASettings{ tuningFilename, {} },
			    ipa::amd::IPAOperationInit, inFlags, &outFlags) };
	if (ret < 0)
		return ret;

	LOG(AmdIsp4, Debug)
		<< "Flag 1 was "
		<< (outFlags & ipa::amd::TestFlag::Flag1 ? "" : "not ")
		<< "set";
	ret = allocateMockIPABuffers();
	if (ret < 0) {
		LOG(AmdIsp4, Warning) << "Cannot allocate mock IPA buffers";
		return ret;
	}

	ipa_ = std::move(ipa);

	return ret;
}

REGISTER_PIPELINE_HANDLER(PipelineHandlerAmdIsp4)

} /* namespace libcamera */
