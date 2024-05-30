/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 * Copyright (C) 2024, Advanced Micro Devices Inc.
 *
 * isp4.cpp - AMD ISP4 Image Processing Algorithm module
 */
#include <libcamera/ipa/amd_ipa_interface.h>

#include <fcntl.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include <iostream>

#include <libcamera/base/file.h>
#include <libcamera/base/log.h>

#include <libcamera/ipa/ipa_interface.h>
#include <libcamera/ipa/ipa_module_info.h>

#include "libcamera/internal/mapped_framebuffer.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(IPAAmdIsp4)

class IPAAmdIsp4 : public ipa::amd::IPAAmdIsp4Interface
{
public:
	IPAAmdIsp4();
	~IPAAmdIsp4();

	int init(const IPASettings &settings,
		 const ipa::amd::IPAOperationCode code,
		 const Flags<ipa::amd::TestFlag> inFlags,
		 Flags<ipa::amd::TestFlag> *outFlags) override;

	int start() override;
	void stop() override;

	int configure(const IPACameraSensorInfo &sensorInfo,
		      const std::map<unsigned int, IPAStream> &streamConfig,
		      const std::map<unsigned int, ControlInfoMap> &entityControls) override;

	void mapBuffers(const std::vector<IPABuffer> &buffers) override;
	void unmapBuffers(const std::vector<unsigned int> &ids) override;

	void queueRequest(uint32_t frame, const ControlList &controls) override;
	void fillParamsBuffer(uint32_t frame, uint32_t bufferId) override;

private:
	void initTrace();
	void trace(enum ipa::amd::IPAOperationCode operation);

	int fd_;
	std::map<unsigned int, MappedFrameBuffer> buffers_;
};

IPAAmdIsp4::IPAAmdIsp4()
	: fd_(-1)
{
	initTrace();
}

IPAAmdIsp4::~IPAAmdIsp4()
{
	if (fd_ != -1)
		::close(fd_);
}

int IPAAmdIsp4::init(const IPASettings &settings,
		  const ipa::amd::IPAOperationCode code,
		  const Flags<ipa::amd::TestFlag> inFlags,
		  Flags<ipa::amd::TestFlag> *outFlags)
{
	trace(ipa::amd::IPAOperationInit);

	LOG(IPAAmdIsp4, Debug)
		<< "initializing amdisp4 IPA with configuration file "
		<< settings.configurationFile;

	LOG(IPAAmdIsp4, Debug) << "Got opcode " << code;

	LOG(IPAAmdIsp4, Debug)
		<< "Flag 2 was "
		<< (inFlags & ipa::amd::TestFlag::Flag2 ? "" : "not ")
		<< "set";

	*outFlags |= ipa::amd::TestFlag::Flag1;

	File conf(settings.configurationFile);
	if (!conf.open(File::OpenModeFlag::ReadOnly)) {
		LOG(IPAAmdIsp4, Error) << "Failed to open configuration file";
		LOG(IPAAmdIsp4, Error) << "Skip error for now since the IPA doesn't do anything yet.";
		//return -EINVAL;
	}

	return 0;
}

int IPAAmdIsp4::start()
{
	trace(ipa::amd::IPAOperationStart);

	LOG(IPAAmdIsp4, Debug) << "start amdisp4 IPA!";

	return 0;
}

void IPAAmdIsp4::stop()
{
	trace(ipa::amd::IPAOperationStop);

	LOG(IPAAmdIsp4, Debug) << "stop amdisp4 IPA!";
}

int IPAAmdIsp4::configure([[maybe_unused]] const IPACameraSensorInfo &sensorInfo,
			[[maybe_unused]] const std::map<unsigned int, IPAStream> &streamConfig,
			[[maybe_unused]] const std::map<unsigned int, ControlInfoMap> &entityControls)
{
	LOG(IPAAmdIsp4, Debug) << "configure()";

	return 0;
}

void IPAAmdIsp4::mapBuffers(const std::vector<IPABuffer> &buffers)
{
	for (const IPABuffer &buffer : buffers) {
		const FrameBuffer fb(buffer.planes);
		buffers_.emplace(std::piecewise_construct,
				 std::forward_as_tuple(buffer.id),
				 std::forward_as_tuple(&fb, MappedFrameBuffer::MapFlag::Read));
	}
}

void IPAAmdIsp4::unmapBuffers(const std::vector<unsigned int> &ids)
{
	for (unsigned int id : ids) {
		auto it = buffers_.find(id);
		if (it == buffers_.end())
			continue;

		buffers_.erase(it);
	}
}

void IPAAmdIsp4::queueRequest([[maybe_unused]] uint32_t frame,
			   [[maybe_unused]] const ControlList &controls)
{
}

void IPAAmdIsp4::fillParamsBuffer([[maybe_unused]] uint32_t frame, uint32_t bufferId)
{
	auto it = buffers_.find(bufferId);
	if (it == buffers_.end()) {
		LOG(IPAAmdIsp4, Error) << "Could not find parameter buffer";
		return;
	}

	Flags<ipa::amd::TestFlag> flags;
	paramsBufferReady.emit(bufferId, flags);
}

void IPAAmdIsp4::initTrace()
{
	struct stat fifoStat;
	int ret = stat(ipa::amd::AmdIPAFIFOPath.c_str(), &fifoStat);
	if (ret)
		return;

	ret = ::open(ipa::amd::AmdIPAFIFOPath.c_str(), O_WRONLY | O_CLOEXEC);
	if (ret < 0) {
		ret = errno;
		LOG(IPAAmdIsp4, Error) << "Failed to open AMD ISP4 IPA test FIFO: "
				    << strerror(ret);
		return;
	}

	fd_ = ret;
}

void IPAAmdIsp4::trace(enum ipa::amd::IPAOperationCode operation)
{
	if (fd_ < 0)
		return;

	int ret = ::write(fd_, &operation, sizeof(operation));
	if (ret < 0) {
		ret = errno;
		LOG(IPAAmdIsp4, Error) << "Failed to write to amdisp4 IPA test FIFO: "
				    << strerror(ret);
	}
}

/*
 * External IPA module interface
 */

extern "C" {
const struct IPAModuleInfo ipaModuleInfo = {
	IPA_MODULE_API_VERSION,
	0,
	"PipelineHandlerAmdIsp4",
	"amdisp4",
};

IPAInterface *ipaCreate()
{
	return new IPAAmdIsp4();
}
}

} /* namespace libcamera */
