# SPDX-License-Identifier: CC0-1.0

conf_files = files([
    'config-default.yaml',
    'config-isp4-ov05c.yaml',
])

conf_filenames = [
    'config-default.yaml',
    'config-isp4-ov05c.yaml',
]

custom_target('copy conf',
              input : conf_filenames,
              output :  '.',
              command : ['cp', '-pP', '@INPUT@', '@OUTPUT@'],
              install : false,
              build_by_default : true)

install_data(conf_files,
             install_dir : pipeline_data_dir / 'amd' / 'isp4',
             install_tag : 'runtime')
