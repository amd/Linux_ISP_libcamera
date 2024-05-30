# SPDX-License-Identifier: CC0-1.0

tuning_files = files([
    'tuning-generic.json',
    'tuning-amd_isp41_mdev.json',
])

tuning_filenames = [
    'tuning-generic.json',
    'tuning-amd_isp41_mdev.json',
]

custom_target('copy tuning',
              input : tuning_filenames,
              output :  '.',
              command : ['cp', '@INPUT@', '@OUTPUT@'],
              install : false,
              build_by_default : true)

install_data(tuning_files,
             install_dir : ipa_data_dir / 'amd' / 'isp4',
             install_tag : 'runtime')
