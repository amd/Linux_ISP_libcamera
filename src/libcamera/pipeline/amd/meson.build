# SPDX-License-Identifier: CC0-1.0

foreach pipeline : pipelines
    pipeline = pipeline.split('/')
    if pipeline.length() < 2 or pipeline[0] != 'amd'
        continue
    endif

    subdir(pipeline[1])
endforeach
