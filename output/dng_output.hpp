/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2024
 *
 * dng_output.hpp - Write raw frames as a CinemaDNG sequence.
 */

#pragma once

#include <condition_variable>
#include <mutex>
#include <queue>
#include <string>

#include <libcamera/controls.h>

#include "core/stream_info.hpp"
#include "core/video_options.hpp"

#include "output.hpp"

class DngOutput : public Output
{
public:
        DngOutput(VideoOptions const *options, StreamInfo const &info, std::string camera_model);

        void outputBuffer(void *mem, size_t size, int64_t timestamp_us, uint32_t flags) override;
        void MetadataReady(libcamera::ControlList &metadata);

private:
        std::string nextFilename();
        libcamera::ControlList waitForMetadata();

        StreamInfo info_;
        std::string camera_model_;
        std::string filename_pattern_;
        std::mutex file_mutex_;
        unsigned int frame_index_ = 0;

        std::mutex metadata_mutex_;
        std::condition_variable metadata_cv_;
        std::queue<libcamera::ControlList> metadata_queue_;
};
