/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2024
 *
 * dng_output.cpp - Write raw frames as a CinemaDNG sequence.
 */

#include "dng_output.hpp"

#include <sys/stat.h>

#include <array>
#include <cstdio>
#include <stdexcept>
#include <vector>

#include <libcamera/base/span.h>

#include "core/logging.hpp"
#include "image/image.hpp"

namespace
{

std::string derive_pattern(const std::string &output)
{
        if (output.empty())
                return "frame-%08u.dng";

        if (output.find('%') != std::string::npos)
                return output;

        struct stat st = {};
        if (stat(output.c_str(), &st) == 0 && S_ISDIR(st.st_mode))
        {
                std::string sep = output.back() == '/' ? "" : "/";
                return output + sep + "frame-%08u.dng";
        }

        auto dot = output.rfind('.');
        if (dot != std::string::npos)
                return output.substr(0, dot) + "-%08u" + output.substr(dot);

        return output + "-%08u.dng";
}

} // namespace

DngOutput::DngOutput(VideoOptions const *options, StreamInfo const &info, std::string camera_model)
        : Output(options), info_(info), camera_model_(std::move(camera_model)),
          filename_pattern_(derive_pattern(options->output))
{
        if (options->output == "-")
                throw std::runtime_error("CinemaDNG output does not support writing to stdout");
}

void DngOutput::MetadataReady(libcamera::ControlList &metadata)
{
        {
                std::lock_guard<std::mutex> lock(metadata_mutex_);
                metadata_queue_.push(metadata);
        }
        metadata_cv_.notify_one();

        Output::MetadataReady(metadata);
}

libcamera::ControlList DngOutput::waitForMetadata()
{
        std::unique_lock<std::mutex> lock(metadata_mutex_);
        metadata_cv_.wait(lock, [this]() { return !metadata_queue_.empty(); });
        libcamera::ControlList metadata = metadata_queue_.front();
        metadata_queue_.pop();
        return metadata;
}

std::string DngOutput::nextFilename()
{
        std::lock_guard<std::mutex> lock(file_mutex_);
        std::array<char, 512> filename {};
        int n = snprintf(filename.data(), filename.size(), filename_pattern_.c_str(), frame_index_);
        if (n < 0 || n >= static_cast<int>(filename.size()))
                throw std::runtime_error("failed to compose CinemaDNG filename");

        frame_index_++;
        if (options_->wrap)
                frame_index_ = frame_index_ % options_->wrap;

        return std::string(filename.data(), n);
}

void DngOutput::outputBuffer(void *mem, size_t size, int64_t, uint32_t)
{
        std::vector<libcamera::Span<uint8_t>> spans;
        spans.emplace_back(static_cast<uint8_t *>(mem), size);

        libcamera::ControlList metadata = waitForMetadata();
        std::string filename = nextFilename();

        LOG(2, "Writing CinemaDNG frame to " << filename);
        dng_save(spans, info_, metadata, filename, camera_model_, nullptr);

        if (frame_written_callback_)
                frame_written_callback_(filename);
}

