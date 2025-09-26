/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2024
 *
 * rpicam_cinedng.cpp - Record raw frames as a CinemaDNG sequence with MJPEG preview.
 */

#include <chrono>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include "core/rpicam_encoder.hpp"
#include "encoder/null_encoder.hpp"
#include "output/dng_output.hpp"

using namespace std::placeholders;

class LibcameraCineDng : public RPiCamEncoder
{
protected:
        void createEncoder() override { encoder_ = std::unique_ptr<Encoder>(new NullEncoder(GetOptions())); }
};

static void event_loop(LibcameraCineDng &app)
{
        VideoOptions const *options = app.GetOptions();

        app.OpenCamera();
        app.ConfigureVideo(RPiCamApp::FLAG_VIDEO_RAW);

        StreamInfo raw_info;
        libcamera::Stream *raw_stream = app.RawStream(&raw_info);
        if (!raw_stream)
                throw std::runtime_error("Raw stream unavailable - cannot write CinemaDNG");

        std::unique_ptr<DngOutput> output =
                std::make_unique<DngOutput>(options, raw_info, app.CameraModel());
        app.SetEncodeOutputReadyCallback(std::bind(&DngOutput::OutputReady, output.get(), _1, _2, _3, _4));
        app.SetMetadataReadyCallback(std::bind(&DngOutput::MetadataReady, output.get(), _1));

        app.StartEncoder();
        app.StartCamera();

        auto start_time = std::chrono::high_resolution_clock::now();
        auto capture_start_time = start_time;
        auto stats_start_time = start_time;
        unsigned int frames_captured = 0;
        unsigned int stats_frames_captured = 0;
        uint64_t total_frames_captured = 0;

        while (true)
        {
                RPiCamEncoder::Msg msg = app.Wait();

                if (msg.type == RPiCamApp::MsgType::Timeout)
                {
                        LOG_ERROR("ERROR: Device timeout detected, attempting a restart");
                        app.StopCamera();
                        app.StartCamera();
                        continue;
                }

                if (msg.type == RPiCamEncoder::MsgType::Quit)
                        break;

                if (msg.type != RPiCamEncoder::MsgType::RequestComplete)
                        throw std::runtime_error("unrecognised message");

                auto now = std::chrono::high_resolution_clock::now();
                bool timeout = options->timeout && (now - start_time) > options->timeout.value;
                bool frame_limit = options->frames && frames_captured >= options->frames;
                if (timeout || frame_limit)
                {
                        if (timeout)
                                LOG(1, "Halting: reached timeout of "
                                                   << options->timeout.get<std::chrono::milliseconds>() << " milliseconds");
                        app.StopCamera();
                        break;
                }

                CompletedRequestPtr &completed_request = std::get<CompletedRequestPtr>(msg.payload);

                if (app.VideoStream())
                        app.ShowPreview(completed_request, app.VideoStream());

                if (!app.EncodeBuffer(completed_request, raw_stream))
                {
                        start_time = std::chrono::high_resolution_clock::now();
                        stats_start_time = start_time;
                        frames_captured = 0;
                        stats_frames_captured = 0;
                        continue;
                }

                total_frames_captured++;
                frames_captured++;
                stats_frames_captured++;

                now = std::chrono::high_resolution_clock::now();
                auto stats_elapsed = now - stats_start_time;
                if (stats_elapsed >= std::chrono::seconds(1))
                {
                        double interval_seconds =
                                std::chrono::duration_cast<std::chrono::duration<double>>(stats_elapsed).count();
                        double interval_fps = interval_seconds > 0.0 ? stats_frames_captured / interval_seconds : 0.0;
                        std::ostringstream stats_stream;
                        stats_stream << std::fixed << std::setprecision(2) << interval_fps;
                        LOG(1, "CinemaDNG write rate: " << stats_stream.str() << " fps over " << stats_frames_captured
                                                        << " frames (total " << total_frames_captured << ")");
                        stats_start_time = now;
                        stats_frames_captured = 0;
                }
        }

        app.StopCamera();
        app.StopEncoder();

        auto capture_end_time = std::chrono::high_resolution_clock::now();
        auto capture_duration = capture_end_time - capture_start_time;
        if (total_frames_captured > 0 && capture_duration > std::chrono::nanoseconds(0))
        {
                double total_seconds =
                        std::chrono::duration_cast<std::chrono::duration<double>>(capture_duration).count();
                double average_fps = total_seconds > 0.0 ? total_frames_captured / total_seconds : 0.0;
                std::ostringstream summary_stream;
                summary_stream << std::fixed << std::setprecision(2) << average_fps;
                LOG(1, "CinemaDNG session average: " << summary_stream.str() << " fps across "
                                                    << total_frames_captured << " frames in "
                                                    << std::chrono::duration_cast<std::chrono::milliseconds>(capture_duration)
                                                               .count()
                                                    << " ms");
        }
}

int main(int argc, char *argv[])
{
        try
        {
                LibcameraCineDng app;
                VideoOptions *options = app.GetOptions();
                if (options->Parse(argc, argv))
                {
                        options->no_raw = false;
                        if (options->verbose >= 2)
                                options->Print();

                        event_loop(app);
                }
        }
        catch (std::exception const &e)
        {
                LOG_ERROR("ERROR: *** " << e.what() << " ***");
                return -1;
        }
        return 0;
}

