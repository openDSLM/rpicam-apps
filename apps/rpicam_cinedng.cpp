/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2024
 *
 * rpicam_cinedng.cpp - Record raw frames as a CinemaDNG sequence with MJPEG preview.
 */

#include <chrono>
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
        unsigned int frames_captured = 0;

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
                        frames_captured = 0;
                        continue;
                }

                frames_captured++;
        }

        app.StopCamera();
        app.StopEncoder();
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

