/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * camera_daemon.cpp - High level controller for the rpicam daemon.
 */

#include "daemon/camera_daemon.hpp"

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include <jpeglib.h>
#include <libcamera/base/span.h>
#include <libcamera/formats.h>

#include "core/logging.hpp"
#include "core/rpicam_encoder.hpp"
#include "core/stream_info.hpp"
#include "encoder/null_encoder.hpp"
#include "output/dng_output.hpp"

namespace rpicam
{
namespace
{

class LibcameraCineDng : public RPiCamEncoder
{
protected:
        void createEncoder() override { encoder_ = std::unique_ptr<Encoder>(new NullEncoder(GetOptions())); }
};

class PreviewEncoder : public RPiCamEncoder
{
protected:
        void createEncoder() override { encoder_ = std::unique_ptr<Encoder>(new NullEncoder(GetOptions())); }
};

std::vector<uint8_t> encodeFrameToJpeg(libcamera::Span<uint8_t> span, StreamInfo const &info, int quality)
{
        if (!span.size())
                return {};

        if (info.width == 0 || info.height == 0 || info.stride == 0)
                throw std::runtime_error("Invalid stream info for preview frame");

        struct jpeg_compress_struct cinfo;
        struct jpeg_error_mgr jerr;
        cinfo.err = jpeg_std_error(&jerr);
        jpeg_create_compress(&cinfo);

        unsigned char *encoded_buffer = nullptr;
        unsigned long encoded_size = 0; // NOLINT: libjpeg API uses unsigned long
        jpeg_mem_dest(&cinfo, &encoded_buffer, &encoded_size);

        cinfo.image_width = info.width;
        cinfo.image_height = info.height;
        cinfo.input_components = 3;
        cinfo.in_color_space = JCS_YCbCr;
        cinfo.restart_interval = 0;
        jpeg_set_defaults(&cinfo);
        cinfo.raw_data_in = TRUE;
        jpeg_set_quality(&cinfo, quality, TRUE);

        cinfo.comp_info[0].h_samp_factor = 2;
        cinfo.comp_info[0].v_samp_factor = 2;
        cinfo.comp_info[1].h_samp_factor = 1;
        cinfo.comp_info[1].v_samp_factor = 1;
        cinfo.comp_info[2].h_samp_factor = 1;
        cinfo.comp_info[2].v_samp_factor = 1;

        jpeg_start_compress(&cinfo, TRUE);

        int stride2 = info.stride / 2;
        uint8_t *Y = span.data();
        uint8_t *U = Y + info.stride * info.height;
        uint8_t *V = U + stride2 * (info.height / 2);
        uint8_t *Y_max = U - info.stride;
        uint8_t *U_max = V - stride2;
        uint8_t *V_max = U_max + stride2 * (info.height / 2);

        JSAMPROW y_rows[16];
        JSAMPROW u_rows[8];
        JSAMPROW v_rows[8];

        for (uint8_t *Y_row = Y, *U_row = U, *V_row = V; cinfo.next_scanline < info.height;)
        {
                for (int i = 0; i < 16; ++i, Y_row += info.stride)
                        y_rows[i] = std::min(Y_row, Y_max);
                for (int i = 0; i < 8; ++i, U_row += stride2, V_row += stride2)
                {
                        u_rows[i] = std::min(U_row, U_max);
                        v_rows[i] = std::min(V_row, V_max);
                }

                JSAMPARRAY rows[] = { y_rows, u_rows, v_rows };
                jpeg_write_raw_data(&cinfo, rows, 16);
        }

        jpeg_finish_compress(&cinfo);

        std::vector<uint8_t> encoded(encoded_buffer, encoded_buffer + encoded_size);
        jpeg_destroy_compress(&cinfo);
        free(encoded_buffer);
        return encoded;
}

} // namespace

CameraDaemon::CameraDaemon() : video_stop_flag_(false) {}

void CameraDaemon::start(uint16_t port)
{
        registerRoutes();
        server_.start(port);
}

void CameraDaemon::stop()
{
        server_.stop();
        std::string ignored;
        stopActiveVideo(ignored);
        joinFinishedVideoThread();
}

SessionState CameraDaemon::getState() const
{
        std::lock_guard<std::mutex> lock(mutex_);
        return session_;
}

CameraSettings CameraDaemon::getSettings() const
{
        std::lock_guard<std::mutex> lock(mutex_);
        return settings_;
}

bool CameraDaemon::updateSettings(JsonObject const &values, std::string &error_message)
{
        std::lock_guard<std::mutex> lock(mutex_);
        CameraSettings updated = settings_;

        if (auto it = values.find("fps"); it != values.end())
        {
                auto number = it->second.asNumber();
                if (!number || *number <= 0)
                {
                        error_message = "Invalid fps value";
                        return false;
                }
                updated.fps = *number;
        }

        if (auto it = values.find("shutter_us"); it != values.end())
        {
                auto number = it->second.asNumber();
                if (!number || *number < 0)
                {
                        error_message = "Invalid shutter_us value";
                        return false;
                }
                updated.shutter_us = *number;
        }

        if (auto it = values.find("analogue_gain"); it != values.end())
        {
                auto number = it->second.asNumber();
                if (!number || *number <= 0)
                {
                        error_message = "Invalid analogue_gain value";
                        return false;
                }
                updated.analogue_gain = *number;
        }

        if (auto it = values.find("auto_exposure"); it != values.end())
        {
                auto boolean = it->second.asBool();
                if (!boolean)
                {
                        error_message = "Invalid auto_exposure value";
                        return false;
                }
                updated.auto_exposure = *boolean;
        }

        if (auto it = values.find("output_dir"); it != values.end())
        {
                auto path = it->second.asString();
                if (!path || path->empty())
                {
                        error_message = "output_dir must be a non-empty string";
                        return false;
                }

                std::string ensure_error;
                if (!ensureOutputDirectory(*path, ensure_error))
                {
                        error_message = ensure_error;
                        return false;
                }

                updated.output_dir = *path;
        }

        settings_ = updated;
        session_.last_error.clear();
        return true;
}

bool CameraDaemon::startSession(SessionMode mode, std::string &error_message)
{
        if (mode == SessionMode::Still)
        {
                CameraSettings settings;
                {
                        std::lock_guard<std::mutex> lock(mutex_);
                        if (session_.active)
                        {
                                error_message = "A session is already active";
                                return false;
                        }
                        session_.mode = SessionMode::Still;
                        session_.active = true;
                        session_.last_error.clear();
                        settings = settings_;
                }

                CaptureResult result = runCineDngCapture(settings, true, nullptr);
                {
                        std::lock_guard<std::mutex> lock(mutex_);
                        session_.active = false;
                        session_.mode = SessionMode::None;
                        if (!result.success)
                        {
                                session_.last_error = result.error;
                                error_message = result.error;
                                return false;
                        }
                        last_capture_.type = "still";
                        last_capture_.frames = std::move(result.frames);
                }
                return true;
        }

        if (mode == SessionMode::Video)
        {
                joinFinishedVideoThread();

                CameraSettings settings;
                {
                        std::lock_guard<std::mutex> lock(mutex_);
                        if (session_.active)
                        {
                                error_message = "A session is already active";
                                return false;
                        }
                        session_.mode = SessionMode::Video;
                        session_.active = true;
                        session_.last_error.clear();
                        settings = settings_;
                        video_stop_flag_.store(false);
                }

                try
                {
                        video_thread_ = std::thread(&CameraDaemon::runVideoCapture, this, settings);
                }
                catch (std::exception const &ex)
                {
                        std::lock_guard<std::mutex> lock(mutex_);
                        session_.active = false;
                        session_.mode = SessionMode::None;
                        session_.last_error = ex.what();
                        error_message = ex.what();
                        return false;
                }

                return true;
        }

        if (mode == SessionMode::None)
        {
                if (stopActiveVideo(error_message))
                        return true;
                if (!error_message.empty())
                        return false;
                error_message.clear();
                return true;
        }

        error_message = "Unsupported mode";
        return false;
}

bool CameraDaemon::stopSession(std::string &error_message)
{
        if (stopActiveVideo(error_message))
                return true;
        std::lock_guard<std::mutex> lock(mutex_);
        if (!session_.active)
        {
                error_message = "No active session";
                return false;
        }

        session_.active = false;
        session_.mode = SessionMode::None;
        session_.last_error.clear();
        return true;
}

std::string CameraDaemon::buildStatusJson() const
{
        std::lock_guard<std::mutex> lock(mutex_);
        std::ostringstream json;
        json << "{\"state\":{"
             << "\"active\":" << (session_.active ? "true" : "false")
             << ",\"mode\":" << jsonString(modeToString(session_.mode))
             << ",\"last_error\":" << jsonString(session_.last_error)
             << "},\"settings\":" << buildSettingsJson(settings_)
             << ",\"last_capture\":";
        if (last_capture_.type.empty())
                json << "null";
        else
                json << buildCaptureJson(last_capture_);
        json << "}";
        return json.str();
}

std::string CameraDaemon::buildSettingsJson(CameraSettings const &settings)
{
        std::ostringstream json;
        json << "{"
             << "\"fps\":" << settings.fps
             << ",\"shutter_us\":" << settings.shutter_us
             << ",\"analogue_gain\":" << settings.analogue_gain
             << ",\"auto_exposure\":" << (settings.auto_exposure ? "true" : "false")
             << ",\"output_dir\":" << jsonString(settings.output_dir)
             << "}";
        return json.str();
}

std::string CameraDaemon::modeToString(SessionMode mode)
{
        switch (mode)
        {
        case SessionMode::None: return "none";
        case SessionMode::Still: return "still";
        case SessionMode::Video: return "video";
        }
        return "none";
}

std::string CameraDaemon::buildCaptureJson(CaptureSummary const &capture)
{
        std::ostringstream json;
        json << "{\"type\":" << jsonString(capture.type) << ",\"frames\":[";
        for (size_t i = 0; i < capture.frames.size(); ++i)
        {
                if (i)
                        json << ',';
                json << jsonString(capture.frames[i]);
        }
        json << "],\"count\":" << capture.frames.size() << "}";
        return json.str();
}

CameraDaemon::CaptureResult CameraDaemon::runCineDngCapture(CameraSettings const &settings, bool single_shot,
                                                            std::atomic<bool> *stop_flag)
{
        CaptureResult result;
        try
        {
                LibcameraCineDng app;
                VideoOptions *options = app.GetOptions();
                // Initialise all Options fields to safe defaults by parsing an empty CLI.
                // This avoids uninitialised members (e.g. lores sizes, camera index, denoise).
                char arg0[] = "rpicam-daemon";
                char *argv[] = { arg0 };
                int argc = 1;
                options->Parse(argc, argv);
                applySettingsToOptions(settings, *options, true);

                std::string ensure_error;
                if (!ensureOutputDirectory(options->output, ensure_error))
                        throw std::runtime_error(ensure_error);

                using namespace std::placeholders;

                app.OpenCamera();
                app.ConfigureVideo(RPiCamApp::FLAG_VIDEO_RAW);

                StreamInfo raw_info;
                libcamera::Stream *raw_stream = app.RawStream(&raw_info);
                if (!raw_stream)
                        throw std::runtime_error("Raw stream unavailable - cannot write CinemaDNG");

                std::unique_ptr<DngOutput> output =
                        std::make_unique<DngOutput>(options, raw_info, app.CameraModel());
                output->SetFrameWrittenCallback([&result](std::string const &path) {
                        result.frames.push_back(path);
                });

                app.SetEncodeOutputReadyCallback(
                        std::bind(&DngOutput::OutputReady, output.get(), _1, _2, _3, _4));
                app.SetMetadataReadyCallback(
                        std::bind(&DngOutput::MetadataReady, output.get(), _1));

                app.StartEncoder();
                bool encoder_started = true;
                app.StartCamera();
                bool camera_running = true;

                auto cleanup = [&]() {
                        if (camera_running)
                        {
                                app.StopCamera();
                                camera_running = false;
                        }
                        if (encoder_started)
                        {
                                app.StopEncoder();
                                encoder_started = false;
                        }
                };

                while (true)
                {
                        RPiCamEncoder::Msg msg = app.Wait();

                        if (msg.type == RPiCamApp::MsgType::Timeout)
                        {
                                LOG_ERROR("Device timeout detected, attempting a restart");
                                if (camera_running)
                                {
                                        app.StopCamera();
                                        camera_running = false;
                                }
                                app.StartCamera();
                                camera_running = true;
                                continue;
                        }

                        if (msg.type == RPiCamEncoder::MsgType::Quit)
                                break;

                        if (msg.type != RPiCamEncoder::MsgType::RequestComplete)
                                throw std::runtime_error("Unrecognised message from camera");

                        CompletedRequestPtr &completed_request = std::get<CompletedRequestPtr>(msg.payload);

                        if (!app.EncodeBuffer(completed_request, raw_stream))
                                continue;

                        if (single_shot && !result.frames.empty())
                        {
                                cleanup();
                                result.success = true;
                                return result;
                        }

                        if (stop_flag && stop_flag->load())
                        {
                                cleanup();
                                result.success = true;
                                return result;
                        }
                }

                cleanup();
                result.success = true;
        }
        catch (std::exception const &ex)
        {
                result.error = ex.what();
        }
        return result;
}

bool CameraDaemon::capturePreviewSnapshot(CameraSettings const &settings, std::vector<uint8_t> &jpeg,
                                           std::string &error)
{
        try
        {
                PreviewEncoder app;
                VideoOptions *options = app.GetOptions();
                // Ensure Options are default-initialised to avoid undefined values.
                char arg0[] = "rpicam-daemon";
                char *argv[] = { arg0 };
                int argc = 1;
                options->Parse(argc, argv);
                applySettingsToOptions(settings, *options, false);
                options->quality = 85;

                app.OpenCamera();
                app.ConfigureVideo();

                StreamInfo stream_info;
                libcamera::Stream *stream = app.VideoStream(&stream_info);
                if (!stream)
                        throw std::runtime_error("Video stream unavailable for preview");
                if (stream_info.pixel_format != libcamera::formats::YUV420)
                        throw std::runtime_error("Preview stream must be YUV420");

                std::mutex mutex;
                std::condition_variable cv;
                bool frame_ready = false;
                std::string callback_error;

                using namespace std::placeholders;
                app.SetEncodeOutputReadyCallback([&](void *mem, size_t size, int64_t, bool) {
                        try
                        {
                                libcamera::Span<uint8_t> span(static_cast<uint8_t *>(mem), size);
                                std::vector<uint8_t> encoded =
                                        encodeFrameToJpeg(span, stream_info, options->quality);
                                {
                                        std::lock_guard<std::mutex> lock(mutex);
                                        if (!frame_ready)
                                        {
                                                jpeg = std::move(encoded);
                                                frame_ready = true;
                                        }
                                }
                                cv.notify_all();
                        }
                        catch (std::exception const &ex)
                        {
                                {
                                        std::lock_guard<std::mutex> lock(mutex);
                                        callback_error = ex.what();
                                        frame_ready = true;
                                }
                                cv.notify_all();
                        }
                });
                app.SetMetadataReadyCallback([](libcamera::ControlList &) {});

                app.StartEncoder();
                bool encoder_started = true;
                app.StartCamera();
                bool camera_running = true;

                auto cleanup = [&]() {
                        if (camera_running)
                        {
                                app.StopCamera();
                                camera_running = false;
                        }
                        if (encoder_started)
                        {
                                app.StopEncoder();
                                encoder_started = false;
                        }
                };

                auto wait_for_frame = [&]() {
                        std::unique_lock<std::mutex> lock(mutex);
                        if (!frame_ready)
                                cv.wait_for(lock, std::chrono::milliseconds(500), [&]() { return frame_ready; });
                        if (!callback_error.empty())
                                throw std::runtime_error(callback_error);
                        if (!frame_ready)
                                throw std::runtime_error("Timed out waiting for preview frame");
                        if (jpeg.empty())
                                throw std::runtime_error("Preview frame is empty");
                };

                while (true)
                {
                        RPiCamEncoder::Msg msg = app.Wait();

                        if (msg.type == RPiCamApp::MsgType::Timeout)
                        {
                                LOG_ERROR("Device timeout detected while fetching preview, attempting restart");
                                if (camera_running)
                                {
                                        app.StopCamera();
                                        camera_running = false;
                                }
                                app.StartCamera();
                                camera_running = true;
                                continue;
                        }

                        if (msg.type == RPiCamEncoder::MsgType::Quit)
                                break;

                        if (msg.type != RPiCamEncoder::MsgType::RequestComplete)
                                throw std::runtime_error("Unrecognised message from camera");

                        CompletedRequestPtr &completed_request = std::get<CompletedRequestPtr>(msg.payload);

                        if (app.EncodeBuffer(completed_request, stream))
                        {
                                wait_for_frame();
                                break;
                        }
                }

                cleanup();
                return true;
        }
        catch (std::exception const &ex)
        {
                error = ex.what();
                return false;
        }
}

bool CameraDaemon::ensureOutputDirectory(std::string const &path, std::string &error_message)
{
        if (path.empty())
                return true;

        std::error_code ec;
        std::filesystem::path dir(path);
        if (std::filesystem::is_regular_file(dir, ec))
                return true;

        if (!std::filesystem::create_directories(dir, ec) && ec)
        {
                error_message = "Failed to create output directory: " + ec.message();
                return false;
        }
        return true;
}

void CameraDaemon::applySettingsToOptions(CameraSettings const &settings, VideoOptions &options, bool request_raw)
{
        options.timeout.value = std::chrono::nanoseconds(0);
        options.nopreview = true;
        options.preview = "0,0,0,0";
        options.preview_stream.clear();
        options.fullscreen = false;
        options.qt_preview = false;
        options.info_text.clear();
        options.width = 0;
        options.height = 0;
        options.viewfinder_width = 0;
        options.viewfinder_height = 0;
        options.lores_width = 0;
        options.lores_height = 0;
        options.lores_par = false;
        options.camera = 0;
        options.denoise = "auto";
        options.no_raw = !request_raw;
        options.output = request_raw ? settings.output_dir : std::string();
        options.framerate = settings.fps;

        if (settings.auto_exposure)
        {
                options.shutter.value = std::chrono::nanoseconds(0);
                options.gain = 0.0f;
        }
        else
        {
                options.shutter.value = std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::microseconds(static_cast<int64_t>(settings.shutter_us)));
                options.gain = settings.analogue_gain;
        }
}

void CameraDaemon::registerRoutes()
{
        server_.addHandler("GET", "/status", [this](HttpRequest const &) {
                HttpResponse response;
                response.body = buildStatusJson();
                return response;
        });

        // Compatibility alias: manage session via a single endpoint
        server_.addHandler("POST", "/session", [this](HttpRequest const &request) {
                HttpResponse response;
                std::string error;
                JsonObject values = parseJsonObject(request.body);
                auto it = values.find("mode");
                if (it == values.end())
                {
                        response.status_code = 400;
                        response.body = "{\"error\":\"Missing field: mode\"}";
                        return response;
                }
                auto mode_str = it->second.asString();
                if (!mode_str)
                {
                        response.status_code = 400;
                        response.body = "{\"error\":\"mode must be a string\"}";
                        return response;
                }
                SessionMode mode = SessionMode::None;
                if (*mode_str == "still")
                        mode = SessionMode::Still;
                else if (*mode_str == "video")
                        mode = SessionMode::Video;
                else if (*mode_str == "none")
                        mode = SessionMode::None;
                else
                {
                        response.status_code = 400;
                        response.body = "{\"error\":\"Unsupported mode\"}";
                        return response;
                }

                if (!startSession(mode, error))
                {
                        response.status_code = 409;
                        if (!error.empty())
                                response.body = "{\"error\":" + jsonString(error) + "}";
                        else
                                response.body = "{\"error\":\"Failed to start session\"}";
                        return response;
                }

                response.body = buildStatusJson();
                return response;
        });

        server_.addHandler("DELETE", "/session", [this](HttpRequest const &) {
                HttpResponse response;
                std::string error;
                if (!stopSession(error))
                {
                        response.status_code = 409;
                        response.body = "{\"error\":" + jsonString(error) + "}";
                        return response;
                }
                response.body = buildStatusJson();
                return response;
        });

        server_.addHandler("GET", "/settings", [this](HttpRequest const &) {
                HttpResponse response;
                CameraSettings settings = getSettings();
                response.body = buildSettingsJson(settings);
                return response;
        });

        server_.addHandler("POST", "/settings", [this](HttpRequest const &request) {
                HttpResponse response;
                std::string error;
                JsonObject values = parseJsonObject(request.body);
                if (!updateSettings(values, error))
                {
                        response.status_code = 400;
                        response.body = "{\"error\":" + jsonString(error) + "}";
                        return response;
                }
                response.body = buildSettingsJson(getSettings());
                return response;
        });

        server_.addHandler("POST", "/capture/still", [this](HttpRequest const &) {
                joinFinishedVideoThread();

                HttpResponse response;
                CameraSettings settings;
                {
                        std::lock_guard<std::mutex> lock(mutex_);
                        if (session_.active)
                        {
                                response.status_code = 409;
                                response.body = "{\"error\":\"A session is already active\"}";
                                return response;
                        }
                        session_.mode = SessionMode::Still;
                        session_.active = true;
                        session_.last_error.clear();
                        settings = settings_;
                }

                CaptureResult result = runCineDngCapture(settings, true, nullptr);

                {
                        std::lock_guard<std::mutex> lock(mutex_);
                        session_.active = false;
                        session_.mode = SessionMode::None;
                        if (result.success)
                        {
                                session_.last_error.clear();
                                last_capture_.type = "still";
                                last_capture_.frames = result.frames;
                        }
                        else
                                session_.last_error = result.error;
                }

                if (!result.success)
                {
                        response.status_code = 500;
                        response.body = "{\"error\":" + jsonString(result.error) + "}";
                        return response;
                }

                std::ostringstream body;
                body << "{\"frames\":[";
                for (size_t i = 0; i < result.frames.size(); ++i)
                {
                        if (i)
                                body << ',';
                        body << jsonString(result.frames[i]);
                }
                body << "],\"count\":" << result.frames.size() << "}";
                response.body = body.str();
                return response;
        });

        server_.addHandler("POST", "/recordings/video", [this](HttpRequest const &) {
                joinFinishedVideoThread();

                HttpResponse response;
                CameraSettings settings;
                {
                        std::lock_guard<std::mutex> lock(mutex_);
                        if (session_.active)
                        {
                                response.status_code = 409;
                                response.body = "{\"error\":\"A session is already active\"}";
                                return response;
                        }
                        session_.mode = SessionMode::Video;
                        session_.active = true;
                        session_.last_error.clear();
                        settings = settings_;
                        video_stop_flag_.store(false);
                }

                try
                {
                        video_thread_ = std::thread(&CameraDaemon::runVideoCapture, this, settings);
                }
                catch (std::exception const &ex)
                {
                        std::lock_guard<std::mutex> lock(mutex_);
                        session_.active = false;
                        session_.mode = SessionMode::None;
                        session_.last_error = ex.what();
                        response.status_code = 500;
                        response.body = "{\"error\":" + jsonString(ex.what()) + "}";
                        return response;
                }

                response.body = buildStatusJson();
                return response;
        });

        server_.addHandler("DELETE", "/recordings/video", [this](HttpRequest const &) {
                HttpResponse response;
                std::thread thread_to_join;
                bool active_video = false;
                {
                        std::lock_guard<std::mutex> lock(mutex_);
                        if (session_.active && session_.mode == SessionMode::Video)
                        {
                                video_stop_flag_.store(true);
                                thread_to_join = std::move(video_thread_);
                                active_video = true;
                        }
                        else if (video_thread_.joinable())
                                thread_to_join = std::move(video_thread_);
                }

                if (thread_to_join.joinable())
                        thread_to_join.join();

                if (!active_video)
                {
                        response.status_code = 409;
                        response.body = "{\"error\":\"No active video session\"}";
                        return response;
                }

                response.body = buildStatusJson();
                return response;
        });

        server_.addHandler("GET", "/preview", [this](HttpRequest const &) {
                HttpResponse response;
                response.content_type = "image/jpeg";
                response.headers.emplace_back("Cache-Control", "no-cache, no-store, must-revalidate");
                response.headers.emplace_back("Pragma", "no-cache");

                std::vector<uint8_t> jpeg;
                CameraSettings settings = getSettings();
                std::string error;
                if (!capturePreviewSnapshot(settings, jpeg, error))
                {
                        response.status_code = 503;
                        response.content_type = "application/json";
                        response.body = "{\"error\":" + jsonString(error) + "}";
                        return response;
                }

                response.body.assign(reinterpret_cast<const char *>(jpeg.data()), jpeg.size());
                return response;
        });
}

void CameraDaemon::joinFinishedVideoThread()
{
        std::thread thread;
        {
                std::lock_guard<std::mutex> lock(mutex_);
                if (session_.active)
                        return;
                if (!video_thread_.joinable())
                        return;
                thread = std::move(video_thread_);
        }
        if (thread.joinable())
                thread.join();
}

void CameraDaemon::runVideoCapture(CameraSettings settings)
{
        CaptureResult result = runCineDngCapture(settings, false, &video_stop_flag_);

        {
                std::lock_guard<std::mutex> lock(mutex_);
                video_stop_flag_.store(false);
                if (result.success)
                {
                        session_.last_error.clear();
                        last_capture_.type = "video";
                        last_capture_.frames = std::move(result.frames);
                }
                else
                        session_.last_error = result.error;
                session_.active = false;
                session_.mode = SessionMode::None;
        }
}

bool CameraDaemon::stopActiveVideo(std::string &error_message)
{
        error_message.clear();
        std::thread thread;
        {
                        std::lock_guard<std::mutex> lock(mutex_);
                        if (!session_.active || session_.mode != SessionMode::Video)
                        {
                                if (video_thread_.joinable())
                                        thread = std::move(video_thread_);
                                return false;
                        }
                        video_stop_flag_.store(true);
                        thread = std::move(video_thread_);
        }

        if (thread.joinable())
                thread.join();
        return true;
}

} // namespace rpicam
