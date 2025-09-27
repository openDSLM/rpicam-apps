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
#include "core/buffer_sync.hpp"
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

class PreviewSubscription
{
public:
        PreviewSubscription(std::atomic<int> &clients, std::atomic<bool> &enabled)
                : clients_(clients), enabled_(enabled)
        {
                clients_.fetch_add(1, std::memory_order_relaxed);
                enabled_.store(true, std::memory_order_relaxed);
        }

        ~PreviewSubscription()
        {
                if (clients_.fetch_sub(1, std::memory_order_relaxed) == 1)
                        enabled_.store(false, std::memory_order_relaxed);
        }

        PreviewSubscription(PreviewSubscription const &) = delete;
        PreviewSubscription &operator=(PreviewSubscription const &) = delete;

private:
        std::atomic<int> &clients_;
        std::atomic<bool> &enabled_;
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

CameraDaemon::CameraDaemon() {}

void CameraDaemon::start(uint16_t port)
{
        registerRoutes();
        server_.start(port);
        startCameraLoop();
}

void CameraDaemon::stop()
{
        server_.stop();
        stopCameraLoop();
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

        if (auto it = values.find("mode"); it != values.end())
        {
                auto mode = it->second.asString();
                if (!mode)
                {
                        error_message = "mode must be a string";
                        return false;
                }
                try
                {
                        Mode check(*mode);
                        (void)check;
                }
                catch (std::exception const &ex)
                {
                        error_message = std::string("Invalid mode: ") + ex.what();
                        return false;
                }
                updated.mode = *mode;
        }

        settings_ = updated;
        session_.last_error.clear();
        return true;
}

bool CameraDaemon::startSession(SessionMode mode, std::string &error_message)
{
        if (mode == SessionMode::Still)
        {
                // Fire a still capture using the background loop and wait for completion
                {
                        std::lock_guard<std::mutex> lock(capture_mutex_);
                        still_result_.clear();
                        still_pending_ = true;
                }
                std::vector<std::string> frames;
                {
                        std::unique_lock<std::mutex> lock(capture_mutex_);
                        if (!still_cv_.wait_for(lock, std::chrono::seconds(5), [&]{ return !still_result_.empty(); }))
                        {
                                error_message = "Timed out waiting for still frame";
                                return false;
                        }
                        frames = still_result_;
                        still_result_.clear();
                }
                {
                        std::lock_guard<std::mutex> lock(mutex_);
                        last_capture_.type = "still";
                        last_capture_.frames = std::move(frames);
                        session_.last_error.clear();
                }
                return true;
        }

        if (mode == SessionMode::Video)
        {
                {
                        std::lock_guard<std::mutex> lock(capture_mutex_);
                        if (video_recording_)
                        {
                                error_message = "Video already recording";
                                return false;
                        }
                        video_recording_ = true;
                }
                {
                        std::lock_guard<std::mutex> lock(mutex_);
                        session_.mode = SessionMode::Video;
                        session_.active = true;
                        session_.last_error.clear();
                }
                return true;
        }

        if (mode == SessionMode::None)
        {
                {
                        std::lock_guard<std::mutex> lock(capture_mutex_);
                        video_recording_ = false;
                }
                {
                        std::lock_guard<std::mutex> lock(mutex_);
                        session_.active = false;
                        session_.mode = SessionMode::None;
                }
                error_message.clear();
                return true;
        }

        error_message = "Unsupported mode";
        return false;
}

bool CameraDaemon::stopSession(std::string &error_message)
{
        {
                std::lock_guard<std::mutex> lock(capture_mutex_);
                if (!video_recording_)
                {
                        error_message = "No active session";
                        return false;
                }
                video_recording_ = false;
        }
        {
                std::lock_guard<std::mutex> lock(mutex_);
                session_.active = false;
                session_.mode = SessionMode::None;
                session_.last_error.clear();
        }
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
             << ",\"mode\":" << jsonString(settings.mode)
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
                // Avoid concurrent CameraManager instances by serialising camera access.
                std::unique_lock<std::mutex> cam_lock(camera_guard_, std::try_to_lock);
                if (!cam_lock.owns_lock())
                {
                        error = "Camera busy (another preview/recording in progress)";
                        return false;
                }
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

        if (!settings.mode.empty())
        {
                options.mode_string = settings.mode;
                options.mode = Mode(settings.mode);
        }
        else
        {
                options.mode_string.clear();
                options.mode = Mode();
        }

        // Do not force a raw mode size; mirror rpicam-cinedng behavior and
        // let libcamera adjust to the closest supported sensor mode.

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
                camera_reconfigure_.store(true);
                response.body = buildSettingsJson(getSettings());
                return response;
        });

        server_.addHandler("POST", "/capture/still", [this](HttpRequest const &) {
                HttpResponse response;
                // Arm a one-shot still capture via the background camera loop.
                {
                        std::lock_guard<std::mutex> lock(capture_mutex_);
                        still_result_.clear();
                        still_pending_ = true;
                }

                // Wait for one frame to be written (with a timeout).
                std::vector<std::string> frames;
                {
                        std::unique_lock<std::mutex> lock(capture_mutex_);
                        bool ok = still_cv_.wait_for(lock, std::chrono::seconds(5), [&] {
                                return !still_result_.empty();
                        });
                        if (!ok)
                        {
                                response.status_code = 503;
                                response.body = "{\"error\":\"Timed out waiting for still frame\"}";
                                return response;
                        }
                        frames = still_result_;
                        still_result_.clear();
                }

                // Update last_capture summary
                {
                        std::lock_guard<std::mutex> lock(mutex_);
                        session_.last_error.clear();
                        last_capture_.type = "still";
                        last_capture_.frames = frames;
                }

                std::ostringstream body;
                body << "{\"frames\":[";
                for (size_t i = 0; i < frames.size(); ++i)
                {
                        if (i) body << ',';
                        body << jsonString(frames[i]);
                }
                body << "],\"count\":" << frames.size() << "}";
                response.body = body.str();
                return response;
        });

server_.addHandler("POST", "/recordings/video", [this](HttpRequest const &) {
                HttpResponse response;
                {
                        std::lock_guard<std::mutex> lock(capture_mutex_);
                        if (video_recording_)
                        {
                                response.status_code = 409;
                                response.body = "{\"error\":\"Video already recording\"}";
                                return response;
                        }
                        video_recording_ = true;
                }
                {
                        std::lock_guard<std::mutex> lock(mutex_);
                        session_.mode = SessionMode::Video;
                        session_.active = true;
                        session_.last_error.clear();
                }
                response.body = buildStatusJson();
                return response;
        });

server_.addHandler("DELETE", "/recordings/video", [this](HttpRequest const &) {
                HttpResponse response;
                bool was_active = false;
                {
                        std::lock_guard<std::mutex> lock(capture_mutex_);
                        was_active = video_recording_;
                        video_recording_ = false;
                }
                if (!was_active)
                {
                        response.status_code = 409;
                        response.body = "{\"error\":\"No active video session\"}";
                        return response;
                }
                {
                        std::lock_guard<std::mutex> lock(mutex_);
                        session_.active = false;
                        session_.mode = SessionMode::None;
                }
                response.body = buildStatusJson();
                return response;
        });

        server_.addHandler("GET", "/preview", [this](HttpRequest const &) {
                PreviewSubscription preview_client(preview_clients_, preview_enabled_);
                HttpResponse response;
                response.content_type = "image/jpeg";
                response.headers.emplace_back("Cache-Control", "no-cache, no-store, must-revalidate");
                response.headers.emplace_back("Pragma", "no-cache");

                std::vector<uint8_t> jpeg;
                uint64_t seq_before;
                {
                        std::unique_lock<std::mutex> lock(preview_mutex_);
                        seq_before = preview_seq_;
                        if (latest_jpeg_.empty())
                        {
                                preview_cv_.wait_for(lock, std::chrono::milliseconds(1000), [&]{ return preview_seq_ != seq_before && !latest_jpeg_.empty(); });
                        }
                        if (latest_jpeg_.empty())
                        {
                                response.status_code = 503;
                                response.content_type = "application/json";
                                response.body = "{\"error\":\"No preview available\"}";
                                return response;
                        }
                        jpeg = latest_jpeg_;
                }
                response.body.assign(reinterpret_cast<const char *>(jpeg.data()), jpeg.size());
                return response;
        });

        server_.addStreamHandler("GET", "/preview/stream", [this](int client_fd, HttpRequest const &)
        {
                PreviewSubscription preview_client(preview_clients_, preview_enabled_);
                const char *hdr =
                        "HTTP/1.1 200 OK\r\n"
                        "Connection: close\r\n"
                        "Cache-Control: no-cache, no-store, must-revalidate\r\n"
                        "Pragma: no-cache\r\n"
                        "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";
                ::send(client_fd, hdr, strlen(hdr), 0);
                streamClientLoop(client_fd);
        });
}

void CameraDaemon::startCameraLoop()
{
        if (camera_thread_.joinable())
                return;
        camera_stop_.store(false);
        camera_thread_ = std::thread(&CameraDaemon::cameraLoop, this);
}

void CameraDaemon::stopCameraLoop()
{
        camera_stop_.store(true);
        if (camera_thread_.joinable())
                camera_thread_.join();
}

void CameraDaemon::cameraLoop()
{
        while (!camera_stop_.load())
        {
                try
                {
                        PreviewEncoder app;
                        VideoOptions *options = app.GetOptions();
                        // Default-initialise Options
                        char arg0[] = "rpicam-daemon";
                        char *argv[] = { arg0 };
                        int argc = 1;
                        options->Parse(argc, argv);

                        CameraSettings settings = getSettings();
                        applySettingsToOptions(settings, *options, true /* request raw */);

                        // Set callbacks that forward to an active DNG writer when present
                        app.SetEncodeOutputReadyCallback([this](void *mem, size_t size, int64_t ts, bool key) {
                                std::shared_ptr<DngOutput> out;
                                {
                                        std::lock_guard<std::mutex> lock(capture_mutex_);
                                        out = active_output_;
                                }
                                if (out)
                                        out->OutputReady(mem, size, ts, key);
                        });
                        app.SetMetadataReadyCallback([this](libcamera::ControlList &ctrls) {
                                std::shared_ptr<DngOutput> out;
                                {
                                        std::lock_guard<std::mutex> lock(capture_mutex_);
                                        out = active_output_;
                                }
                                if (out)
                                        out->MetadataReady(ctrls);
                        });

                        app.OpenCamera();
                        // Match rpicam-cinedng configuration: video + raw (raw is used for DNG).
                        app.ConfigureVideo(RPiCamApp::FLAG_VIDEO_RAW);

                        StreamInfo vinfo;
                        libcamera::Stream *vstream = app.VideoStream(&vinfo);
                        if (!vstream)
                                throw std::runtime_error("Video stream unavailable for preview");

                        StreamInfo rinfo;
                        libcamera::Stream *rstream = app.RawStream(&rinfo);
                        if (!rstream)
                                throw std::runtime_error("Raw stream unavailable");

                        // Prepare DNG writer creation lambda
                        auto ensure_output = [this, options, &rinfo, &app]() {
                                if (active_output_)
                                        return;
                                auto out = std::make_shared<DngOutput>(options, rinfo, app.CameraModel());
                                out->SetFrameWrittenCallback([this](std::string const &path) {
                                        std::lock_guard<std::mutex> lock(capture_mutex_);
                                        if (still_pending_)
                                        {
                                                still_result_.push_back(path);
                                                still_pending_ = false;
                                                still_cv_.notify_all();
                                        }
                                        if (video_recording_)
                                        {
                                                std::lock_guard<std::mutex> lock2(mutex_);
                                                last_capture_.type = "video";
                                                last_capture_.frames.push_back(path);
                                        }
                                });
                                active_output_ = std::move(out);
                        };

                        app.StartEncoder();
                        app.StartCamera();

                        while (!camera_stop_.load() && !camera_reconfigure_.load())
                        {
                                RPiCamEncoder::Msg msg = app.Wait();
                                if (msg.type == RPiCamApp::MsgType::Timeout)
                                {
                                        LOG_ERROR("Device timeout detected, attempting a restart");
                                        app.StopCamera();
                                        app.StartCamera();
                                        continue;
                                }
                                if (msg.type == RPiCamEncoder::MsgType::Quit)
                                        break;
                                if (msg.type != RPiCamEncoder::MsgType::RequestComplete)
                                        throw std::runtime_error("Unrecognised message from camera");

                                CompletedRequestPtr &completed_request = std::get<CompletedRequestPtr>(msg.payload);

                                // Update preview JPEG from the video stream (disabled unless preview_enabled_)
                                if (preview_enabled_.load(std::memory_order_relaxed))
                                {
                                        libcamera::FrameBuffer *buffer = completed_request->buffers[vstream];
                                        BufferReadSync v(&app, buffer);
                                        auto const &planes = v.Get();
                                        if (!planes.empty())
                                        {
                                                libcamera::Span<uint8_t> span = planes[0];
                                                std::vector<uint8_t> jpeg = encodeFrameToJpeg(span, vinfo, 85);
                                                if (!jpeg.empty())
                                                {
                                                        std::lock_guard<std::mutex> lock(preview_mutex_);
                                                        latest_jpeg_ = std::move(jpeg);
                                                        preview_seq_++;
                                                }
                                        }
                                        preview_cv_.notify_all();
                                }

                                // Handle capturing frames to DNG when requested
                                bool need_output = false;
                                bool keep_output = false;
                                {
                                        std::lock_guard<std::mutex> lock(capture_mutex_);
                                        need_output = video_recording_ || still_pending_;
                                        keep_output = video_recording_;
                                        if (need_output && !active_output_)
                                                ensure_output();
                                }
                                if (need_output)
                                        app.EncodeBuffer(completed_request, rstream);
                                else
                                {
                                        // Drop any active output if not recording
                                        std::lock_guard<std::mutex> lock(capture_mutex_);
                                        if (active_output_ && !keep_output)
                                                active_output_.reset();
                                }
                        }

                        app.StopCamera();
                        app.StopEncoder();

                        if (camera_reconfigure_.load())
                                camera_reconfigure_.store(false);
                }
                catch (std::exception const &ex)
                {
                        LOG_ERROR("Camera loop error: " << ex.what());
                        std::this_thread::sleep_for(std::chrono::milliseconds(200));
                }
        }
}

void CameraDaemon::streamClientLoop(int client_fd)
{
        uint64_t last_seq = 0;
        while (true)
        {
            std::vector<uint8_t> jpeg;
            {
                std::unique_lock<std::mutex> lock(preview_mutex_);
                preview_cv_.wait_for(lock, std::chrono::seconds(2), [&]{ return preview_seq_ != last_seq || camera_stop_.load(); });
                if (camera_stop_.load())
                        break;
                if (preview_seq_ == last_seq || latest_jpeg_.empty())
                        continue;
                last_seq = preview_seq_;
                jpeg = latest_jpeg_;
            }

            char header[256];
            int n = snprintf(header, sizeof(header),
                             "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %zu\r\n\r\n",
                             jpeg.size());
            if (n <= 0)
                    break;
            if (::send(client_fd, header, n, 0) < 0)
                    break;
            if (::send(client_fd, reinterpret_cast<const char *>(jpeg.data()), jpeg.size(), 0) < 0)
                    break;
            static const char *crlf = "\r\n";
            if (::send(client_fd, crlf, 2, 0) < 0)
                    break;
        }
}

} // namespace rpicam
