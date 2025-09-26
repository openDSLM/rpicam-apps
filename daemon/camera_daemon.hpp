/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * camera_daemon.hpp - High level controller for the rpicam daemon.
 */

#pragma once

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "core/video_options.hpp"
#include "daemon/json_utils.hpp"
#include "daemon/simple_http.hpp"

class DngOutput; // forward declaration

namespace rpicam
{

struct CameraSettings
{
        double fps = 24.0;
        double shutter_us = 0.0;
        double analogue_gain = 1.0;
        bool auto_exposure = true;
        std::string output_dir = "/ssd/RAW";
};

enum class SessionMode
{
        None,
        Still,
        Video
};

struct SessionState
{
        SessionMode mode = SessionMode::None;
        bool active = false;
        std::string last_error;
};

struct CaptureSummary
{
        std::string type;
        std::vector<std::string> frames;
};

class CameraDaemon
{
public:
        CameraDaemon();

        void start(uint16_t port);
        void stop();

        SessionState getState() const;
        CameraSettings getSettings() const;

        bool updateSettings(JsonObject const &values, std::string &error_message);
        bool startSession(SessionMode mode, std::string &error_message);
        bool stopSession(std::string &error_message);

private:
        struct CaptureResult
        {
                bool success = false;
                std::vector<std::string> frames;
                std::string error;
        };

        std::string buildStatusJson() const;
        static std::string buildSettingsJson(CameraSettings const &settings);
        static std::string modeToString(SessionMode mode);
        static std::string buildCaptureJson(CaptureSummary const &capture);

        CaptureResult runCineDngCapture(CameraSettings const &settings, bool single_shot,
                                        std::atomic<bool> *stop_flag);
        bool capturePreviewSnapshot(CameraSettings const &settings, std::vector<uint8_t> &jpeg,
                                    std::string &error);
        static bool ensureOutputDirectory(std::string const &path, std::string &error_message);
        static void applySettingsToOptions(CameraSettings const &settings, VideoOptions &options,
                                           bool request_raw);

        void registerRoutes();
        // Unified background camera loop
        void cameraLoop();
        void startCameraLoop();
        void stopCameraLoop();
        // Per-client MJPEG streaming writer
        void streamClientLoop(int client_fd);

        mutable std::mutex mutex_;
        SimpleHttpServer server_;
        CameraSettings settings_;
        SessionState session_;
        CaptureSummary last_capture_;

        // Background camera thread and control flags
        std::thread camera_thread_;
        std::atomic<bool> camera_stop_{false};
        std::atomic<bool> camera_reconfigure_{false};
        std::mutex camera_guard_;

        // Latest preview frame (JPEG)
        std::mutex preview_mutex_;
        std::condition_variable preview_cv_;
        std::vector<uint8_t> latest_jpeg_;
        uint64_t preview_seq_ = 0;
        bool preview_enabled_ = false; // generate preview from RAW when true

        // Capture state shared with camera loop
        std::mutex capture_mutex_;
        std::shared_ptr<DngOutput> active_output_;
        bool video_recording_ = false;
        bool still_pending_ = false;
        std::vector<std::string> still_result_;
        std::condition_variable still_cv_;
};

} // namespace rpicam
