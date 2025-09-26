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

        static CaptureResult runCineDngCapture(CameraSettings const &settings, bool single_shot,
                                                std::atomic<bool> *stop_flag);
        static bool capturePreviewSnapshot(CameraSettings const &settings, std::vector<uint8_t> &jpeg,
                                           std::string &error);
        static bool ensureOutputDirectory(std::string const &path, std::string &error_message);
        static void applySettingsToOptions(CameraSettings const &settings, VideoOptions &options,
                                           bool request_raw);

        void registerRoutes();
        void joinFinishedVideoThread();
        void runVideoCapture(CameraSettings settings);
        bool stopActiveVideo(std::string &error_message);

        mutable std::mutex mutex_;
        SimpleHttpServer server_;
        CameraSettings settings_;
        SessionState session_;
        CaptureSummary last_capture_;
        std::atomic<bool> video_stop_flag_;
        std::thread video_thread_;
};

} // namespace rpicam

