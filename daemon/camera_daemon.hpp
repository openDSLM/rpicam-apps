/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * camera_daemon.hpp - High level controller for the rpicam daemon.
 */

#pragma once

#include <mutex>
#include <optional>
#include <string>

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
};

enum class SessionMode
{
        None,
        Preview,
        Still,
        Video,
        CinemaDng
};

struct SessionState
{
        SessionMode mode = SessionMode::None;
        bool active = false;
        std::string last_error;
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
        std::string buildStatusJson() const;
        static std::string buildSettingsJson(CameraSettings const &settings);
        static std::string modeToString(SessionMode mode);
        static std::optional<SessionMode> modeFromString(std::string const &value);

        void registerRoutes();

        mutable std::mutex mutex_;
        SimpleHttpServer server_;
        CameraSettings settings_;
        SessionState session_;
};

} // namespace rpicam

