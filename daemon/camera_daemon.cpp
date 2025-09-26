/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * camera_daemon.cpp - High level controller for the rpicam daemon.
 */

#include "daemon/camera_daemon.hpp"

#include <algorithm>
#include <cctype>
#include <sstream>

namespace rpicam
{

CameraDaemon::CameraDaemon() = default;

void CameraDaemon::start(uint16_t port)
{
        registerRoutes();
        server_.start(port);
}

void CameraDaemon::stop()
{
        server_.stop();
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

        settings_ = updated;
        session_.last_error.clear();
        return true;
}

bool CameraDaemon::startSession(SessionMode mode, std::string &error_message)
{
        std::lock_guard<std::mutex> lock(mutex_);
        if (session_.active)
        {
                error_message = "A session is already active";
                return false;
        }

        session_.mode = mode;
        session_.active = true;
        session_.last_error.clear();
        // Future work: kick off the appropriate libcamera pipeline.
        return true;
}

bool CameraDaemon::stopSession(std::string &error_message)
{
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
        std::string settings_json = buildSettingsJson(settings_);
        json << "{\"state\":{"
             << "\"active\":" << (session_.active ? "true" : "false")
             << ",\"mode\":" << jsonString(modeToString(session_.mode))
             << ",\"last_error\":" << jsonString(session_.last_error)
             << "},\"settings\":" << settings_json
             << "}";
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
             << "}";
        return json.str();
}

std::string CameraDaemon::modeToString(SessionMode mode)
{
        switch (mode)
        {
        case SessionMode::None: return "none";
        case SessionMode::Preview: return "preview";
        case SessionMode::Still: return "still";
        case SessionMode::Video: return "video";
        case SessionMode::CinemaDng: return "cinemadng";
        }
        return "none";
}

std::optional<SessionMode> CameraDaemon::modeFromString(std::string const &value)
{
        std::string lowered = value;
        std::transform(lowered.begin(), lowered.end(), lowered.begin(), [](unsigned char c) { return std::tolower(c); });
        if (lowered == "none")
                return SessionMode::None;
        if (lowered == "preview")
                return SessionMode::Preview;
        if (lowered == "still")
                return SessionMode::Still;
        if (lowered == "video")
                return SessionMode::Video;
        if (lowered == "cinemadng" || lowered == "cinedng")
                return SessionMode::CinemaDng;
        return std::nullopt;
}

void CameraDaemon::registerRoutes()
{
        server_.addHandler("GET", "/status", [this](HttpRequest const &) {
                HttpResponse response;
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

        server_.addHandler("POST", "/session", [this](HttpRequest const &request) {
                HttpResponse response;
                JsonObject values = parseJsonObject(request.body);
                auto it = values.find("mode");
                if (it == values.end())
                {
                        response.status_code = 400;
                        response.body = "{\"error\":\"Missing mode\"}";
                        return response;
                }

                auto mode_str = it->second.asString();
                if (!mode_str)
                {
                        response.status_code = 400;
                        response.body = "{\"error\":\"Mode must be a string\"}";
                        return response;
                }

                auto mode = modeFromString(*mode_str);
                if (!mode)
                {
                        response.status_code = 400;
                        response.body = "{\"error\":\"Unsupported mode\"}";
                        return response;
                }

                std::string error;
                if (!startSession(*mode, error))
                {
                        response.status_code = 409;
                        response.body = "{\"error\":" + jsonString(error) + "}";
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

        server_.addHandler("GET", "/preview", [](HttpRequest const &) {
                HttpResponse response;
                response.status_code = 503;
                response.body = "{\"error\":\"Preview streaming is not yet implemented\"}";
                return response;
        });
}

} // namespace rpicam

