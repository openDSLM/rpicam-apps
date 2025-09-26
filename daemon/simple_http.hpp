/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * simple_http.hpp - Minimal HTTP server utilities for the rpicam daemon.
 */

#pragma once

#include <atomic>
#include <cerrno>
#include <csignal>
#include <cstring>
#include <functional>
#include <map>
#include <mutex>
#include <netinet/in.h>
#include <optional>
#include <stdexcept>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <utility>
#include <vector>
#include <sstream>

namespace rpicam
{

struct HttpRequest
{
        std::string method;
        std::string target;
        std::map<std::string, std::string> headers;
        std::string body;
};

struct HttpResponse
{
        int status_code = 200;
        std::string content_type = "application/json";
        std::string body;
        std::vector<std::pair<std::string, std::string>> headers;
};

class SimpleHttpServer
{
public:
        using Handler = std::function<HttpResponse(const HttpRequest &)>;

        SimpleHttpServer();
        ~SimpleHttpServer();

        void addHandler(const std::string &method, const std::string &path, Handler handler);
        void start(uint16_t port);
        void stop();

        bool running() const { return running_; }

private:
        void listenLoop(uint16_t port);
        void handleClient(int client_fd);
        bool parseRequest(int client_fd, HttpRequest &request);
        static std::string trim(const std::string &value);

        using HandlerKey = std::pair<std::string, std::string>;

        struct KeyCompare
        {
                bool operator()(HandlerKey const &a, HandlerKey const &b) const
                {
                        if (a.first < b.first)
                                return true;
                        if (a.first > b.first)
                                return false;
                        return a.second < b.second;
                }
        };

        std::map<HandlerKey, Handler, KeyCompare> handlers_;
        std::atomic<bool> running_;
        std::thread listen_thread_;
        int server_fd_;
        std::mutex handler_mutex_;
};

inline SimpleHttpServer::SimpleHttpServer() : running_(false), server_fd_(-1) {}

inline SimpleHttpServer::~SimpleHttpServer()
{
        stop();
}

inline void SimpleHttpServer::addHandler(const std::string &method, const std::string &path, Handler handler)
{
        std::lock_guard<std::mutex> lock(handler_mutex_);
        handlers_[{method, path}] = std::move(handler);
}

inline void SimpleHttpServer::start(uint16_t port)
{
        if (running_)
                throw std::runtime_error("HTTP server already running");

        running_ = true;
        listen_thread_ = std::thread(&SimpleHttpServer::listenLoop, this, port);
}

inline void SimpleHttpServer::stop()
{
        if (!running_)
                return;

        running_ = false;
        if (server_fd_ >= 0)
        {
                ::shutdown(server_fd_, SHUT_RDWR);
                ::close(server_fd_);
                server_fd_ = -1;
        }

        if (listen_thread_.joinable())
                listen_thread_.join();
}

inline void SimpleHttpServer::listenLoop(uint16_t port)
{
        server_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
        if (server_fd_ < 0)
                throw std::runtime_error("Failed to create socket");

        int opt = 1;
        ::setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_ANY);
        address.sin_port = htons(port);

        if (::bind(server_fd_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0)
        {
                int err = errno;
                ::close(server_fd_);
                server_fd_ = -1;
                running_ = false;
                throw std::runtime_error("Failed to bind HTTP server socket: " + std::string(std::strerror(err)));
        }

        if (::listen(server_fd_, 5) < 0)
        {
                int err = errno;
                ::close(server_fd_);
                server_fd_ = -1;
                running_ = false;
                throw std::runtime_error("Failed to listen on HTTP server socket: " + std::string(std::strerror(err)));
        }

        while (running_)
        {
                sockaddr_in client{};
                socklen_t len = sizeof(client);
                int client_fd = ::accept(server_fd_, reinterpret_cast<sockaddr *>(&client), &len);
                if (client_fd < 0)
                {
                        if (running_)
                                continue;
                        break;
                }

                std::thread(&SimpleHttpServer::handleClient, this, client_fd).detach();
        }

        if (server_fd_ >= 0)
        {
                ::close(server_fd_);
                server_fd_ = -1;
        }
}

inline void SimpleHttpServer::handleClient(int client_fd)
{
        HttpRequest request;
        if (!parseRequest(client_fd, request))
        {
                ::close(client_fd);
                return;
        }

        Handler handler;
        {
                std::lock_guard<std::mutex> lock(handler_mutex_);
                auto it = handlers_.find({request.method, request.target});
                if (it != handlers_.end())
                        handler = it->second;
        }

        HttpResponse response;
        if (handler)
        {
                response = handler(request);
        }
        else
        {
                response.status_code = 404;
                response.content_type = "application/json";
                response.body = "{\"error\":\"Not found\"}";
        }

        std::string status_line = "HTTP/1.1 " + std::to_string(response.status_code) + "\r\n";
        std::string headers = "Content-Type: " + response.content_type + "\r\n";
        headers += "Content-Length: " + std::to_string(response.body.size()) + "\r\n";
        for (auto const &h : response.headers)
                headers += h.first + ": " + h.second + "\r\n";
        std::string payload = status_line + headers + "Connection: close\r\n\r\n" + response.body;
        ::send(client_fd, payload.data(), payload.size(), 0);
        ::close(client_fd);
}

inline bool SimpleHttpServer::parseRequest(int client_fd, HttpRequest &request)
{
        std::string data;
        char buffer[4096];
        ssize_t bytes = 0;
        bool header_complete = false;
        size_t expected_length = 0;

        while (true)
        {
            bytes = ::recv(client_fd, buffer, sizeof(buffer), 0);
            if (bytes <= 0)
                    break;
            data.append(buffer, buffer + bytes);
            if (!header_complete)
            {
                    size_t header_end = data.find("\r\n\r\n");
                    if (header_end != std::string::npos)
                    {
                            header_complete = true;
                            std::string headers_str = data.substr(0, header_end + 2);
                            std::istringstream header_stream(headers_str);
                            std::string request_line;
                            if (!std::getline(header_stream, request_line))
                                    return false;
                            if (!request_line.empty() && request_line.back() == '\r')
                                    request_line.pop_back();
                            std::istringstream request_line_stream(request_line);
                            request_line_stream >> request.method >> request.target;
                            std::string version;
                            request_line_stream >> version;
                            std::string header_line;
                            while (std::getline(header_stream, header_line))
                            {
                                    if (!header_line.empty() && header_line.back() == '\r')
                                            header_line.pop_back();
                                    if (header_line.empty())
                                            continue;
                                    size_t colon = header_line.find(':');
                                    if (colon == std::string::npos)
                                            continue;
                                    std::string name = trim(header_line.substr(0, colon));
                                    std::string value = trim(header_line.substr(colon + 1));
                                    request.headers[name] = value;
                                    if (name == "Content-Length")
                                            expected_length = std::stoul(value);
                            }
                            size_t body_start = header_end + 4;
                            request.body = data.substr(body_start);
                            if (expected_length <= request.body.size())
                                    return true;
                    }
            }
            else if (expected_length <= request.body.size())
                    return true;
        }

        return header_complete && expected_length <= request.body.size();
}

inline std::string SimpleHttpServer::trim(const std::string &value)
{
        const char *whitespace = " \t\r\n";
        size_t start = value.find_first_not_of(whitespace);
        size_t end = value.find_last_not_of(whitespace);
        if (start == std::string::npos)
                return "";
        return value.substr(start, end - start + 1);
}

} // namespace rpicam

