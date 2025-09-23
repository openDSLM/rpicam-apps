/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2024
 *
 * jpeg_stream_preview.cpp - Serve preview frames as an MJPEG stream.
 */

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cstring>
#include <jpeglib.h>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "core/logging.hpp"
#include "core/options.hpp"
#include "core/stream_info.hpp"
#include "core/video_options.hpp"

#include "preview.hpp"

#if JPEG_LIB_VERSION_MAJOR > 9 || (JPEG_LIB_VERSION_MAJOR == 9 && JPEG_LIB_VERSION_MINOR >= 4)
typedef size_t jpeg_mem_len_t;
#else
typedef unsigned long jpeg_mem_len_t;
#endif

#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0
#endif

namespace
{

std::pair<std::string, uint16_t> parse_stream_spec(const std::string &spec)
{
        if (spec.empty())
                throw std::runtime_error("preview stream specification is empty");

        auto pos = spec.rfind(':');
        std::string host;
        std::string port_str;
        if (pos == std::string::npos)
        {
                host = "0.0.0.0";
                port_str = spec;
        }
        else
        {
                host = spec.substr(0, pos);
                if (host.empty())
                        host = "0.0.0.0";
                port_str = spec.substr(pos + 1);
        }

        if (port_str.empty())
                throw std::runtime_error("preview stream port must be specified");

        int port = std::stoi(port_str);
        if (port <= 0 || port > 65535)
                throw std::runtime_error("preview stream port out of range");

        if (host == "localhost")
                host = "127.0.0.1";

        return { host, static_cast<uint16_t>(port) };
}

class JpegStreamPreview : public Preview
{
public:
        explicit JpegStreamPreview(Options const *options);
        ~JpegStreamPreview() override;

        void Show(int fd, libcamera::Span<uint8_t> span, StreamInfo const &info) override;
        void Reset() override;
        bool Quit() override { return false; }
        void MaxImageSize(unsigned int &w, unsigned int &h) const override { w = h = 0; }

private:
        void acceptThread();
        void closeClients();
        void sendFrame(const std::vector<uint8_t> &jpeg_data);
        std::vector<uint8_t> encode(libcamera::Span<uint8_t> span, StreamInfo const &info);

        int listen_fd_ = -1;
        std::vector<int> clients_;
        std::mutex clients_mutex_;
        std::thread accept_thread_;
        std::atomic<bool> abort_{ false };
        int quality_ = 75;
};

JpegStreamPreview::JpegStreamPreview(Options const *options) : Preview(options)
{
        std::string spec = options->preview_stream;
        auto [host, port] = parse_stream_spec(spec);

        int fd = socket(AF_INET, SOCK_STREAM, 0);
        if (fd < 0)
                throw std::runtime_error("failed to open preview stream socket");

        int enable = 1;
        if (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable)) < 0)
        {
                close(fd);
                throw std::runtime_error("failed to set socket options for preview stream");
        }

        sockaddr_in addr = {};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port);
        if (inet_aton(host.c_str(), &addr.sin_addr) == 0)
        {
                close(fd);
                throw std::runtime_error("invalid preview stream address " + host);
        }

        if (bind(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0)
        {
                close(fd);
                throw std::runtime_error("failed to bind preview stream socket");
        }

        if (listen(fd, 4) < 0)
        {
                close(fd);
                throw std::runtime_error("failed to listen on preview stream socket");
        }

        listen_fd_ = fd;

        if (auto video_opts = dynamic_cast<VideoOptions const *>(options))
                quality_ = video_opts->quality;

        LOG(1, "Serving MJPEG preview on " << host << ":" << port);
        accept_thread_ = std::thread(&JpegStreamPreview::acceptThread, this);
}

JpegStreamPreview::~JpegStreamPreview()
{
        abort_ = true;
        if (listen_fd_ >= 0)
        {
                shutdown(listen_fd_, SHUT_RDWR);
                close(listen_fd_);
                listen_fd_ = -1;
        }
        if (accept_thread_.joinable())
                accept_thread_.join();
        closeClients();
}

void JpegStreamPreview::closeClients()
{
        std::lock_guard<std::mutex> lock(clients_mutex_);
        for (int fd : clients_)
        {
                shutdown(fd, SHUT_RDWR);
                close(fd);
        }
        clients_.clear();
}

void JpegStreamPreview::acceptThread()
{
        while (!abort_)
        {
                sockaddr_in client_addr = {};
                socklen_t len = sizeof(client_addr);
                int client = accept(listen_fd_, reinterpret_cast<sockaddr *>(&client_addr), &len);
                if (client < 0)
                {
                        if (abort_ || errno == EBADF)
                                return;
                        if (errno == EINTR)
                                continue;
                        LOG_ERROR("Failed to accept preview client: " << strerror(errno));
                        continue;
                }

                const char header[] =
                        "HTTP/1.0 200 OK\r\n"
                        "Cache-Control: no-cache\r\n"
                        "Pragma: no-cache\r\n"
                        "Connection: close\r\n"
                        "Content-Type: multipart/x-mixed-replace; boundary=--frame\r\n\r\n";
                if (send(client, header, sizeof(header) - 1, MSG_NOSIGNAL) < 0)
                {
                        LOG_ERROR("Failed to send preview header to client");
                        shutdown(client, SHUT_RDWR);
                        close(client);
                        continue;
                }

                {
                        std::lock_guard<std::mutex> lock(clients_mutex_);
                        clients_.push_back(client);
                }
                LOG(2, "Preview client connected");
        }
}

std::vector<uint8_t> JpegStreamPreview::encode(libcamera::Span<uint8_t> span, StreamInfo const &info)
{
        if (!span.size())
                return {};

        struct jpeg_compress_struct cinfo;
        struct jpeg_error_mgr jerr;
        cinfo.err = jpeg_std_error(&jerr);
        jpeg_create_compress(&cinfo);

        unsigned char *encoded_buffer = nullptr;
        jpeg_mem_len_t encoded_size = 0;
        jpeg_mem_dest(&cinfo, &encoded_buffer, &encoded_size);

        cinfo.image_width = info.width;
        cinfo.image_height = info.height;
        cinfo.input_components = 3;
        cinfo.in_color_space = JCS_YCbCr;
        cinfo.restart_interval = 0;
        jpeg_set_defaults(&cinfo);
        cinfo.raw_data_in = TRUE;
        jpeg_set_quality(&cinfo, quality_, TRUE);

        // Match the sampling to the incoming 4:2:0 buffers.
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
                for (int i = 0; i < 16; i++, Y_row += info.stride)
                        y_rows[i] = std::min(Y_row, Y_max);
                for (int i = 0; i < 8; i++, U_row += stride2, V_row += stride2)
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

void JpegStreamPreview::sendFrame(const std::vector<uint8_t> &jpeg_data)
{
        if (jpeg_data.empty())
                return;

        std::lock_guard<std::mutex> lock(clients_mutex_);
        if (clients_.empty())
                return;

        std::string header = "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " +
                             std::to_string(jpeg_data.size()) + "\r\n\r\n";
        std::vector<int> to_remove;

        for (size_t i = 0; i < clients_.size(); ++i)
        {
                int fd = clients_[i];
                bool failed = false;
                if (send(fd, header.c_str(), header.size(), MSG_NOSIGNAL) < 0)
                        failed = true;
                else if (send(fd, reinterpret_cast<const char *>(jpeg_data.data()), jpeg_data.size(), MSG_NOSIGNAL) < 0)
                        failed = true;
                else if (send(fd, "\r\n", 2, MSG_NOSIGNAL) < 0)
                        failed = true;

                if (failed)
                {
                        LOG_ERROR("Removing preview client after send error");
                        shutdown(fd, SHUT_RDWR);
                        close(fd);
                        to_remove.push_back(i);
                }
        }

        // Remove failed clients in reverse order so indices remain valid.
        for (auto it = to_remove.rbegin(); it != to_remove.rend(); ++it)
                clients_.erase(clients_.begin() + *it);
}

void JpegStreamPreview::Show(int fd, libcamera::Span<uint8_t> span, StreamInfo const &info)
{
        try
        {
                auto jpeg_data = encode(span, info);
                sendFrame(jpeg_data);
        }
        catch (std::exception const &e)
        {
                LOG_ERROR("Failed to encode preview frame: " << e.what());
        }

        if (done_callback_)
                done_callback_(fd);
}

void JpegStreamPreview::Reset()
{
        closeClients();
}

} // namespace

Preview *make_jpeg_stream_preview(Options const *options)
{
        return new JpegStreamPreview(options);
}

