/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * rpicam_daemon.cpp - HTTP daemon backing future DSLR-style UI workflows.
 */

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

#include "daemon/camera_daemon.hpp"

namespace
{

std::atomic<bool> keep_running{true};

void signalHandler(int)
{
        keep_running = false;
}

uint16_t parsePort(int argc, char *argv[])
{
        uint16_t port = 8400;
        for (int i = 1; i < argc; ++i)
        {
                std::string arg = argv[i];
                if ((arg == "--port" || arg == "-p") && i + 1 < argc)
                {
                        int value = std::atoi(argv[++i]);
                        if (value <= 0 || value > 65535)
                                throw std::runtime_error("Port must be between 1 and 65535");
                        port = static_cast<uint16_t>(value);
                }
                else if (arg == "--help" || arg == "-h")
                {
                        std::cout << "Usage: rpicam-daemon [--port <port>]" << std::endl;
                        std::exit(0);
                }
                else
                        throw std::runtime_error("Unknown argument: " + arg);
        }
        return port;
}

} // namespace

int main(int argc, char *argv[])
{
        try
        {
                uint16_t port = parsePort(argc, argv);

                rpicam::CameraDaemon daemon;
                daemon.start(port);

                std::signal(SIGINT, signalHandler);
                std::signal(SIGTERM, signalHandler);

                std::cout << "rpicam-daemon listening on port " << port << std::endl;
                std::cout << "Press Ctrl+C to stop." << std::endl;

                while (keep_running)
                        std::this_thread::sleep_for(std::chrono::milliseconds(200));

                daemon.stop();
        }
        catch (std::exception const &ex)
        {
                std::cerr << "ERROR: " << ex.what() << std::endl;
                return EXIT_FAILURE;
        }

        return EXIT_SUCCESS;
}

