#include <iostream>
#include <chrono>
#include <thread>
#include <map>
#include <vector>
#include <string>
#include <csignal>
#include <atomic>
#include <algorithm>

#include <Arduino/arduino.h>
#include "Logger.h"

// --- Forward declaration for signal handler ---
void signalHandler(int signum);

static std::atomic<bool> stop_requested{false};
static Arduino ard;
static Logger logger("arduino_logs");
static bool log_enabled = false;

int main(int argc, char **argv)
{
    int blink_count = 0;
    int kick_count = 0;
    int dribble_duration = 0;
    int stop_count = 0;

    // --- Setup signal handler to catch Ctrl+C ---
    std::signal(SIGINT, signalHandler);

    // Parse flags
    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];

        if (arg == "-l" || arg == "--log")
        {
            log_enabled = true;
        }
        else if ((arg == "-b" || arg == "--blink") && i + 1 < argc)
        {
            blink_count = std::stoi(argv[++i]);
        }
        else if ((arg == "-k" || arg == "--kick") && i + 1 < argc)
        {
            kick_count = std::stoi(argv[++i]);
        }
        else if ((arg == "-d" || arg == "--dribble") && i + 1 < argc)
        {
            dribble_duration = std::stoi(argv[++i]);
        }
        else if ((arg == "-s" || arg == "--stop") && i + 1 < argc)
        {
            stop_count = std::stoi(argv[++i]);
        }
        else
        {
            std::cerr << "Unknown flag or argument: " << arg << std::endl;
        }
    }

    // Initialize logger if enabled
    if (log_enabled)
    {
        if (!logger.initialize({"arduino"}))
        {
            std::cerr << "Failed to initialize logger" << std::endl;
            return 1;
        }
        std::cout << "Logging enabled to arduino_logs/" << std::endl;
    }

    // Initialize Arduino
    if (!ard.findArduino())
    {
        std::cerr << "Failed to find Arduino" << std::endl;
        if (log_enabled)
            logger.log("arduino", "Failed to find Arduino", LogLevel::CRIT);
        return 1;
    }

    std::cout << "Arduino connected on port: " << ard.getPort() << std::endl;
    if (log_enabled)
        logger.log("arduino", "Arduino connected on port: " + ard.getPort(), LogLevel::INFO);

    if (!ard.connect(ard.getPort()))
    {
        std::cerr << "Failed to connect to Arduino" << std::endl;
        if (log_enabled)
            logger.log("arduino", "Failed to connect to Arduino", LogLevel::CRIT);
        return 1;
    }

    std::cout << "Connected successfully" << std::endl;
    if (log_enabled)
        logger.log("arduino", "Connected successfully", LogLevel::INFO);

    // --- Execute Commands ---
    // (In Order of Blink, Kick, Dribble, Stop)

    // Blink
    if (blink_count > 0)
    {
        std::cout << "\nExecuting " << blink_count << " blink commands..." << std::endl;
        for (int i = 0; i < blink_count; ++i)
        {
            if (stop_requested.load()) break;
            if (ard.sendCommand('B'))
            {
                std::cout << "Blink " << (i + 1) << " sent" << std::endl;
                if (log_enabled)
                    logger.log("arduino", "Blink command sent", LogLevel::INFO);
                for (int ms = 0; ms < 500 && !stop_requested.load(); ms += 50)
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
            else
            {
                std::cerr << "Failed to send blink command " << (i + 1) << std::endl;
                if (log_enabled)
                    logger.log("arduino", "Failed to send blink command", LogLevel::WARN);
                break;
            }
        }
    }

    // Kick
    if (kick_count > 0)
    {
        std::cout << "\nExecuting " << kick_count << " kick commands..." << std::endl;
        for (int i = 0; i < kick_count; ++i)
        {
            if (stop_requested.load()) break;
            if (ard.sendCommand('K'))
            {
                std::cout << "Kick " << (i + 1) << " sent" << std::endl;
                if (log_enabled)
                    logger.log("arduino", "Kick command sent", LogLevel::INFO);
                for (int ms = 0; ms < 500 && !stop_requested.load(); ms += 50)
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
            else
            {
                std::cerr << "Failed to send kick command " << (i + 1) << std::endl;
                if (log_enabled)
                    logger.log("arduino", "Failed to send kick command", LogLevel::WARN);
                break;
            }
        }
    }

    // Dribble
    if (dribble_duration > 0)
    {
        std::cout << "\nStarting dribbler for " << dribble_duration << " seconds..." << std::endl;

        if (ard.sendCommand('D'))
        {
            if (log_enabled)
                logger.log("arduino", "Dribble started", LogLevel::INFO);

            auto start_time = std::chrono::steady_clock::now();

            while (true)
            {
                if (stop_requested.load()) break;
                auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                                   std::chrono::steady_clock::now() - start_time)
                                   .count();

                if (elapsed >= dribble_duration)
                {
                    std::cout << "Dribble duration complete" << std::endl;
                    break;
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            if (ard.sendCommand('S'))
            {
                std::cout << "Sent stop dribble command" << std::endl;
                if (log_enabled)
                    logger.log("arduino", "Dribble stopped", LogLevel::INFO);
            }
        }
        else
        {
            std::cerr << "Failed to send dribble command" << std::endl;
            if (log_enabled)
                logger.log("arduino", "Failed to send dribble command", LogLevel::CRIT);
        }
    }

    // Stop
    if (stop_count > 0)
    {
        std::cout << "\nExecuting " << stop_count << " stop commands..." << std::endl;
        for (int i = 0; i < stop_count; ++i)
        {
            if (stop_requested.load()) break;
            if (ard.sendCommand('S'))
            {
                std::cout << "Stop " << (i + 1) << " sent" << std::endl;
                if (log_enabled)
                    logger.log("arduino", "Stop command sent", LogLevel::INFO);
                for (int ms = 0; ms < 500 && !stop_requested.load(); ms += 50)
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
            else
            {
                std::cerr << "Failed to send stop command " << (i + 1) << std::endl;
                if (log_enabled)
                    logger.log("arduino", "Failed to send stop command", LogLevel::WARN);
                break;
            }
        }
    }

    // If a stop was requested, ensure device is commanded to stop once here
    if (stop_requested.load()) {
        ard.sendCommand('S');
        if (log_enabled)
            logger.log("arduino", "Stopped by Ctrl^C", LogLevel::DONE);
    }

    // Disconnect
    ard.disconnect();
    std::cout << "\nTest complete. Arduino disconnected." << std::endl;

    if (log_enabled)
    {
        logger.log("arduino", "Test complete", LogLevel::DONE);
        logger.closeAll();
    }

    return 0;
}

// --- Signal handler for Ctrl+C ---
void signalHandler(int signum)
{
    // Only set an atomic flag; handle stop on main thread
    stop_requested.store(true);
}
