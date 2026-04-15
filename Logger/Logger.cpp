#include "Logger.h"

#include <iomanip>
#include <sstream>
#include <iostream>
#include <ctime>
#include <cmath>
#include <cstdlib>
#ifdef _WIN32
#include <io.h>
#define CHMOD(path, mode) _chmod(path, mode)
#else
#include <sys/stat.h>
#define CHMOD(path, mode) chmod(path, mode)
#endif

Logger::Logger(const std::string &dir) : log_directory(dir), is_initialized(false) {}

Logger::~Logger()
{
    closeAll();
}

std::string Logger::sanitize(double value)
{
    // Convert numeric values to a canonical string form.
    // NaN/Inf are rendered as "null" to avoid corrupting downstream parsing.
    if (std::isnan(value) || std::isinf(value))
    {
        return "null";
    }
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6) << value;
    return oss.str();
}

bool Logger::initialize(const std::vector<std::string> &components)
{
    // Prepare base directory and per-component files; sets a session timestamp used in filenames.
    // Repeated calls re-open files for new components while preserving the existing buffers.
    // Create base log directory
    system(("mkdir -p " + log_directory).c_str());
    // Make base directory writable/removable by any user
    CHMOD(log_directory.c_str(), 0777);
    start_time = std::chrono::high_resolution_clock::now();

    // Set the clock from now and keep the time since then
    auto now = std::chrono::system_clock::now();
    auto t = std::chrono::system_clock::to_time_t(now);
    std::stringstream timestamp;

    // Timeset to know when logs were created
    timestamp << std::put_time(std::localtime(&t), "%Y%m%d_%H%M%S");
    session_timestamp = timestamp.str();

    for (const auto &comp : components)
    {
        // Each component gets a dedicated file in its own directory.
        // Ensure component directory exists: logs/<component>
        std::string comp_dir = log_directory + "/" + comp;
        system(("mkdir -p " + comp_dir).c_str());

        // Ensure component directory is writable/removable by any user
        CHMOD(comp_dir.c_str(), 0777);

        // Default file for this component: logs/<component>/<component>_<timestamp>.log
        std::string filename = comp_dir + "/" + comp + "_" + session_timestamp + ".log";
        files[comp].open(filename, std::ios::app);
        if (!files[comp].is_open())
        {
            std::cerr << "Failed to open log file: " << filename << std::endl;
            return false;
        }
        // Make log file writable by any user
        CHMOD(filename.c_str(), 0666);
        buffers[comp] = {};
    }

    is_initialized = true;
    return true;
}

void Logger::log(const std::string &component, const std::map<std::string, double> &numeric_data)
{
    // Numeric-only entry; uses INFO level implicitly and empty message.
    log(component, numeric_data, "", LogLevel::INFO);
}

void Logger::log(const std::string &component, const std::string &message, LogLevel level)
{
    // Message-only entry; stores the level and an empty numeric map.
    log(component, std::map<std::string, double>{}, message, level);
}

void Logger::log(const std::string &component,
                 const std::map<std::string, double> &numeric_data,
                 const std::string &message,
                 LogLevel level)
{
    // Unified path for component logs. Ensures file is open, buffers the entry,
    // and triggers flush when buffer threshold is reached.
    if (!is_initialized || files.find(component) == files.end())
    {
        ensureOpen(component, component);
        if (files.find(component) == files.end()) return;
    }

    auto now = std::chrono::high_resolution_clock::now();
    long long elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();

    buffers[component].push_back({elapsed_ms, numeric_data, message, level});

    if (buffers[component].size() >= BUFFER_SIZE)
    {
        flush(component);
    }
}

void Logger::flush(const std::string &component)
{
    // Write buffered entries for a component to disk and clear the buffer.
    auto &file = files[component];
    auto &buf = buffers[component];
    if (buf.empty())
    {
        return;
    }

    for (const auto &entry : buf)
    {
        file << std::setw(8) << std::setfill('0') << entry.timestamp_ms;
        for (const auto &kv : entry.numeric_data)
        {
            // Only values are printed to keep lines compact.
            file << " | " << std::setw(10) << std::fixed << std::setprecision(6) << kv.second;
        }
        if (!entry.message.empty())
        {
            file << " [" << logLevelToString(entry.level) << "] " << entry.message;
        }
        file << "\n";
    }

    file.flush();
    buf.clear();
}

void Logger::flushAll()
{
    // Flush all component and sub-component buffers.
    for (auto &pair : buffers)
    {
        flush(pair.first);
    }
}

void Logger::closeAll()
{
    // Flush everything, close all files, and reset initialization state.
    flushAll();
    for (auto &pair : files)
    {
        if (pair.second.is_open())
        {
            pair.second.close();
        }
    }
    files.clear();
    buffers.clear();
    is_initialized = false;
}

void Logger::log(const std::string &component,
                 const std::string &sub,
                 const std::string &message,
                 LogLevel level)
{
    // Sub-component message-only logging. Key is "component/sub" and file lives under the component dir.
    std::string key = sub.empty() ? component : (component + "/" + sub);
    if (!is_initialized)
    {
        ensureOpen(key, component, sub);
    }
    else if (files.find(key) == files.end())
    {
        ensureOpen(key, component, sub);
    }

    if (files.find(key) == files.end()) return;

    auto now = std::chrono::high_resolution_clock::now();
    long long elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();
    buffers[key].push_back({elapsed_ms, {}, message, level});
    if (buffers[key].size() >= BUFFER_SIZE) flush(key);
}

// Sub-component numeric data logging: logs/<component>/<sub>_<timestamp>.log
void Logger::log(const std::string &component,
                 const std::string &sub,
                 const std::map<std::string, double> &numeric_data,
                 const std::string &message,
                 LogLevel level)
{
    // Sub-component numeric+message logging.
    std::string key = sub.empty() ? component : (component + "/" + sub);
    if (!is_initialized)
    {
        ensureOpen(key, component, sub);
    }
    else if (files.find(key) == files.end())
    {
        ensureOpen(key, component, sub);
    }

    if (files.find(key) == files.end()) return;

    auto now = std::chrono::high_resolution_clock::now();
    long long elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();
    buffers[key].push_back({elapsed_ms, numeric_data, message, level});
    if (buffers[key].size() >= BUFFER_SIZE) flush(key);
}

void Logger::log(const std::string &component,
                 const std::map<std::string, double> &numeric_data,
                 LogLevel level)
{
    // Numeric-only with explicit level; empty message.
    log(component, numeric_data, "", level);
}

void Logger::ensureOpen(const std::string &key, const std::string &component, const std::string &sub)
{
    // Lazily create/open a log file for the given component/sub key.
    // Ensures directories exist and permissions are set.
    if (files.find(key) != files.end()) return;

    // Ensure component dir exists
    std::string comp_dir = log_directory + "/" + component;
    system(("mkdir -p " + comp_dir).c_str());
    // Ensure permissions allow deletion by non-root users
    CHMOD(comp_dir.c_str(), 0777);

    // Use session timestamp if set, else compute once
    if (session_timestamp.empty())
    {
        auto now = std::chrono::system_clock::now();
        auto t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ts;
        ts << std::put_time(std::localtime(&t), "%Y%m%d_%H%M%S");
        session_timestamp = ts.str();
    }

    std::string prefix = sub.empty() ? component : sub;
    std::string filename = comp_dir + "/" + prefix + "_" + session_timestamp + ".log";
    files[key].open(filename, std::ios::app);
    if (!files[key].is_open())
    {
        std::cerr << "Failed to open log file: " << filename << std::endl;
        return;
    }
    // Make log file writable by any user
    CHMOD(filename.c_str(), 0666);
    buffers[key] = {};
}