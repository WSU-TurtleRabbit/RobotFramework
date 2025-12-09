#ifndef LOGGER_H
#define LOGGER_H

#include <fstream>
#include <map>
#include <string>
#include <vector>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <iostream>

enum class LogLevel {
    INFO,
    WARN,
    CRIT
};

inline std::string logLevelToString(LogLevel level) {
    switch(level) {
        case LogLevel::INFO: return "INFO";
        case LogLevel::WARN: return "WARN";
        case LogLevel::CRIT: return "CRIT";
        default: return "UNKNOWN";
    }
}

struct LogEntry {
    long long timestamp_ms;
    std::map<std::string, double> numeric_data;
    std::string message;
    LogLevel level = LogLevel::INFO;
};

class Logger {
private:
    std::map<std::string, std::ofstream> files;
    std::map<std::string, std::vector<LogEntry>> buffers;
    std::string log_directory;
    bool is_initialized;
    const size_t BUFFER_SIZE = 100;
    std::chrono::high_resolution_clock::time_point start_time;

    std::string sanitize(double value) {
        if (std::isnan(value) || std::isinf(value)) return "null";
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(6) << value;
        return oss.str();
    }

public:
    Logger(const std::string& dir = "logs") 
        : log_directory(dir), is_initialized(false) {}

    ~Logger() { closeAll(); }

    bool initialize(const std::vector<std::string>& components) {
        system(("mkdir -p " + log_directory).c_str());
        start_time = std::chrono::high_resolution_clock::now();
        auto now = std::chrono::system_clock::now();
        auto t = std::chrono::system_clock::to_time_t(now);
        std::stringstream timestamp;
        timestamp << std::put_time(std::localtime(&t), "%Y%m%d_%H%M%S");

        for (const auto& comp : components) {
            std::string filename = log_directory + "/" + comp + "_" + timestamp.str() + ".log";
            files[comp].open(filename, std::ios::app);
            if (!files[comp].is_open()) {
                std::cerr << "Failed to open log file: " << filename << std::endl;
                return false;
            }
            buffers[comp] = {};
        }
        is_initialized = true;
        return true;
    }

    void log(const std::string& component, const std::map<std::string, double>& numeric_data) {
        log(component, numeric_data, "", LogLevel::INFO);
    }

    void log(const std::string& component, const std::string& message, LogLevel level = LogLevel::INFO) {
        log(component, {}, message, level);
    }

    void log(const std::string& component,
             const std::map<std::string, double>& numeric_data,
             const std::string& message,
             LogLevel level = LogLevel::INFO) 
    {
        if (!is_initialized || files.find(component) == files.end()) return;

        auto now = std::chrono::high_resolution_clock::now();
        long long elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();

        buffers[component].push_back({elapsed_ms, numeric_data, message, level});

        if (buffers[component].size() >= BUFFER_SIZE) flush(component);
    }

private:
    void flush(const std::string& component) {
        auto& file = files[component];
        auto& buf = buffers[component];
        if (buf.empty()) return;

        for (const auto& entry : buf) {
            file << std::setw(8) << std::setfill('0') << entry.timestamp_ms;

            for (const auto& kv : entry.numeric_data) {
                file << " | " << std::setw(10) << std::fixed << std::setprecision(6) << kv.second;
            }

            if (!entry.message.empty()) {
                file << " [" << logLevelToString(entry.level) << "] " << entry.message;
            }

            file << "\n";
        }

        file.flush();
        buf.clear();
    }

public:
    void flushAll() {
        for (auto& pair : buffers) flush(pair.first);
    }

    void closeAll() {
        flushAll();
        for (auto& pair : files)
            if (pair.second.is_open()) pair.second.close();
        files.clear();
        buffers.clear();
        is_initialized = false;
    }
};

#endif // LOGGER_H
