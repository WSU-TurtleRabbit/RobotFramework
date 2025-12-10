#ifndef LOGGER_H
#define LOGGER_H

#include <fstream>
#include <map>
#include <string>
#include <vector>
#include <chrono>

enum class LogLevel
{
    INFO,
    WARN,
    CRIT,
    DONE,
    FAIL,
    MISC,
    LOVE,
    HATE
};

inline std::string logLevelToString(LogLevel level)
{
    switch (level)
    {
    case LogLevel::INFO:
        return "INFO";
    case LogLevel::WARN:
        return "WARN";
    case LogLevel::CRIT:
        return "CRIT";

    case LogLevel::DONE:
        return "DONE";
    case LogLevel::FAIL:
        return "FAIL";

    case LogLevel::MISC:
        return "MISC";
    
    case LogLevel::LOVE:
        return "LOVE";
    case LogLevel::HATE:
        return "HATE";

    default:
        return "----";
    }
}

struct LogEntry
{
    long long timestamp_ms;
    std::map<std::string, double> numeric_data;
    std::string message;
    LogLevel level = LogLevel::INFO;
};

class Logger
{
private:
    std::map<std::string, std::ofstream> files;
    std::map<std::string, std::vector<LogEntry>> buffers;
    std::string log_directory;
    bool is_initialized;
    const size_t BUFFER_SIZE = 100;
    std::chrono::high_resolution_clock::time_point start_time;
    std::string session_timestamp;

    std::string sanitize(double value);
    void flush(const std::string &component);
    void ensureOpen(const std::string &key, const std::string &component, const std::string &sub = "");

public:
    explicit Logger(const std::string &dir = "logs");
    ~Logger();

    bool initialize(const std::vector<std::string> &components);

    void log(const std::string &component, const std::map<std::string, double> &numeric_data);
    void log(const std::string &component, const std::string &message, LogLevel level = LogLevel::INFO);
    void log(const std::string &component,
             const std::map<std::string, double> &numeric_data,
             const std::string &message,
             LogLevel level = LogLevel::INFO);

    void log(const std::string &component,
             const std::string &sub,
             const std::string &message,
             LogLevel level = LogLevel::INFO);

    void log(const std::string &component,
             const std::string &sub,
             const std::map<std::string, double> &numeric_data,
             const std::string &message = "",
             LogLevel level = LogLevel::INFO);

    void flushAll();
    void closeAll();
};

#endif // LOGGER_H
