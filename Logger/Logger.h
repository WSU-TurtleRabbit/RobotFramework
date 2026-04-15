#ifndef LOGGER_H
#define LOGGER_H

#include <fstream>
#include <map>
#include <string>
#include <vector>
#include <chrono>

/*
* Logger
*
* Purpose:
* - Lightweight, buffered component logger for telemetry and events.
* - Creates a base log directory (e.g., "logs") and one subdirectory per component.
* - Writes lines containing timestamp, optional numeric values, and optional message + level.
*
* File structure:
* - Base dir: <log_directory>/
* - Component dir: <log_directory>/<component>/
* - Default file: <log_directory>/<component>/<component>_<session_timestamp>.log
* - Sub-component file: <log_directory>/<component>/<sub>_<session_timestamp>.log
*
* Line format:
* - Timestamp (ms since Logger::initialize) zero-padded to 8 chars
* - Followed by " | value" for each numeric datum (map values only; keys are not printed)
* - If message is present: " [LEVEL] message"
* - Always ends with a newline
*
* Buffering:
* - Each component/sub has an in-memory buffer of log entries.
* - When buffer size reaches BUFFER_SIZE, flush() writes and clears it.
* - flushAll() and closeAll() ensure all buffered logs are written.
*
* Permissions:
* - Directories are chmod 0777 and files 0666 to avoid permission issues across users.
*
* Usage:
*  Logger logger("logs");
*  logger.initialize({"motor", "network"});
*  logger.log("motor", std::map<std::string,double>{{"vel", 1.0}});        // numeric-only
*  logger.log("motor", "Started", LogLevel::INFO);                         // message-only
*  logger.log("motor", std::map<std::string,double>{{"cur", 2.0}}, "tick", LogLevel::MISC);
*  logger.log("motor", "subA", "calibrated", LogLevel::DONE);              // sub-component message
*  logger.log("motor", "subA", std::map<std::string,double>{{"err", 0.01}}, "", LogLevel::INFO);
*  logger.closeAll();
*/

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

    void log(const std::string &component,
             const std::map<std::string, double> &numeric_data,
             LogLevel level);

    void flushAll();
    void closeAll();
};

#endif // LOGGER_H
