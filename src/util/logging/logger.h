/**
 * @file logger.h
 * @author Marcel Flottmann
 * @date 2020-09-02
 */

#pragma once

#include <mutex>
#include <sstream>
#include <chrono>
#include <iomanip>
#include <vector>
#include <memory>
#include <algorithm>

#include "sink.h"

namespace FastSense::util::logging
{

enum class LogLevel
{
    Debug,
    Info,
    Warning,
    Error,
    Fatal
};

class Logger
{
    Logger();

    static Logger& getInst();

    template<typename T, typename ...Args>
    void print(std::ostringstream& msg, const T& val, const Args& ...args);

    void print(std::ostringstream& msg);

    template<typename ...Args>
    void writeMessage(LogLevel level, const Args& ...args);

    const char* const logLevelToString[5] =
    {
        "[DEBUG]   ",
        "[INFO]    ",
        "[WARNING] ",
        "[ERROR]   ",
        "[FATAL]   ",
    };

    std::mutex mtx;

    LogLevel currentLogLevel;

    std::vector<std::shared_ptr<sink::Sink>> sinks;

public:
    template<typename ...Args>
    static void debug(const Args& ...args);

    template<typename ...Args>
    static void info(const Args& ...args);

    template<typename ...Args>
    static void warning(const Args& ...args);

    template<typename ...Args>
    static void error(const Args& ...args);

    template<typename ...Args>
    static void fatal(const Args& ...args);

    template<typename ...Args>
    static void log(LogLevel level, const Args& ...args);

    static void setLoglevel(LogLevel level);

    static void addSink(const std::shared_ptr<sink::Sink>& s);

    static void removeSink(const std::shared_ptr<sink::Sink>& s);
};

}

#include "logger.tcc"
