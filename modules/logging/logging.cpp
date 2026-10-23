#include "logging.hpp"

#include "engine/kernel/filesystem_utilities.hpp"

#include <chrono>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <iostream>

#if defined(_WIN32)
// WIN32_LEAN_AND_MEAN and NOMINMAX are defined for the whole build through
// the root CMakeLists; windows.h must not be included without them.
#include <io.h>
#include <windows.h>
#else
#include <unistd.h>
#endif

namespace engine
{

namespace
{

bool standardOutputIsTerminal()
{
#if defined(_WIN32)
    return ::_isatty(::_fileno(stdout)) != 0;
#else
    return ::isatty(::fileno(stdout)) != 0;
#endif
}

LogLevel parseLevelName(
    const std::string &levelName
)
{
    if (levelName == "trace")
    {
        return LogLevel::Trace;
    }
    if (levelName == "debug")
    {
        return LogLevel::Debug;
    }
    if (levelName == "information")
    {
        return LogLevel::Information;
    }
    if (levelName == "warning")
    {
        return LogLevel::Warning;
    }
    if (levelName == "error")
    {
        return LogLevel::Error;
    }
    return LogLevel::Information;
}

} // namespace

LoggingModule::LoggingModule()
{
    componentManifest.name                 = "logging";
    componentManifest.kind                 = ComponentKind::Module;
    componentManifest.providedCapabilities = {"logging"};
    componentManifest.requiredCapabilities = {};
    componentManifest.requiredPermissions  = {};
    componentManifest.knownConfigKeys      = {"logging.level", "logging.file"};
}

void LoggingModule::construct(
    ComponentContext &context
)
{
    const std::string levelName = context.configurationString("level").value_or("information");
    minimumLevel                = parseLevelName(levelName);

    const std::string fileName = context.configurationString("file").value_or("");
    if (!fileName.empty())
    {
        const std::filesystem::path filePath = std::filesystem::path(fileName);
        if (filePath.is_relative())
        {
            fileSink.open(executableDirectory() / filePath, std::ios::out | std::ios::app);
        }
        else
        {
            fileSink.open(filePath, std::ios::out | std::ios::app);
        }
    }
}

void LoggingModule::start()
{
    logMessage(LogLevel::Information, "logging module started");
}

void LoggingModule::stop()
{
    if (fileSink.is_open())
    {
        fileSink.flush();
        fileSink.close();
    }
}

void *LoggingModule::capabilityPointerFor(
    const std::string &capabilityName
)
{
    if (capabilityName == LoggerCapability::capabilityName)
    {
        return static_cast<LoggerCapability *>(this);
    }
    return nullptr;
}

const char *LoggingModule::levelTag(
    LogLevel level
)
{
    switch (level)
    {
    case LogLevel::Trace:
        return "TRACE";
    case LogLevel::Debug:
        return "DEBUG";
    case LogLevel::Information:
        return "INFO ";
    case LogLevel::Warning:
        return "WARN ";
    case LogLevel::Error:
        return "ERROR";
    }
    return "LOG  ";
}

const char *LoggingModule::levelColor(
    LogLevel level
)
{
    switch (level)
    {
    case LogLevel::Trace:
        return "\033[90m";
    case LogLevel::Debug:
        return "\033[36m";
    case LogLevel::Information:
        return "\033[32m";
    case LogLevel::Warning:
        return "\033[33m";
    case LogLevel::Error:
        return "\033[31m";
    }
    return "";
}

void LoggingModule::logMessage(
    LogLevel level, const std::string &message
)
{
    if (static_cast<int>(level) < static_cast<int>(minimumLevel))
    {
        return;
    }

    const std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm           timeInformation{};
#if defined(_WIN32)
    ::localtime_s(&timeInformation, &now);
#else
    ::localtime_r(&now, &timeInformation);
#endif
    char timestamp[32] = {};
    std::strftime(timestamp, sizeof(timestamp), "%H:%M:%S", &timeInformation);

    const std::lock_guard<std::mutex> guard(sinkMutex);
    if (standardOutputIsTerminal())
    {
        std::cout << levelColor(level) << "[" << timestamp << " " << levelTag(level) << "]\033[0m " << message << "\n";
    }
    else
    {
        std::cout << "[" << timestamp << " " << levelTag(level) << "] " << message << "\n";
    }
    std::cout.flush();

    if (fileSink.is_open())
    {
        fileSink << "[" << timestamp << " " << levelTag(level) << "] " << message << "\n";
        fileSink.flush();
    }
}

} // namespace engine
