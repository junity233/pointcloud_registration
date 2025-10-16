#pragma once

#include <atomic>
#include <format>
#include <mutex>
#include <string_view>
#include <utility>
#include "singleton.hpp"

enum class LogLevel { Debug = 0, Info, Warn, Error };

class Logger: public Singleton<Logger> {
public:
    void set_level(LogLevel level);
    LogLevel level() const;

    friend class Singleton<Logger>;

    template <typename... Args>
    void log(LogLevel level,
             std::string_view role,
             std::format_string<Args...> fmt,
             Args &&...args) {
        if (static_cast<int>(level) < static_cast<int>(_level.load())) {
            return;
        }
        const auto message = std::format(fmt, std::forward<Args>(args)...);
        log_impl(level, role, message);
    }

    template <typename... Args>
    void log_info(std::format_string<Args...> fmt, Args &&...args) {
        log(LogLevel::Info, "", fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    void log_warn(std::format_string<Args...> fmt, Args &&...args) {
        log(LogLevel::Warn, "", fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    void log_error(std::format_string<Args...> fmt, Args &&...args) {
        log(LogLevel::Error, "", fmt, std::forward<Args>(args)...);
    }

    void progress(double ratio, std::size_t completed, std::size_t total);

private:
    Logger() = default;

    void log_impl(LogLevel level, std::string_view role, std::string_view message);
    static std::string_view level_to_string(LogLevel level);

    mutable std::mutex _mutex;
    std::atomic<LogLevel> _level{LogLevel::Info};
};

#define LOG_LOGGER_CALL(level, role, fmt, ...)                                                   \
    Logger::instance().log(level, role, fmt __VA_OPT__(, ) __VA_ARGS__)

#define LOG_DEBUG(role, fmt, ...) LOG_LOGGER_CALL(LogLevel::Debug, role, fmt __VA_OPT__(, ) __VA_ARGS__)
#define LOG_INFO(role, fmt, ...) LOG_LOGGER_CALL(LogLevel::Info, role, fmt __VA_OPT__(, ) __VA_ARGS__)
#define LOG_WARN(role, fmt, ...) LOG_LOGGER_CALL(LogLevel::Warn, role, fmt __VA_OPT__(, ) __VA_ARGS__)
#define LOG_ERROR(role, fmt, ...) LOG_LOGGER_CALL(LogLevel::Error, role, fmt __VA_OPT__(, ) __VA_ARGS__)

template <typename T>
class LoggerAble {
protected:
    template <typename... Args>
    void log(LogLevel level, std::format_string<Args...> fmt, Args &&...args) const {
        Logger::instance().log(level, reinterpret_cast<const T*>(this)->name(), fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    void log_info(std::format_string<Args...> fmt, Args &&...args) const {
        log(LogLevel::Info, fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    void log_warn(std::format_string<Args...> fmt, Args &&...args) const {
        log(LogLevel::Warn, fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    void log_error(std::format_string<Args...> fmt, Args &&...args) const {
        log(LogLevel::Error, fmt, std::forward<Args>(args)...);
    }
};
