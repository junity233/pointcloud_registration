#include "logger.hpp"

#include <algorithm>
#include <chrono>
#include <ctime>
#include <format>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace {
std::string current_timestamp() {
    using namespace std::chrono;
    const auto now = system_clock::now();
    const auto time_t = system_clock::to_time_t(now);

    std::tm tm_snapshot;
#if defined(_WIN32)
    localtime_s(&tm_snapshot, &time_t);
#else
    localtime_r(&time_t, &tm_snapshot);
#endif

    auto micros = duration_cast<microseconds>(now.time_since_epoch()) % 1'000'000;

    std::ostringstream oss;
    oss << std::put_time(&tm_snapshot, "%Y-%m-%d %H:%M:%S")
        << '.' << std::setw(6) << std::setfill('0') << micros.count();
    return oss.str();
}
} // namespace

void Logger::set_level(LogLevel level) {
    _level.store(level);
}

LogLevel Logger::level() const {
    return _level.load();
}

void Logger::log_impl(LogLevel level, std::string_view role, std::string_view message) {
    std::scoped_lock lock(_mutex);
    auto &stream = (level == LogLevel::Error) ? std::cerr : std::cout;
    const auto role_view = role.empty() ? std::string_view{"-"} : role;
    stream << std::format("[{}] [{}] [{}] {}", current_timestamp(),
                          level_to_string(level), role_view, message)
           << '\n';
    stream.flush();
}

void Logger::progress(double ratio, std::size_t completed, std::size_t total) {
    std::scoped_lock lock(_mutex);

    if (total == 0) {
        std::cout << "\rProgress: 0.00% (0/0)" << std::flush;
        return;
    }

    ratio = std::clamp(ratio, 0.0, 1.0);
    const double percent = ratio * 100.0;
    std::cout << '\r'
              << std::format("Progress: {:6.2f}% ({}/{})",
                             percent, completed, total)
              << std::flush;

    if (completed >= total) {
        std::cout << '\n';
    }
}

std::string_view Logger::level_to_string(LogLevel level) {
    switch (level) {
    case LogLevel::Debug:
        return "DEBUG";
    case LogLevel::Info:
        return "INFO";
    case LogLevel::Warn:
        return "WARN";
    case LogLevel::Error:
        return "ERROR";
    default:
        return "UNKNOWN";
    }
}
