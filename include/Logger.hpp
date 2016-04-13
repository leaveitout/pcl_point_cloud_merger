//
// Created by sean on 06/12/15.
//

#ifndef PCL_CLOUD_REGISTRATION_LOGGER_HPP
#define PCL_CLOUD_REGISTRATION_LOGGER_HPP

#include <string>
#include <pcl/console/print.h>

namespace Logger {

    enum Level {
        ALWAYS = pcl::console::L_ALWAYS,
        ERROR = pcl::console::L_ERROR,
        WARN = pcl::console::L_WARN,
        INFO = pcl::console::L_INFO,
        DEBUG = pcl::console::L_DEBUG,
        VERBOSE = pcl::console::L_VERBOSE
    };

    void log(Logger::Level level, const char *format, ...);

    inline void log(Logger::Level level, const std::string& str) {
        log(level, str.c_str());
    }

    void log(const char* format, ...);

    inline void log(const std::string& str) {
        log(Level::INFO, str.c_str());
    }
}

#endif //PCL_CLOUD_REGISTRATION_LOGGER_HPP
