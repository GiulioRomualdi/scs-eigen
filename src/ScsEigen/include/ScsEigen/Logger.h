/**
 * @file Logger.h
 * @authors Giulio Romualdi
 * @copyright Released under the terms of the MIT License.
 */

#ifndef SCS_EIGEN_LOGGER_H
#define SCS_EIGEN_LOGGER_H

#include <string>

namespace ScsEigen
{

/**
 * Logger is a Singleton that can be used to print info errors and warnings in the terminal.
 */
class Logger
{

public:
    /**
     * @brief Level is an enum representing the level of the print.
     * @note If the level is set to Logger::Level::error only the errors will be print. If set to
     * Logger::Level::info warnings and errors are printed.
     */
    enum class Level : std::size_t
    {
        none = 0,
        error = 1,
        warning = 2,
        info = 3,
        debug = 4
    };

    friend Logger* const log();

    /**
     * @brief Print an error message.
     */
    void error(std::string_view error);

    /**
     * @brief Print an info message.
     */
    void info(std::string_view info);

    /**
     * @brief Print a warning message.
     */
    void warning(std::string_view warning);

    /**
     * @brief Print a debug message.
     */
    void debug(std::string_view debug);

    /**
     * @brief set the level of the verbosity.
     * @note If the level is set to Logger::Level::error only the errors will be print. If set to
     * Logger::Level::info warnings and errors are printed. Set the level equal to
     * Logger::Level::none if you want to disable all the messages.
     */
    void setLevel(Level level);

private:
    Level m_level{Level::info};
};

/**
 * Get an the instance of the log
 */
Logger* const log();

} // namespace ScsEigen

#endif // SCS_EIGEN_LOGGER_H
