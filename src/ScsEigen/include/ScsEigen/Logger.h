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
class Logger
{

public:
    enum class Level : std::size_t
    {
        error = 0,
        warning = 1,
        info = 2,
        debug = 3
    };

    friend Logger* const log();

    void error(std::string_view error);
    void info(std::string_view info);
    void warning(std::string_view warning);
    void debug(std::string_view debug);

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
