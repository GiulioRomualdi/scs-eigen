/**
 * @file Logger.cpp
 * @author Giulio Romualdi
 * @copyright Released under the terms of the MIT License.
 * @date 2021
 */

#include <ScsEigen/Logger.h>

#include <iostream>

using namespace ScsEigen;

template <typename E> constexpr auto toUnderlinedType(E e) -> typename std::underlying_type<E>::type
{
    return static_cast<typename std::underlying_type<E>::type>(e);
}

Logger* const ScsEigen::log()
{
    static Logger l;
    return &l;
}

void Logger::error(std::string_view error)
{
    if (toUnderlinedType(m_level) >= toUnderlinedType(Level::error))
    {
        std::cout << "[ERROR] " << error << std::endl;
    }
}

void Logger::info(std::string_view info)
{
    if (toUnderlinedType(m_level) >= toUnderlinedType(Level::info))
    {
        std::cout << "[INFO] " << info << std::endl;
    }
}

void Logger::warning(std::string_view warning)
{
    if (toUnderlinedType(m_level) >= toUnderlinedType(Level::warning))
    {
        std::cout << "[WARNING] " << warning << std::endl;
    }
}

void Logger::debug(std::string_view debug)
{
    if (toUnderlinedType(m_level) >= toUnderlinedType(Level::debug))
    {
        std::cout << "[DEBUG] " << debug << std::endl;
    }
}

void Logger::setLevel(Level level)
{
    m_level = level;
}
