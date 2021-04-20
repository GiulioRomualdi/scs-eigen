/**
 * @file Settings.cpp
 * @author Giulio Romualdi
 * @copyright Released under the terms of the MIT License.
 * @date 2021
 */

// scs
#include <scs.h>

#include <ScsEigen/Settings.h>
#include <ScsEigen/impl/SettingsImpl.h>

using namespace ScsEigen;

Settings::Settings()
    : m_pimpl(std::make_unique<Impl>())
{
}

Settings::~Settings() = default;

void Settings::normalize(bool normalize)
{
    m_pimpl->settings.normalize = normalize ? 1 : 0;
}
bool Settings::normalize() const
{
    return m_pimpl->settings.normalize == 1;
}

bool Settings::scale(double scale)
{
    if (scale < 0)
    {
        return false;
    }

    m_pimpl->settings.scale = scale;
    return true;
}

double Settings::scale() const
{
    return m_pimpl->settings.scale;
}

bool Settings::rho(double rho)
{
    if (rho < 0)
    {
        return false;
    }

    m_pimpl->settings.rho_x = rho;
    return true;
}

double Settings::rho() const
{
    return m_pimpl->settings.rho_x;
}

bool Settings::maxIterations(int iterations)
{
    if (iterations < 0)
    {
        return false;
    }

    m_pimpl->settings.max_iters = iterations;
    return true;
}

int Settings::maxIterations() const
{
    return m_pimpl->settings.max_iters;
}

bool Settings::eps(double eps)
{
    if (eps < 0)
    {
        return false;
    }

    m_pimpl->settings.eps = eps;
    return true;
}

double Settings::eps() const
{
    return m_pimpl->settings.eps;
}

bool Settings::alpha(double alpha)
{
    if (alpha < 0)
    {
        return false;
    }

    m_pimpl->settings.alpha = alpha;
    return true;
}

double Settings::alpha() const
{
    return m_pimpl->settings.alpha;
}

void Settings::verbose(bool verbose)
{
    m_pimpl->settings.verbose = verbose ? 1 : 0;
}

bool Settings::verbose() const
{
    return m_pimpl->settings.verbose == 1;
}

void Settings::warmStart(bool warmStart)
{
    m_pimpl->settings.warm_start = warmStart ? 1 : 0;
}

bool Settings::warmStart() const
{
    return m_pimpl->settings.warm_start == 1;
}

bool Settings::accelerationLookback(int accelerationLookback)
{
    if (accelerationLookback < 0)
    {
        return false;
    }

    m_pimpl->settings.acceleration_lookback = accelerationLookback;
    return true;
}

void Settings::writeDataFilename(std::string_view filename)
{
    m_pimpl->filename = filename;
    m_pimpl->settings.write_data_filename = m_pimpl->filename.c_str();
}

std::string_view Settings::writeDataFilename() const
{
    return m_pimpl->settings.write_data_filename;
}
