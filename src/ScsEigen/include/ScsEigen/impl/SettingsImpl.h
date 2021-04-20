/**
 * @file SettingsImpl.h
 * @author Giulio Romualdi
 * @copyright Released under the terms of the MIT License.
 * @date 2021
 */

#ifndef SCS_EIGEN_SETTINGS_IMPL_H
#define SCS_EIGEN_SETTINGS_IMPL_H

#include <string>

#include <scs.h>

#include <ScsEigen/Settings.h>

/**
 * ScsEigen namespace.
 */
namespace ScsEigen
{


struct Settings::Impl
{
    ScsSettings settings; /**< Scs Setting struct */
    std::string filename; /**< name of the file where the solution is stored. */
};

} // namespace ScsEigen

#endif // SCS_EIGEN_SETTINGS_IMPL_H
