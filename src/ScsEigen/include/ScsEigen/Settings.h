/**
 * @file Settings.h
 * @author Giulio Romualdi
 * @copyright Released under the terms of the MIT License.
 * @date 2021
 */

#ifndef SCS_EIGEN_SETTINGS_H
#define SCS_EIGEN_SETTINGS_H

#include <memory>

/**
 * ScsEigen namespace.
 */
namespace ScsEigen
{
/**
 * Settings class is a wrapper of the SCS Settings struct.
 * @note You can find further information here:
 * https://github.com/cvxgrp/scs/blob/48dfbe81caad2162c3ce5757faccdb3f3d31e142/include/scs.h#L44-L63
 */

class Solver;

class Settings
{
    struct Impl;
    std::unique_ptr<Impl> m_pimpl;

    friend class Solver;

public:

    /**
     * @brief Constructor.
     */
    Settings();

    /**
     * brief Destructor, Required for the pimpl idiom.
     */
    ~Settings();

    /**
     * brief Enable the heuristic data rescaling.
     * @param normalize if true the rescaling is enabled.
     * @warning If you call this function you should reinitialize the problem.
     */
    void normalize(bool normalize);

    /**
     * brief Is rescaling enabled?
     * @return true if the rescaling is enabled.
     */
    bool normalize() const;

    /**
     * @brief Set the scaling factor.
     * @param scale a positive number.
     * @return true in case of success, false otherwise.
     * @warning If you call this function you should reinitialize the problem.
     */
    bool scale(double scale);

    /**
     * @brief Get the scaling factor
     * @return the scaling factor.
     */
    double scale() const;

    /**
     * @brief Set the equality constraint scaling.
     * @param rho a positive number.
     * @return true in case of success, false otherwise.
     * @warning If you call this function you should reinitialize the problem.
     */
    bool rho(double rho);

    /**
     * @brief Get the scaling factor.
     * @return the scaling factor.
     */
    double rho() const;

    /**
     * @brief Set the maximum number of iterations.
     * @param iterations number of iterations
     * @return true in case of success, false otherwise.
     */
    bool maxIterations(int iterations);

    /**
     * @brief Get the number of iterations.
     * @return the iterations.
     */
    int maxIterations() const;

    /**
     * @brief Set the convergence tolerance.
     * @param eps convergence tolerance.
     * @return true in case of success, false otherwise.
     */
    bool eps(double eps);

    /**
     * @brief Get the convergence tolerance.
     * @return the tolerance.
     */
    double eps() const;

    /**
     * @brief Set the relaxation parameter.
     * @param alpha relaxation parameter.
     * @return true in case of success, false otherwise.
     */
    bool alpha(double alpha);

    /**
     * @brief Get the relaxation parameter.
     * @return the parameter.
     */
    double alpha() const;

    /**
     * @brief Set the verbosity of the solver.
     * @param isVerbose if true the solver will be verbose.
     */
    void verbose(bool isVerbose);

    /**
     * @brief Is solver verbose?
     * @return true if the solver is verbose.
     */
    bool verbose() const;

    /**
     * @brief Enable the warm start.
     * @param warmStart if true the warm start is enable
     */
    void warmStart(bool warmStart);

    /**
     * @brief Is warm start enabled?
     * @return true if warm start is enabled.
     */
    bool warmStart() const;

    /**
     * @brief Set the memory for acceleration.
     * @param accelerationLookBack memory acceleration.
     * @return true if the solver is verbose.
     */
    bool accelerationLookback(int accelerationLookback);

    /**
     * @brief Get the memory acceleration.
     * @return the memory acceleration.
     */
    int accelerationLookback();

    /**
     * @brief Set the filename used to store the data.
     * @filename name of the file where the data will be stored.
     */
    void writeDataFilename(std::string_view filename);

    /**
     * @brief Get the filename.
     * @return a string containing the filename.
     */
    std::string_view writeDataFilename() const;
};

} // namespace ScsEigen

#endif // SCS_EIGEN_SETTINGS_H
