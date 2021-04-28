/**
 * @file Solver.h
 * @author Giulio Romualdi
 * @copyright Released under the terms of the MIT License.
 * @date 2021
 */
#ifndef SCS_EIGEN_SOLVER_H
#define SCS_EIGEN_SOLVER_H

// Std
#include <memory>

// Eigen
#include <Eigen/Dense>

#include <ScsEigen/MathematicalProgram.h>
#include <ScsEigen/Settings.h>
#include <ScsEigen/Solution.h>

/**
 * ScsEigen namespace.
 */
namespace ScsEigen
{

/**
 * Solver class is a wrapper of the scs library.
 * This is the main class that you should use when you want to solve a convex optimization problem.
 * This [example](./pages/qp.md) can be used as a reference to solve a QP problem.
 */
class Solver
{
    struct Impl;
    std::unique_ptr<Impl> m_pimpl;

public:
    /**
     * @brief Constructor.
     */
    Solver();

    /**
     * @brief Destructor.
     */
    ~Solver();

    /**
     * @brief Access to the ScsEigen::Settings object.
     */
    Settings& settings();

    /**
     * @brief Access to the ScsEigen::Settings object.
     */
    const Settings& settings() const;

    /**
     * @brief Access to the ScsEigen::MathematicalProgram object.
     */
    MathematicalProgram& mathematicalProgram();

    /**
     * @brief Access to the ScsEigen::MathematicalProgram object.
     */
    const MathematicalProgram& mathematicalProgram() const;

    /**
     * @brief Solve the optimization problem described in MathematicalProgram class
     * @return true in case of success/false otherwise.
     */
    bool solve();

    /**
     * @brief Access to ScsEigen::Solution object.
     */
    const Solution& solution() const;
};

} // namespace ScsEigen

#endif // SCS_EIGEN_SOLVER_H
