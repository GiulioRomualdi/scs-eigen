/**
 * @file Solver.h
 * @author Giulio Romualdi
 * @copyright  Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */
#ifndef SCS_EIGEN_SOLVER_H
#define SCS_EIGEN_SOLVER_H

// Std
#include <memory>

// Eigen
#include <Eigen/Dense>

#include <ScsEigen/Settings.h>
#include <ScsEigen/MathematicalProgram.h>

/**
 * ScsEigen namespace.
 */
namespace ScsEigen
{

/**
 * Solver class is a wrapper of the scs library
 */
class Solver
{
    struct Impl;
    std::unique_ptr<Impl> m_pimpl;

public:

    /**
     * Constructor.
     */
    Solver();

    /**
     * Destructor.
     */
    ~Solver();

    /**
     * Access to the ScsEigen::Settings object.
     */
    Settings& settings();

    /**
     * Access to the ScsEigen::Settings object.
     */
    const Settings& settings() const;

    /**
     * Access to the ScsEigen::MathematicalProgram object.
     */
    MathematicalProgram& mathematicalProgram();

    /**
     * Access to the ScsEigen::MathematicalProgram object.
     */
    const MathematicalProgram& mathematicalProgram() const;

    /**
     * Solve the optimization problem described in MathematicalProgram class
     * @return true in case of success/false otherwise.
     */
    bool solve();

};

} // namespace ScsEigen

#endif // SCS_EIGEN_SOLVER_H
