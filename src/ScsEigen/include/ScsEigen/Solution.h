/**
 * @file Solution.h
 * @author Giulio Romualdi
 * @copyright  Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */
#ifndef SCS_EIGEN_SOLUTION_H
#define SCS_EIGEN_SOLUTION_H

// Std
#include <cmath>

#include <Eigen/Dense>

/**
 * ScsEigen namespace.
 */
namespace ScsEigen
{

class Solver;

/**
 * Solution class contains the solution of the convex problem.
 */
struct Solution
{
    /**
     * @brief Status of scs solver.
     * @note You can find further information
     * [here](https://github.com/cvxgrp/scs/blob/07ca69c296312c260027c755f545f05bf45156eb/include/glbopts.h#L18-L28).
     */
    enum class Status
    {
        infeasible_inaccurate,
        unbounded_inaccurate,
        sigint,
        failed,
        indeterminate,
        infeasible,
        unbounded,
        unfinished,
        solved,
        solved_inaccurate,
        unknown,
    };

    int iteration{0}; /**<@brief Number of iterations taken by the solver */
    double primalObjective{std::nan("0.0")}; /**<@brief Primal objective value */
    double dualObjective{std::nan("0.0")}; /**<@brief Dual objective value */
    double primalResidue{std::nan("0.0")}; /**<@brief Primal equality residue */
    double residueInfeasibility{std::nan("0.0")}; /**<@brief infeasibility certificate residue. */
    double residueUnbounded{std::nan("0.0")}; /**<@brief unbounded certificate residue */
    double relativeDualityGap{std::nan("0.0")}; /**<@brief relative duality gap */
    double setupTime{std::nan("0.0")}; /**<@brief Time taken for SCS to setup in milliseconds. */
    double solveTime{std::nan("0.0")}; /**<@brief Time taken for SCS to solve the problem in
                                          milliseconds. */

    Eigen::VectorXd solution; /**<@brief Solution of the problem stored in MathematicalProgram. */
    Eigen::VectorXd completeSolution; /**<@brief Complete Solution of the problem solved by SCS. It
                                         contains also the slack variables. */
    Eigen::VectorXd dualVariable; /**<@brief Dual solution. */
    Eigen::VectorXd primalEqualitySlack; /**<@brief Equality slack variables. */

    Status status{Status::unknown}; /**<@brief Status of the Solution.. */

    /**
     * @brief Check if the solution is valid
     */
    bool isValid() const;

private:
    friend Solver;
    static Solution::Status getStatus(int i);
};

} // namespace ScsEigen

#endif // SCS_EIGEN_SOLUTION_H
