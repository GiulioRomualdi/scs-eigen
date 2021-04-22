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


    int iteration{0}; /**< Number of iterations taken by the solver */
    double primalObjective{std::nan("0")}; /**< Primal objective value */
    double dualObjective{std::nan("0")}; /**< Dual objective value */
    double primalResidue{std::nan("0")}; /**< Primal equality residue */
    double residueInfeasibility{std::nan("0")}; /**< infeasibility certificate residue. */
    double residueUnbounded{std::nan("0")}; /**< unbounded certificate residue */
    double relativeDualityGap{std::nan("0")}; /**< relative duality gap */
    double setupTime{std::nan("0")}; /**< Time taken for SCS to setup in milliseconds. */
    double solveTime{std::nan("0")}; /**< Time taken for SCS to solve the problem in milliseconds. */

    Eigen::VectorXd solution;
    Eigen::VectorXd completeSolution;
    Eigen::VectorXd dualVariable;
    Eigen::VectorXd primalEqualitySlack;

    Status status{Status::unknown};

    bool isValid() const;

private:
    friend Solver;
    static Solution::Status getStatus(int i);
};

} // namespace ScsEigen

#endif // SCS_EIGEN_SOLUTION_H
