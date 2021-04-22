/**
 * @file Solution.cpp
 * @author Giulio Romualdi
 * @copyright Released under the terms of the MIT License.
 * @date 2021
 */

#include <scs.h>
#include <util.h>
#include <glbopts.h>

#include <ScsEigen/Solution.h>

#include <Eigen/Sparse>

using namespace ScsEigen;

Solution::Status Solution::getStatus(int i)
{
    switch (i)
    {
    case SCS_INFEASIBLE_INACCURATE:
        return Status::infeasible_inaccurate;
    case SCS_UNBOUNDED_INACCURATE:
        return Status::unbounded_inaccurate;
    case SCS_SIGINT:
        return Status::sigint;
    case SCS_FAILED:
        return Status::failed;
    case SCS_INDETERMINATE:
        return Status::indeterminate;
    case SCS_INFEASIBLE:
        return Status::infeasible;
    case SCS_UNBOUNDED:
        return Status::unbounded;
    case SCS_UNFINISHED:
        return Status::unfinished;
    case SCS_SOLVED:
        return Status::solved;
    case SCS_SOLVED_INACCURATE:
        return Status::solved_inaccurate;
    default:
        return Status::unknown;
    }
}

bool Solution::isValid() const
{
    // solution and complete solution can be empty only if solution Solution::Status is different
    // from "solved" and "solved_inaccurate"

    const bool isSolutionOk = ((status == Status::solved || status == Status::solved_inaccurate)
                               && (solution.size() != 0 && completeSolution.size() != 0))
                              || (status != Status::solved && status != Status::solved_inaccurate);

    // residueUnbounded and residueInfeasibility can be nan. Please check
    // https://github.com/cvxgrp/scs/blob/48dfbe81caad2162c3ce5757faccdb3f3d31e142/src/scs.c#L226-L229

    return (status != Status::unknown) && !std::isnan(primalObjective) && !std::isnan(dualObjective)
           && !std::isnan(primalResidue) && !std::isnan(relativeDualityGap)
           && !std::isnan(setupTime) && !std::isnan(solveTime) && (dualVariable.size() != 0)
           && (primalEqualitySlack.size() != 0) && isSolutionOk;
}
