/**
 * @file Math.cpp
 * @author Giulio Romualdi
 * @copyright Released under the terms of the MIT License.
 * @date 2021
 */

#include <utility>

#include <Eigen/Cholesky>
#include <Eigen/Dense>

#include <ScsEigen/Logger.h>
#include <ScsEigen/Math.h>

std::pair<bool, Eigen::MatrixXd>
ScsEigen::choleskyDecomposition(const Eigen::Ref<const Eigen::MatrixXd>& A)
{
    if (A.rows() != A.cols())
    {
        log()->error("[ScsEigen::choleskyDecomposition] A is not square.");
        assert(false);
        return std::pair(false, Eigen::MatrixXd());
    }

    // In most case the cholesky decomposition should be fine.
    // if eigen is not able to find a feasible solution we try to use a robust cholesky
    // decomposition
    Eigen::LLT<Eigen::MatrixXd> lltA(A);
    if (lltA.info() == Eigen::Success)
    {
        return std::pair(true, lltA.matrixU());
    } else
    {
        Eigen::LDLT<Eigen::MatrixXd> ldltA(A);
        if (ldltA.info() == Eigen::Success)
        {
            return std::pair(true, ldltA.matrixU());
        }
    }

    log()->error("[ScsEigen::choleskyDecomposition] Unable to compute the Cholesky decomposition.");
    return std::pair(false, Eigen::MatrixXd());
}
