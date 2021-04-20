/**
 * @file Math.h
 * @author Giulio Romualdi
 * @copyright Released under the terms of the MIT License.
 * @date 2021
 */

#ifndef SCS_EIGEN_LINEAR_MATH_H
#define SCS_EIGEN_LINEAR_MATH_H

#include <utility>

#include <Eigen/Dense>

/**
 * ScsEigen namespace.
 */
namespace ScsEigen
{

/**
 * Compute the Cholesky decomposition of a PSD matrix A.
 * @note please be sure the matrix A is PSD.
 */
std::pair<bool, Eigen::MatrixXd> choleskyDecomposition(const Eigen::Ref<const Eigen::MatrixXd>& A);
}

#endif // SCS_EIGEN_LINEAR_MATH_H
