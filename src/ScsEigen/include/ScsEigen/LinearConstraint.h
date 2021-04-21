/**
 * @file LinearConstraint.h
 * @author Giulio Romualdi
 * @copyright Released under the terms of the MIT License.
 * @date 2021
 */

#ifndef SCS_EIGEN_LINEAR_CONSTRAINT_H
#define SCS_EIGEN_LINEAR_CONSTRAINT_H

#include <string>
#include <optional>

#include <Eigen/Dense>

#include <ScsEigen/Constraint.h>

/**
 * ScsEigen namespace.
 */
namespace ScsEigen
{

class LinearConstraint : public Constraint
{
public:
    using SparsityPattern = Eigen::Array<bool, Eigen::Dynamic, Eigen::Dynamic>;

private:
    Eigen::MatrixXd m_A;
    SparsityPattern m_ASparsityPattern;

public:
    LinearConstraint() = default;

    LinearConstraint(const Eigen::Ref<const Eigen::MatrixXd>& A,
                     const Eigen::Ref<const Eigen::VectorXd>& lowerBound,
                     const Eigen::Ref<const Eigen::VectorXd>& upperBound,
                     std::optional<Eigen::Ref<const SparsityPattern>> sparsityPattern = {});

    bool setA(const Eigen::Ref<const Eigen::MatrixXd>& A,
              std::optional<Eigen::Ref<const SparsityPattern>> sparsityPattern = {});

    Eigen::Ref<const Eigen::MatrixXd> getA() const;

    Eigen::Ref<const SparsityPattern> getASparsityPattern() const;
};

} // namespace ScsEigen

#endif // SCS_EIGEN_LINEAR_CONSTRAINT_H
