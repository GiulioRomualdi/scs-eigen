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

/**
 * LinearConstraint is a concrete implementation of Constraint. It implements a constraint of the
 * form.
 * \f[
 *  l \le A x  \le u
 * \f]
 * where \f$x \in \mathbb{R}^n\f$ is the optimization variable. The linear constraints are defined
 * by matrix \f$A \in \mathbb{R}^{m \times n}\f$ and vectors \f$l\f$ and \f$u\f$ so that \f$l_i \in
 * \mathbb{R} \cup \{-\infty\}\f$ and \f$u_i \in \mathbb{R} \cup \{\infty\}\f$ for all \f$i \in
 * \{1,...,m\}\f$.
 */
class LinearConstraint : public Constraint
{
public:
    using SparsityPattern = Eigen::Array<bool, Eigen::Dynamic, Eigen::Dynamic>;

private:
    Eigen::MatrixXd m_A;
    SparsityPattern m_ASparsityPattern;

public:
    /**
     * @brief Constructor.
     * @note If you initialize the class with this constructor please remember to call the
     * LinearConstraint::setA() and LinearConstraint::setBounds() methods before calling
     * Solver::solve()
     */
    LinearConstraint() = default;

    /**
     * @brief Constructor.
     * @param A linear constraint matrix.
     * @param lowerBound vector containing the lower bound.
     * @param upperBound vector containing the upper bound.
     * @param sparsityPattern optional parameter containing the sparsity pattern of the matrix A. If
     * not provided the sparsity pattern is automatically computed.
     * @note In case of error an invalid LinearConstraint will be build.
     */
    LinearConstraint(const Eigen::Ref<const Eigen::MatrixXd>& A,
                     const Eigen::Ref<const Eigen::VectorXd>& lowerBound,
                     const Eigen::Ref<const Eigen::VectorXd>& upperBound,
                     std::optional<Eigen::Ref<const SparsityPattern>> sparsityPattern = {});

    /**
     * @brief Set the constraint matrix.
     * @param A linear constraint matrix.
     * @param sparsityPattern optional parameter containing the sparsity pattern of the matrix A. If
     * not provided the sparsity pattern is automatically computed.
     * @return true/false in case of success/failure.
     */
    bool setA(const Eigen::Ref<const Eigen::MatrixXd>& A,
              std::optional<Eigen::Ref<const SparsityPattern>> sparsityPattern = {});

    /**
     * @brief Get the constraint matrix.
     * @return the constraint matrix A.
     */
    Eigen::Ref<const Eigen::MatrixXd> getA() const;

    /**
     * @brief Get the sparsity pattern of the constraint matrix A,
     * @return the sparsity pattern of A.
     */
    Eigen::Ref<const SparsityPattern> getASparsityPattern() const;
};

} // namespace ScsEigen

#endif // SCS_EIGEN_LINEAR_CONSTRAINT_H
