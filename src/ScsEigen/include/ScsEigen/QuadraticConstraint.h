/**
 * @file QuadraticConstraint.h
 * @author Giulio Romualdi
 * @copyright Released under the terms of the MIT License.
 * @date 2021
 */

#ifndef SCS_EIGEN_QUADRATIC_CONSTRAINT_H
#define SCS_EIGEN_QUADRATIC_CONSTRAINT_H

#include <string>

#include <Eigen/Dense>

#include <ScsEigen/Constraint.h>

/**
 * ScsEigen namespace.
 */
namespace ScsEigen
{

/**
 * QuadraticConstraint is a concrete implementation of Constraint. It implements a constraint of the form
 * \f[
 * \frac{1}{2} x ^\top Q x + b^\top x   \le u
 * \f]
 * where \f$x \in \mathbb{R}^n\f$ is the optimization variable. \f$Q \in S^n_+\f$  is a semipositive
 * definite matrix.
 */
class QuadraticConstraint : public Constraint
{
private:
    Eigen::MatrixXd m_Q;
    Eigen::VectorXd m_b;

public:
    /**
     * @brief Constructor.
     * @note If you initialize the class with this constructor please remember to call the
     * QuadraticConstraint::setQ(), QuadraticConstraint::setB() QuadraticConstraint::setBounds()
     * methods before calling Solver::solve()
     */
    QuadraticConstraint();

    /**
     * @brief Constructor.
     * @param Q the PSD matrix representing the hessian of the constraint function.
     * @param b the gradient of the constraint function.
     * @param lowerBound vector containing the lower bound.
     * @param upperBound vector containing the upper bound.
     * @note In case of error an invalid QuadraticConstraint will be build.
     */
    QuadraticConstraint(const Eigen::Ref<const Eigen::MatrixXd>& Q,
                        const Eigen::Ref<const Eigen::MatrixXd>& b,
                        double upperBound);

    /**
     * @brief Set the hessian matrix Q.
     * @param Q the PSD matrix representing the hessian of the constraint function.
     * @note True/false in case of success/failure.
     */
    bool setQ(const Eigen::Ref<const Eigen::MatrixXd>& Q);

    /**
     * @brief Set the gradient vector.
     * @param b the vector
     * @note True/false in case of success/failure.
     */
    bool setB(const Eigen::Ref<const Eigen::VectorXd>& b);

    /**
     * @brief Set the lower bound.
     * @param lowerBound vector representing the lower bound.
     * @return true in case of success/false otherwise
     * @warning In case of QuadraticConstraint the only admissible lower bound is
     * - std::numeric_limits<double>::infinity()
     */
    bool setLowerBound(const Eigen::Ref<const Eigen::VectorXd>& lowerBound) final;

    /**
     * @brief Get the hessian \f$Q\f$.
     */
    Eigen::Ref<const Eigen::MatrixXd> getQ() const;

    /**
     * @brief Get the vector \f$b\f$.
     */
    Eigen::Ref<const Eigen::VectorXd> getB() const;
};

} // namespace ScsEigen

#endif // SCS_EIGEN_QUADRATIC_CONSTRAINT_H
