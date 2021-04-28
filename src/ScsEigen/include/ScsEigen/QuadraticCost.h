/**
 * @file QuadraticCost.h
 * @author Giulio Romualdi
 * @copyright Released under the terms of the MIT License.
 * @date 2021
 */

#ifndef SCS_EIGEN_QUADRATIC_COST_H
#define SCS_EIGEN_QUADRATIC_COST_H

#include <string>

#include <Eigen/Dense>

#include <ScsEigen/Cost.h>

/**
 * ScsEigen namespace.
 */
namespace ScsEigen
{

/**
 * QuadraticCost is a concrete implementation of Cost. It implements a cost of the form
 * \f[
 *  f(x) = \frac{1}{2} x ^\top Q x + b^\top x + c
 * \f]
 * where \f$x \in \mathbb{R}^n\f$ is the optimization variable. \f$Q \in S^n_+\f$  is a semipositive
 * definite matrix, \f$b \in \mathbb{R}^n\f$ and the scalar \f$c \in \mathbb{R}\f$
 */
class QuadraticCost : public Cost
{
private:
    Eigen::MatrixXd m_Q;
    Eigen::VectorXd m_b;
    double m_c{0};

public:
    /**
     * @brief Constructor.
     * @note If you initialize the class with this constructor please remember to call the
     * QuadraticCost::setQ(), QuadraticCost::setB() and QuadraticCost::setC() methods before calling
     * Solver::solve()
     */
    QuadraticCost() = default;

    /**
     * @brief Constructor.
     * @param Q the PSD matrix representing the hessian of the cost function.
     * @param b the gradient of the cost function.
     * @param c optional parameter. If not provided the is set to zero.
     * @note In case of error an invalid QuadraticCost will be build.
     */
    QuadraticCost(const Eigen::Ref<const Eigen::MatrixXd>& Q,
                  const Eigen::Ref<const Eigen::MatrixXd>& b,
                  double c = 0);

    /**
     * @brief Set the hessian matrix Q.
     * @param Q the PSD matrix representing the hessian of the cost function.
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
     * @brief Set the scalar term.
     * @param c the scalar term
     */
    void setC(double c);

    /**
     * @brief Get the hessian \f$Q\f$.
     */
    Eigen::Ref<const Eigen::MatrixXd> getQ() const;

    /**
     * @brief Get the vector \f$b\f$.
     */
    Eigen::Ref<const Eigen::VectorXd> getB() const;

    /**
     * @brief Get the scalar \f$c\f$.
     */
    double getC() const;
};

} // namespace ScsEigen

#endif // SCS_EIGEN_QUADRATIC_COST_H
