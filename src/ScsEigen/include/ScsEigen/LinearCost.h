/**
 * @file LinearCost.h
 * @author Giulio Romualdi
 * @copyright Released under the terms of the MIT License.
 * @date 2021
 */

#ifndef SCS_EIGEN_LINEAR_COST_H
#define SCS_EIGEN_LINEAR_COST_H

#include <string>

#include <Eigen/Dense>

#include <ScsEigen/Cost.h>

/**
 * ScsEigen namespace.
 */
namespace ScsEigen
{
/**
 * LinearCost is a concrete implementation of Cost. It implements a cost of the form
 * \f[
 *  f(x) = a ^\top x + b
 * \f]
 * where \f$x \in \mathbb{R}^n\f$ is the optimization variable, \f$a \in \mathbb{R}^n\f$ and \f$b \in \mathbb{R}\f$
 */
class LinearCost : public Cost
{
private:
    Eigen::VectorXd m_a;
    double m_b{0};

public:
    /**
     * @brief Constructor.
     * @note If you initialize the class with this constructor please remember to call the
     * LinearCost::setA() and LinearCost::setB() methods before calling Solver::solve()
     */
    LinearCost() = default;

    /**
     * @brief Constructor.
     * @param a the vector \f$a\f$.
     * @param b optional parameter. If not provided the is set to zero.
     * @note In case of error an invalid LinearCost will be build.
     */
    LinearCost(const Eigen::Ref<const Eigen::VectorXd>& a, double b = 0);

    /**
     * @brief Set the vector \f$a\f$.
     * @param a the vector \f$a\f$.
     * @return true/false in case of success/failure.
     */
    bool setA(const Eigen::Ref<const Eigen::VectorXd>& a);

    /**
     * @brief Set the scalar \f$b\f$.
     * @param b the scalar \f$b\f$.
     */
    void setB(double b);

    /**
     * @brief Get the vector \f$a\f$.
     */
    Eigen::Ref<const Eigen::VectorXd> getA() const;

    /**
     * @brief Get the scalar \f$b\f$.
     */
    double getB() const;
};

} // namespace ScsEigen

#endif // SCS_EIGEN_LINEAR_COST_H
