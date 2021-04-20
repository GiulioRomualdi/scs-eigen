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
 * Constructs a cost of the form
 * @f[
 * \frac{1}{2} x ^\top Q x + b^\top x + c
 * @f]
 * @param Q Quadratic term.
 * @param b Linear term.
 * @param c (optional) Constant term.
 */
class QuadraticCost : public Cost
{
private:
    Eigen::MatrixXd m_Q;
    Eigen::VectorXd m_b;
    double m_c{0};

public:
    QuadraticCost() = default;

    QuadraticCost(const Eigen::Ref<const Eigen::MatrixXd>& Q,
                  const Eigen::Ref<const Eigen::MatrixXd>& b,
                  double c = 0);

    bool setQ(const Eigen::Ref<const Eigen::MatrixXd>& Q);

    bool setB(const Eigen::Ref<const Eigen::VectorXd>& b);

    void setC(double C);

    Eigen::Ref<const Eigen::MatrixXd> getQ() const;

    Eigen::Ref<const Eigen::VectorXd> getB() const;

    double getC() const;
};

} // namespace ScsEigen

#endif // SCS_EIGEN_QUADRATIC_COST_H
