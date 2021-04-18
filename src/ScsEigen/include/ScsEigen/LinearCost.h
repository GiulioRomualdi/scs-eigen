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

class LinearCost : public Cost
{
private:
    Eigen::VectorXd m_a;
    double m_b{0};

public:
    LinearCost() = default;

    LinearCost(const Eigen::Ref<const Eigen::VectorXd>& a, double b = 0);

    bool setA(const Eigen::Ref<const Eigen::VectorXd>& a);

    void setB(double b);

    Eigen::Ref<const Eigen::VectorXd> getA() const;

    double getB() const;
};

} // namespace ScsEigen

#endif // SCS_EIGEN_LINEAR_COST_H
