/**
 * @file LinearCost.cpp
 * @author Giulio Romualdi
 * @copyright Released under the terms of the MIT License.
 * @date 2021
 */

#include <string>

#include <Eigen/Dense>

#include <ScsEigen/LinearCost.h>
#include <ScsEigen/Logger.h>

using namespace ScsEigen;

LinearCost::LinearCost(const Eigen::Ref<const Eigen::VectorXd>& a, double b)
    : Cost(a.size(), "Linear cost")
    , m_a(a)
    , m_b(b)
{
}

bool LinearCost::setA(const Eigen::Ref<const Eigen::VectorXd>& a)
{
    if (m_a.size() != 0)
    {
        if (a.size() != m_a.size())
        {
            log()->error("[LinearCost::setA] The size of the vector 'a' cannot change.");
            return false;
        }
    } else if (!this->setNumberOfVariables(a.size()))
    {
        log()->error("[LinearCost::setA] Unable to set the number of variables.");
        return false;
    }

    m_a = a;
    return true;
}

void LinearCost::setB(double b)
{
    m_b = b;
}

Eigen::Ref<const Eigen::VectorXd> LinearCost::getA() const
{
    return m_a;
}

double LinearCost::getB() const
{
    return m_b;
}
