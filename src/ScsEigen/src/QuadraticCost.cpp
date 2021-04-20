/**
 * @file QuadraticCost.cpp
 * @author Giulio Romualdi
 * @copyright Released under the terms of the MIT License.
 * @date 2021
 */

#include <string>

#include <Eigen/Dense>

#include <ScsEigen/Logger.h>
#include <ScsEigen/QuadraticCost.h>

using namespace ScsEigen;

QuadraticCost::QuadraticCost(const Eigen::Ref<const Eigen::MatrixXd>& Q,
                             const Eigen::Ref<const Eigen::MatrixXd>& b,
                             double c)
    : Cost((Q.rows() == Q.cols() && Q.rows() == b.rows()) ? Q.rows() : 0, "Quadratic cost")
{
    if (Q.rows() != Q.cols() || Q.rows() != b.rows())
    {

        log()->error("[QuadraticCost::QuadraticCost] Q matrix must be square and the size of b "
                     "should be coherent with Q");
        assert(false);
    } else
    {
        m_Q = (Q + Q.transpose()) / 2;
        m_b = b;
        m_c = c;
    }
}

bool QuadraticCost::setQ(const Eigen::Ref<const Eigen::MatrixXd>& Q)
{
    if (m_Q.size() != 0)
    {
        if (Q.size() != m_Q.size())
        {
            log()->error("[QuadraticCost::setQ] The size of the matrix 'Q' cannot change.");
            return false;
        }
    } else if (Q.rows() != Q.cols())
    {
        log()->error("[QuadraticCost::QuadraticCost] Q matrix must be square.");
        return false;
    } else if (!this->setNumberOfVariables(Q.rows()))
    {
        log()->error("[QuadraticCost::setQ] Unable to set the number of variables.");
        return false;
    }

    m_Q = (Q + Q.transpose()) / 2;
    return true;
}

bool QuadraticCost::setB(const Eigen::Ref<const Eigen::VectorXd>& b)
{
    if (m_b.size() != 0)
    {
        if (b.size() != m_b.size())
        {
            log()->error("[QuadraticCost::setB] The size of the vector 'b' cannot change.");
            return false;
        }
    } else if (!this->setNumberOfVariables(b.size()))
    {
        log()->error("[QuadraticCost::setB] Unable to set the number of variables.");
        return false;
    }

    m_b = b;
    return true;
}

void QuadraticCost::setC(double c)
{
    m_c = c;
}

Eigen::Ref<const Eigen::VectorXd> QuadraticCost::getB() const
{
    return m_b;
}

Eigen::Ref<const Eigen::MatrixXd> QuadraticCost::getQ() const
{
    return m_Q;
}

double QuadraticCost::getC() const
{
    return m_c;
}
