/**
 * @file QuadraticConstraint.cpp
 * @author Giulio Romualdi
 * @copyright Released under the terms of the MIT License.
 * @date 2021
 */

#include <cmath>
#include <limits>
#include <string>

#include <Eigen/Dense>

#include <ScsEigen/Logger.h>
#include <ScsEigen/Math.h>
#include <ScsEigen/QuadraticConstraint.h>

using namespace ScsEigen;

QuadraticConstraint::QuadraticConstraint()
{
    this->m_lowerBound = ScsEigen::Vector1d::Constant(
        -std::numeric_limits<ScsEigen::Vector1d::Scalar>::infinity());
}

QuadraticConstraint::QuadraticConstraint(const Eigen::Ref<const Eigen::MatrixXd>& Q,
                                         const Eigen::Ref<const Eigen::MatrixXd>& b,
                                         double upperBound)
    : Constraint((Q.rows() == Q.cols() && Q.rows() == b.rows()) ? Q.rows() : 0,
                 "Quadratic constraint")
{
    if (Q.rows() != Q.cols() || Q.rows() != b.rows())
    {

        log()->error("[QuadraticConstraint::QuadraticConstraint] Q matrix must be square and the "
                     "size of b "
                     "should be coherent with Q");
        assert(false);
    } else
    {
        m_Q = (Q + Q.transpose()) / 2;
        m_b = b;
    }

    // the only admissible lower bound is
    // -std::numeric_limits<ScsEigen::Vector1d::Scalar>::infinity()
    this->m_lowerBound = ScsEigen::Vector1d::Constant(
        -std::numeric_limits<ScsEigen::Vector1d::Scalar>::infinity());

    // set the upperbound
    this->setUpperBound(ScsEigen::Vector1d::Constant(upperBound));
}

bool QuadraticConstraint::setQ(const Eigen::Ref<const Eigen::MatrixXd>& Q)
{
    if (m_Q.size() != 0)
    {
        if (Q.size() != m_Q.size())
        {
            log()->error("[QuadraticConstraint::setQ] The size of the matrix 'Q' cannot change.");
            return false;
        }
    } else if (Q.rows() != Q.cols())
    {
        log()->error("[QuadraticConstraint::QuadraticConstraint] Q matrix must be square.");
        return false;
    } else if (!this->setNumberOfVariables(Q.rows()))
    {
        log()->error("[QuadraticConstraint::setQ] Unable to set the number of variables.");
        return false;
    }

    m_Q = (Q + Q.transpose()) / 2;
    return true;
}

bool QuadraticConstraint::setB(const Eigen::Ref<const Eigen::VectorXd>& b)
{
    if (m_b.size() != 0)
    {
        if (b.size() != m_b.size())
        {
            log()->error("[QuadraticConstraint::setB] The size of the vector 'b' cannot change.");
            return false;
        }
    } else if (!this->setNumberOfVariables(b.size()))
    {
        log()->error("[QuadraticConstraint::setB] Unable to set the number of variables.");
        return false;
    }

    m_b = b;
    return true;
}

bool QuadraticConstraint::setLowerBound(const Eigen::Ref<const Eigen::VectorXd>& lowerBound)
{
    if (lowerBound.size() != 1)
    {
        log()->error("[QuadraticConstraint::setLowerBound] The lower bound should be a scalar.");
        return false;
    }

    if (lowerBound[0] > 0 || !std::isinf(lowerBound[0]))
    {
        log()->error("[QuadraticConstraint::setLowerBound] The only admissible lowerBound is "
                     "-std::numeric_limits<ScsEigen::Vector1d::Scalar>::infinity()");
        return false;
    }

    // the only admissible lowerBound is
    // -std::numeric_limits<ScsEigen::Vector1d::Scalar>::infinity() and it has been already set in
    // the constructor
    return true;
}

Eigen::Ref<const Eigen::VectorXd> QuadraticConstraint::getB() const
{
    return m_b;
}

Eigen::Ref<const Eigen::MatrixXd> QuadraticConstraint::getQ() const
{
    return m_Q;
}
