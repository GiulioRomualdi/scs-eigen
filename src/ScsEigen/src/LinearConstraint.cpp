/**
 * @file LinearConstraint.cpp
 * @author Giulio Romualdi
 * @copyright Released under the terms of the MIT License.
 * @date 2021
 */

#include <optional>
#include <string>

#include <Eigen/Dense>

#include <ScsEigen/LinearConstraint.h>
#include <ScsEigen/Logger.h>

using namespace ScsEigen;

LinearConstraint::LinearConstraint(const Eigen::Ref<const Eigen::MatrixXd>& A,
                                   const Eigen::Ref<const Eigen::VectorXd>& lowerBound,
                                   const Eigen::Ref<const Eigen::VectorXd>& upperBound,
                                   std::optional<Eigen::Ref<const SparsityPattern>> sparsityPattern)
    : Constraint(A.cols(), "Linear constraint")
{
    constexpr double tolerance = 1e-10;

    if (!this->setBounds(lowerBound, upperBound))
    {
        log()->error("[LinearConstraint::LinearConstraint] Unable to set the bounds.");
        assert(false);
    }

    if (A.rows() != this->getNumberOfConstraints())
    {
        log()->error("[LinearConstraint::LinearConstraint] Size mismatch for the constraint "
                     "matrix.");
        assert(false);
    } else
    {

        m_A = A;
    }

    if (!sparsityPattern.has_value())
        m_ASparsityPattern = (m_A.array().abs() > tolerance);
    else if (sparsityPattern->size() != m_A.size())
    {
        log()->error("[LinearConstraint::LinearConstraint] Size mismatch for the sparsity "
                     "pattern.");
        assert(false);
    } else
    {
        m_ASparsityPattern = sparsityPattern.value();
    }
}

bool LinearConstraint::setA(const Eigen::Ref<const Eigen::MatrixXd>& A,
                            std::optional<Eigen::Ref<const SparsityPattern>> sparsityPattern)
{
    if (m_A.size() != 0)
    {
        if (A.size() != m_A.size())
        {
            log()->error("[LinearConstraint::setA] The size of the matrix 'A' cannot change.");
            return false;
        }
    } else if (!this->setNumberOfVariables(m_A.cols()))
    {
        log()->error("[LinearConstraint::setA] Unable to set the number of variables.");
        return false;
    } else if (!this->setNumberOfConstraints(m_A.rows()))
    {
        log()->error("[LinearConstraint::setA] Unable to set the number of constraints.");
        return false;
    }

    m_A = A;

    // check if sparsity pattern has been already set
    //
    if (m_ASparsityPattern.size() == 0)
    {
        if (!sparsityPattern.has_value())
        {
            constexpr double tolerance = 1e-10;
            m_ASparsityPattern = (m_A.array().abs() > tolerance);
        } else if (sparsityPattern->size() != m_A.size())
        {
            log()->error("[LinearConstraint::setA] Size mismatch for the sparsity pattern.");
            assert(false);
            return false;
        } else
        {
            m_ASparsityPattern = sparsityPattern.value();
        }
    } else
    {
        log()->debug("[LinearConstraint::setA] The sparsity pattern is set only once.");
    }
    return true;
}

Eigen::Ref<const Eigen::MatrixXd> LinearConstraint::getA() const
{
    return m_A;
}

Eigen::Ref<const LinearConstraint::SparsityPattern> LinearConstraint::getASparsityPattern() const
{
    return m_ASparsityPattern;
}
