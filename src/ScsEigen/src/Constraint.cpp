/**
 * @file Constraint.cpp
 * @author Giulio Romualdi
 * @copyright Released under the terms of the MIT License.
 * @date 2021
 */

#include <string>

#include <Eigen/Dense>

#include <ScsEigen/Constraint.h>
#include <ScsEigen/Logger.h>

using namespace ScsEigen;

Constraint::Constraint(int numberOfVariables, std::string_view description)
{
    if (description == "")
    {
        m_description = description;
    }

    if (numberOfVariables <= 0)
    {
        log()->error("The number of variables should be a positive number. The default value will  "
                     "be used: "
                     + std::to_string(m_numberOfVariables) + ".");
    } else
    {
        m_numberOfVariables = numberOfVariables;
    }
}

void Constraint::setDescription(std::string_view description)
{
    m_description = description;
}

bool Constraint::setNumberOfConstraints(int numberOfConstraints)
{
    if (numberOfConstraints < 0)
    {
        log()->error("The number of constraints should be a positive number. The default value will  "
                     "be used: "
                     + std::to_string(m_numberOfConstraints) + ".");
        return false;
    } else if (m_numberOfConstraints != 0 && m_numberOfConstraints != numberOfConstraints)
    {
        log()->error("The number of constraints has been already set and it is different from the "
                     "expected number of constraints.");
        return false;
    } else
    {
        m_numberOfConstraints = numberOfConstraints;
    }
    return true;
}

bool Constraint::setNumberOfVariables(int numberOfVariables)
{
    if (numberOfVariables < 0)
    {
        log()->error("The number of variables should be a positive number. The default value will  "
                     "be used: "
                     + std::to_string(m_numberOfVariables) + ".");
        return false;
    } else if (m_numberOfVariables != 0 && m_numberOfVariables != numberOfVariables)
    {
        log()->error("The number of variables has been already set and it is different from the "
                     "expected number of variables.");
        return false;
    } else
    {
        m_numberOfVariables = numberOfVariables;
    }
    return true;
}


bool Constraint::setLowerBound(const Eigen::Ref<const Eigen::VectorXd>& lowerBound)
{
    if (m_numberOfConstraints == 0)
    {
        m_numberOfConstraints = lowerBound.size();
        m_lowerBound = lowerBound;
        return true;
    } else if (m_numberOfConstraints != lowerBound.size())
    {
        log()->error("[Constraint::setLowerBound] The size of the lower bound is different "
                     "from expected. Lower bound size: "
                     + std::to_string(lowerBound.size())
                     + ", expected: " + std::to_string(m_numberOfConstraints) + ".");
        return false;
    }
    m_lowerBound = lowerBound;

    return true;
}

bool Constraint::setUpperBound(const Eigen::Ref<const Eigen::VectorXd>& upperBound)
{
    if (m_numberOfConstraints == 0)
    {
        m_numberOfConstraints = upperBound.size();
        m_upperBound = upperBound;
        return true;
    } else if (m_numberOfConstraints != upperBound.size())
    {
        log()->error("[Constraint::setUpperBound] The size of the upper bound is different "
                     "from expected. Upper bound size: "
                     + std::to_string(upperBound.size())
                     + ", expected: " + std::to_string(m_numberOfConstraints) + ".");
        return false;
    }
    m_upperBound = upperBound;

    return true;
}

bool Constraint::setBounds(const Eigen::Ref<const Eigen::VectorXd>& lowerBound,
                           const Eigen::Ref<const Eigen::VectorXd>& upperBound)
{
    bool ok = true;
    ok = ok && this->setLowerBound(lowerBound);
    ok = ok && this->setUpperBound(upperBound);
    return ok;
}



int Constraint::getNumberOfVariables() const
{
    return m_numberOfVariables;
}

int Constraint::getNumberOfConstraints() const
{
    return m_numberOfConstraints;
}

Eigen::Ref<const Eigen::VectorXd> Constraint::getLowerBound() const
{
    return m_lowerBound;
}

Eigen::Ref<const Eigen::VectorXd> Constraint::getUpperBound() const
{
    return m_upperBound;
}

std::string_view Constraint::getDescription() const
{
    return m_description;
}
