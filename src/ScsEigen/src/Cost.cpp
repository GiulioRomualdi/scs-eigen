/**
 * @file Cost.cpp
 * @author Giulio Romualdi
 * @copyright Released under the terms of the MIT License.
 * @date 2021
 */


#include <string>

#include <Eigen/Dense>

#include <ScsEigen/Cost.h>
#include <ScsEigen/Logger.h>

using namespace ScsEigen;

Cost::Cost(int numberOfVariables, std::string_view description)
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
    }

    m_numberOfVariables = numberOfVariables;
}

void Cost::setDescription(std::string_view description)
{
    m_description = description;
}

bool Cost::setNumberOfVariables(int numberOfVariables)
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

int Cost::getNumberOfVariables() const
{
    return m_numberOfVariables;
}

std::string_view Cost::getDescrition() const
{
    return m_description;
}
