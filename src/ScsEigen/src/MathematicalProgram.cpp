/**
 * @file MathematicalProgram.cpp
 * @author Giulio Romualdi
 * @copyright Released under the terms of the MIT License.
 * @date 2021
 */

#include <string>
#include <unordered_map>

#include <ScsEigen/Logger.h>
#include <ScsEigen/MathematicalProgram.h>

using namespace ScsEigen;

struct MathematicalProgram::Impl
{
    unsigned int numberOfVariables{0};
    MathematicalProgram::dictionary<LinearCost> linearCosts;
    MathematicalProgram::dictionary<QuadraticCost> quadraticCosts;

    MathematicalProgram::dictionary<LinearConstraint> linearConstraints;
};

MathematicalProgram::MathematicalProgram()
    : m_pimpl(std::make_unique<MathematicalProgram::Impl>())
{
}

MathematicalProgram::~MathematicalProgram() = default;

void MathematicalProgram::setNumberOfVariables(unsigned int variables)
{
    m_pimpl->numberOfVariables = variables;
}

unsigned int MathematicalProgram::numberOfVariables() const
{
    return m_pimpl->numberOfVariables;
}

bool MathematicalProgram::addLinearCost(std::shared_ptr<LinearCost> cost, std::string_view name)
{
    if (m_pimpl->numberOfVariables == 0)
    {
        log()->error("[MathematicalProgram::addLinearCost] Please set the number of variables.");
        return false;
    }

    if (m_pimpl->linearCosts.find(name) != m_pimpl->linearCosts.end())
    {
        log()->error("[MathematicalProgram::addLinearCost] The cost named " + std::string(name)
                     + "already exists.");
        return false;
    }

    if (cost == nullptr)
    {
        log()->error("[MathematicalProgram::addLinearCost] The cost is not valid.");
        return false;
    }

    if (cost->getNumberOfVariables() != m_pimpl->numberOfVariables)
    {
        log()->error("[MathematicalProgram::addLinearCost] The size of the cost is different from "
                     "the number of variables stored in the MathematicalProgram. Expected:"
                     + std::to_string(m_pimpl->numberOfVariables)
                     + " passed:" + std::to_string(cost->getNumberOfVariables()) + ".");
        return false;
    }

    m_pimpl->linearCosts[name] = cost;
    return true;
}

std::weak_ptr<LinearCost> MathematicalProgram::getLinearCost(std::string_view name) const
{
    auto cost = m_pimpl->linearCosts.find(name);
    if (cost != m_pimpl->linearCosts.end())
    {
        return cost->second;
    }

    log()->warning("[MathematicalProgram::getLinearCost] The cost named " + std::string(name)
                   + "does not exists. An invalid weak_ptr will be returned.");

    return std::shared_ptr<LinearCost>();
}

const MathematicalProgram::dictionary<LinearCost>& MathematicalProgram::getLinearCosts() const
{
    return m_pimpl->linearCosts;
}

bool MathematicalProgram::addQuadraticCost(std::shared_ptr<QuadraticCost> cost,
                                           std::string_view name)
{
    if (m_pimpl->numberOfVariables == 0)
    {
        log()->error("[MathematicalProgram::addQuadraticCost] Please set the number of variables.");
        return false;
    }

    if (m_pimpl->quadraticCosts.find(name) != m_pimpl->quadraticCosts.end())
    {
        log()->error("[MathematicalProgram::addQuadraticCost] The cost named " + std::string(name)
                     + "already exists.");
        return false;
    }

    if (cost == nullptr)
    {
        log()->error("[MathematicalProgram::addQuadraticCost] The cost is not valid.");
        return false;
    }

    if (cost->getNumberOfVariables() != m_pimpl->numberOfVariables)
    {
        log()->error("[MathematicalProgram::addQuadraticCost] The size of the cost is different "
                     "from the number of variables stored in the MathematicalProgram. Expected:"
                     + std::to_string(m_pimpl->numberOfVariables)
                     + " passed:" + std::to_string(cost->getNumberOfVariables()) + ".");
        return false;
    }

    m_pimpl->quadraticCosts[name] = cost;
    return true;
}

std::weak_ptr<QuadraticCost> MathematicalProgram::getQuadraticCost(std::string_view name) const
{
    auto cost = m_pimpl->quadraticCosts.find(name);
    if (cost != m_pimpl->quadraticCosts.end())
    {
        return cost->second;
    }

    log()->warning("[MathematicalProgram::getQuadraticCost] The cost named " + std::string(name)
                   + "does not exists. An invalid weak_ptr will be returned.");

    return std::shared_ptr<QuadraticCost>();
}

const MathematicalProgram::dictionary<QuadraticCost>& MathematicalProgram::getQuadraticCosts() const
{
    return m_pimpl->quadraticCosts;
}

bool MathematicalProgram::addLinearConstraint(std::shared_ptr<LinearConstraint> constraint,
                                              std::string_view name)
{
    if (m_pimpl->numberOfVariables == 0)
    {
        log()->error("[MathematicalProgram::addLinearConstraint] Please set the number of "
                     "variables.");
        return false;
    }

    if (m_pimpl->linearConstraints.find(name) != m_pimpl->linearConstraints.end())
    {
        log()->error("[MathematicalProgram::addLinearConstraint] The constraint named "
                     + std::string(name) + "already exists.");
        return false;
    }

    if (constraint == nullptr)
    {
        log()->error("[MathematicalProgram::addLinearConstraint] The constraint is not valid.");
        return false;
    }

    if (constraint->getNumberOfVariables() != m_pimpl->numberOfVariables)
    {
        log()->error("[MathematicalProgram::addLinearConstraint] The size of the constraint is "
                     "different from the number of variables stored in the MathematicalProgram. "
                     "Expected:"
                     + std::to_string(m_pimpl->numberOfVariables)
                     + " passed:" + std::to_string(constraint->getNumberOfVariables()) + ".");
        return false;
    }

    m_pimpl->linearConstraints[name] = constraint;
    return true;
}

std::weak_ptr<LinearConstraint>
MathematicalProgram::getLinearConstraint(std::string_view name) const
{
    auto constraint = m_pimpl->linearConstraints.find(name);
    if (constraint != m_pimpl->linearConstraints.end())
    {
        return constraint->second;
    }

    log()->warning("[MathematicalProgram::getLinearConstraint] The constraint named "
                   + std::string(name) + "does not exists. An invalid weak_ptr will be returned.");

    return std::shared_ptr<LinearConstraint>();
}

const MathematicalProgram::dictionary<LinearConstraint>&
MathematicalProgram::getLinearConstraints() const
{
    return m_pimpl->linearConstraints;
}
