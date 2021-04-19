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
    std::unordered_map<std::string_view, std::shared_ptr<LinearCost>> linearCosts;
};

MathematicalProgram::MathematicalProgram()
    : m_pimpl(std::make_unique<MathematicalProgram::Impl>())
{
}

MathematicalProgram::~MathematicalProgram() = default;

void MathematicalProgram::setNumberOfVariabels(unsigned int variables)
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

const std::unordered_map<std::string_view, std::shared_ptr<LinearCost>>&
MathematicalProgram::getLinearCosts() const
{
    return m_pimpl->linearCosts;
}
