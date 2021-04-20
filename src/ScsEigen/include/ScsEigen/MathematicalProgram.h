/**
 * @file MathematicalProgram.h
 * @author Giulio Romualdi
 * @copyright Released under the terms of the MIT License.
 * @date 2021
 */

#ifndef SCS_EIGEN_MATHEMATICAL_PROGRAM_H
#define SCS_EIGEN_MATHEMATICAL_PROGRAM_H

#include <memory>
#include <string>
#include <unordered_map>

#include <ScsEigen/LinearCost.h>

/**
 * ScsEigen namespace.
 */
namespace ScsEigen
{
/**
 * MathematicalProgram is a class that describes an optimization problem
 */
class MathematicalProgram
{
    struct Impl;
    std::unique_ptr<Impl> m_pimpl;

public:
    MathematicalProgram();

    ~MathematicalProgram();

    void setNumberOfVariabels(unsigned int variables);

    unsigned int numberOfVariables() const;

    bool addLinearCost(std::shared_ptr<LinearCost> cost, std::string_view name);

    std::weak_ptr<LinearCost> getLinearCost(std::string_view name) const;

    const std::unordered_map<std::string_view, std::shared_ptr<LinearCost>>& getLinearCosts() const;
};

} // namespace ScsEigen

#endif // SCS_EIGEN_MATHEMATICAL_PROGRAM_H