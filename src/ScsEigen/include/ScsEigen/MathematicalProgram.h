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

#include <ScsEigen/LinearConstraint.h>
#include <ScsEigen/LinearCost.h>
#include <ScsEigen/QuadraticCost.h>

/**
 * ScsEigen namespace.
 */
namespace ScsEigen
{
/**
 * MathematicalProgram is a class that describes an optimization problem. Please use if when you
 * want to define an optimization problem.
 *
 * This simple example shows you how to use the MathematicalProgram class
 * ```cpp
 * #include <ScsEigen/MathematicalProgram.h>
 *
 * ...
 *
 * // Initialize the mathematical programming problem
 * ScsEigen::MathematicalProgram mp;
 *
 * // Add a quadratic cost
 * mp.addQuadraticCost(
 *        std::make_shared<ScsEigen::QuadraticCost>(H,
 *        "quadratic cost"));
 *
 * // Add linear constraint
 * mp.addLinearConstraint(
 *        std::make_shared<ScsEigen::LinearConstraint>(A, lowerBound, upperBound),
 *        "linear constraint"));
 * ```
 */
class MathematicalProgram
{
    struct Impl;
    std::unique_ptr<Impl> m_pimpl;

public:
    /**
     * @brief dictionary used to store cost and constraints.
     */
    template <class T> using dictionary = std::unordered_map<std::string_view, std::shared_ptr<T>>;

    /**
     * @brief Constructor.
     */
    MathematicalProgram();

    /**
     * @brief Destructor.
     */
    ~MathematicalProgram();

    /**
     * @brief Set the number of variables.
     */
    void setNumberOfVariables(unsigned int variables);

    /**
     * @brief Get the number of variables
     */
    unsigned int numberOfVariables() const;

    /**
     * @brief Set the linear cost
     * @param cost a shared pointer to the cost.
     * @param name the name of the cost.
     * @note the User is in charge to set the vectors related to the cost.
     */
    bool addLinearCost(std::shared_ptr<LinearCost> cost, std::string_view name);

    /**
     * @brief Get the linear cost.
     * @param name name of the cost.
     * @return a weak_ptr to the cost.
     * @note if the cost associated to the name "name" is not found an unlockable weak_ptr is
     * returned.
     */
    std::weak_ptr<LinearCost> getLinearCost(std::string_view name) const;

    /**
     * @brief Get all the linear costs stored the in the MathematicalProgram class.
     */
    const dictionary<LinearCost>& getLinearCosts() const;

    /**
     * @brief Set the quadratic cost
     * @param cost a shared pointer to the cost.
     * @param name the name of the cost.
     * @note the User is in charge to set the vectors related to the cost.
     */
    bool addQuadraticCost(std::shared_ptr<QuadraticCost> cost, std::string_view name);

    /**
     * @brief Get the quadratic cost.
     * @param name name of the cost.
     * @return a weak_ptr to the cost.
     * @note if the cost associated to the name "name" is not found an unlockable weak_ptr is
     * returned.
     */
    std::weak_ptr<QuadraticCost> getQuadraticCost(std::string_view name) const;

    /**
     * @brief Get all the quadratic costs stored the in the MathematicalProgram class.
     */
    const dictionary<QuadraticCost>& getQuadraticCosts() const;

    /**
     * @brief Set the linear constraint
     * @param constraint a shared pointer to the constraint.
     * @param name the name of the constraint.
     * @note the User is in charge to set the vectors related to the constraint.
     */
    bool addLinearConstraint(std::shared_ptr<LinearConstraint> constraint, std::string_view name);

    /**
     * @brief Get the linear constraint.
     * @param name name of the constraint.
     * @return a weak_ptr to the constraint.
     * @note if the constraint associated to the name "name" is not found an unlockable weak_ptr is
     * returned.
     */
    std::weak_ptr<LinearConstraint> getLinearConstraint(std::string_view name) const;

    /**
     * @brief Get all the linear constraints stored the in the MathematicalProgram class.
     */
    const dictionary<LinearConstraint>& getLinearConstraints() const;
};

} // namespace ScsEigen

#endif // SCS_EIGEN_MATHEMATICAL_PROGRAM_H
