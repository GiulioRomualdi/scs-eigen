/**
 * @file Cost.h
 * @author Giulio Romualdi
 * @copyright Released under the terms of the MIT License.
 * @date 2021
 */

#ifndef SCS_EIGEN_COST_H
#define SCS_EIGEN_COST_H

#include <string>

#include <Eigen/Dense>

/**
 * ScsEigen namespace.
 */
namespace ScsEigen
{
/**
 * Cost describes a generic convex cost.
 */
class Cost
{
protected:
    std::string m_description{"Generic Cost"}; /**< Description of the cost. */
    int m_numberOfVariables{0}; /**< Number of variables associated to the constraint. */

    /**
     * @brief Constructor,
     */
    Cost() = default;

    /**
     * @brief Constructor,
     * @param numberOfVariables number of variables associated to the constraint.
     * @param description string describing the constraint.
     */
    Cost(int numberOfVariables, std::string_view description = "");

    /**
     * @brief Set the description of the constraint.
     * @param description string describing the constraint.
     */
    void setDescription(std::string_view description);

    /**
     * @brief Set the number of variables.
     * @param numberOfVariables a positive number describing the number of variables.
     * @return true in case of success/false otherwise
     */
    bool setNumberOfVariables(int numberOfVariables);

public:
    /**
     * @brief Desctructor
     */
    ~Cost() = default;

    /**
     * @brief Get the description
     * @return the string containng the description
     */
    std::string_view getDescription() const;

    /**
     * @brief Get the number of variables.
     * @return the number of variables,
     */
    int getNumberOfVariables() const;
};

} // namespace ScsEigen

#endif // SCS_EIGEN_COST_H
