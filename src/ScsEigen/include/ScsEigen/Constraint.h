/**
 * @file Constraint.h
 * @author Giulio Romualdi
 * @copyright Released under the terms of the MIT License.
 * @date 2021
 */

#ifndef SCS_EIGEN_CONSTRAINT_H
#define SCS_EIGEN_CONSTRAINT_H

#include <string>

#include <Eigen/Dense>

/**
 * ScsEigen namespace.
 */
namespace ScsEigen
{
/**
 * Constraint class describes a generic convex constraint set of the form
 * \f[
 * l \le _k f(x)  \le _k u
 * \f]
 * Where \f$ \le _k \f$ represents a generalized inequality.
 */
class Constraint
{
protected:
    std::string m_description{"Generic Constraint"}; /**< Description of the constraint. */
    int m_numberOfVariables{0}; /**< Number of variables associated to the constraint. */
    int m_numberOfConstraints{0}; /**< Size of the  constraint. */
    Eigen::VectorXd m_lowerBound; /**< Lower bound. */
    Eigen::VectorXd m_upperBound; /**< Upper bound. */

    /**
     * @brief Constructor,
     */
    Constraint() = default;

    /**
     * @brief Constructor,
     * @param numberOfVariables number of variables associated to the constraint.
     * @param description string describing the constraint.
     */
    Constraint(int numberOfVariables, std::string_view description = "");

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

    /**
     * @brief Set the lower bound.
     * @param lowerBound vector representing the lower bound.
     * @return true in case of success/false otherwise
     */
    virtual bool setLowerBound(const Eigen::Ref<const Eigen::VectorXd>& lowerBound);

    /**
     * @brief Set the upper bound.
     * @param upperBound vector representing the upper bound.
     * @return true in case of success/false otherwise
     */
    virtual bool setUpperBound(const Eigen::Ref<const Eigen::VectorXd>& upperBound);

    /**
     * @brief Set the lower and the upper bounds.
     * @param lowerBound vector representing the lower bound.
     * @param upperBound vector representing the upper bound.
     * @return true in case of success/false otherwise
     */
    bool setBounds(const Eigen::Ref<const Eigen::VectorXd>& lowerBound,
                   const Eigen::Ref<const Eigen::VectorXd>& upperBound);

    /**
     * @brief Set the number of constraints (i.e. the size of lowerBound and upperBound).
     * @param numberOfConstraints a positive number describing the size of lowerBound and
     * upperBound.
     * @return true in case of success/false otherwise
     */
    bool setNumberOfConstraints(int numberOfConstraints);

public:
    /**
     * @brief Desctructor.
     */
    ~Constraint() = default;

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

    /**
     * @brief Get the number of constraints.
     * @return the number of constraints,
     */
    int getNumberOfConstraints() const;

    /**
     * @brief Get the vector containing the lowerBound.
     * @return the lower bound.
     */
    Eigen::Ref<const Eigen::VectorXd> getLowerBound() const;

    /**
     * @brief Get the vector containing the upperBound.
     * @return the upper bound.
     */
    Eigen::Ref<const Eigen::VectorXd> getUpperBound() const;
};

} // namespace ScsEigen

#endif // SCS_EIGEN_CONSTRAINT_H
