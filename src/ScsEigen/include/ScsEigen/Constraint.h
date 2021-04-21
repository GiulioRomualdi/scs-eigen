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
 * Constraint
 */
class Constraint
{
protected:
    std::string m_description{"Generic Constraint"};
    int m_numberOfVariables{0};
    int m_numberOfConstraints{0};
    Eigen::VectorXd m_lowerBound;
    Eigen::VectorXd m_upperBound;

    Constraint() = default;

    Constraint(int numberOfVariables,
               std::string_view description = "");

    void setDescription(std::string_view description);

    bool setNumberOfVariables(int numberOfVariables);

    bool setLowerBound(const Eigen::Ref<const Eigen::VectorXd>& lowerBound);
    bool setUpperBound(const Eigen::Ref<const Eigen::VectorXd>& upperBound);

    bool setBounds(const Eigen::Ref<const Eigen::VectorXd>& lowerBound,
                   const Eigen::Ref<const Eigen::VectorXd>& upperBound);

    bool setNumberOfConstraints(int numberOfConstraints);

public:
    ~Constraint() = default;

    std::string_view getDescrition() const;

    int getNumberOfVariables() const;
    int getNumberOfConstraints() const;
    Eigen::Ref<const Eigen::VectorXd> getLowerBound() const;
    Eigen::Ref<const Eigen::VectorXd> getUpperBound() const;
};

} // namespace ScsEigen

#endif // SCS_EIGEN_CONSTRAINT_H
