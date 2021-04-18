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
 * Cost
 */
class Cost
{
protected:
    std::string m_description{"Generic Cost"};
    int m_numberOfVariables{0};

    Cost() = default;

    Cost(int numverOfVariables, std::string_view description = "");

    void setDescription(std::string_view description);

    bool setNumberOfVariables(int numberOfVariables);

public:
    ~Cost() = default;

    std::string_view getDescrition() const;
    int getNumberOfVariables() const;
};

} // namespace ScsEigen

#endif // SCS_EIGEN_COST_H
