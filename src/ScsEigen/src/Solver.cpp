/**
 * @file Solver.cpp
 * @author Giulio Romualdi
 * @copyright Released under the terms of the MIT License.
 * @date 2021
 */

#include <functional>
#include <numeric>

// scs
#include <scs.h>
#include <util.h>

#include <ScsEigen/Logger.h>
#include <ScsEigen/Math.h>
#include <ScsEigen/MathematicalProgram.h>
#include <ScsEigen/Settings.h>
#include <ScsEigen/Solver.h>

#include <Eigen/Sparse>

using namespace ScsEigen;

struct Solver::Impl
{
    Settings settings; /**< The settings of the ScsSolver */
    MathematicalProgram mathematicalProgram; /**< The mathematical problem that should be solved */

    double constant{0}; /**< Constant term in the cost function. */
    std::vector<double> gradient; /**< Gradient term in the cost function. */

    int optimizationVariables{0}; /**< Number of optimization variable */
    std::vector<int> secondOrderConesSize; /**< Sizes of the second order cone. */

    std::vector<Eigen::Triplet<double>> constraintMatrixTriplets; /**< Triplets associated to the
                                                                     constraint matrix. */
    unsigned int constraintMatrixRows{0}; /**< Number of the rows of the constraint matrix */
    std::vector<double> constraintVector; /**< vector containing the equality constraints */

    std::unique_ptr<ScsCone, std::function<void(ScsCone*)>> cone;

    Impl()
        : cone(static_cast<ScsCone*>(scs_calloc(1, sizeof(ScsCone))), [](ScsCone* ptr) {
            if (ptr != nullptr)
                // scs_free_data takes ScsData and ScsCone
                // https://github.com/cvxgrp/scs/blob/48dfbe81caad2162c3ce5757faccdb3f3d31e142/src/util.c#L155
                scs_free_data(nullptr, ptr);
        })
    {
    }

    bool linearCostEmbedding()
    {
        for (const auto& [name, cost] : this->mathematicalProgram.getLinearCosts())
        {
            assert(cost->getNumberOfVariables() >= this->gradient.size());

            // the first cost->getNumberOfVariables() must be greater than zero
            Eigen::Map<Eigen::VectorXd>(this->gradient.data(),
                                        this->mathematicalProgram.numberOfVariables())
                += cost->getA();
            this->constant += cost->getB();
        }

        return true;
    }
};

Solver::Solver()
    : m_pimpl(std::make_unique<Impl>())
{
}

Solver::~Solver() = default;

Settings& Solver::settings()
{
    return m_pimpl->settings;
}

const Settings& Solver::settings() const
{
    return m_pimpl->settings;
}

MathematicalProgram& Solver::mathematicalProgram()
{
    return m_pimpl->mathematicalProgram;
}

const MathematicalProgram& Solver::mathematicalProgram() const
{
    return m_pimpl->mathematicalProgram;
}

bool Solver::solve()
{
    if (m_pimpl->mathematicalProgram.numberOfVariables() == 0)
    {
        log()->error("[Solver::solve] The number of variables is equal to zero. Please set the "
                     "number of variables.");
        return false;
    }

    // The number of optimization variables may changes accordingly to the constraints and costs
    m_pimpl->optimizationVariables = m_pimpl->mathematicalProgram.numberOfVariables();

    // reset all the gradient vector equal to zero
    m_pimpl->gradient.resize(m_pimpl->optimizationVariables, 0.0);

    // perform the embedding
    if (!m_pimpl->linearCostEmbedding())
    {
        log()->error("[Solver::solve] Unable to perform linear cost embedding.");
        return false;
    }

    return true;
}
