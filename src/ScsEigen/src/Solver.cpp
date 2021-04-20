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

    void rotateSecondOrderConeEmbedding(const std::vector<Eigen::Triplet<double>>& AConeTriplets,
                                        const Eigen::Ref<const Eigen::VectorXd>& bCone,
                                        const std::vector<int>& AiVarInded)
    {
        for (const auto& triplet : AConeTriplets)
        {
            const int xIndex = AiVarInded[triplet.col()];
            if (triplet.row() == 0)
            {
                this->constraintMatrixTriplets.emplace_back(this->constraintMatrixRows,
                                                            xIndex,
                                                            -0.5 * triplet.value());
                this->constraintMatrixTriplets.emplace_back(this->constraintMatrixRows + 1,
                                                            xIndex,
                                                            -0.5 * triplet.value());
            } else if (triplet.row() == 1)
            {
                this->constraintMatrixTriplets.emplace_back(this->constraintMatrixRows,
                                                            xIndex,
                                                            -0.5 * triplet.value());
                this->constraintMatrixTriplets.emplace_back(this->constraintMatrixRows + 1,
                                                            xIndex,
                                                            0.5 * triplet.value());
            } else
            {
                this->constraintMatrixTriplets.emplace_back(this->constraintMatrixRows
                                                                + triplet.row(),
                                                            xIndex,
                                                            -triplet.value());
            }
        }

        assert(bCone.size() >= 2
               && "Solver::Impl::rotateSecondOrderConeEmbedding] The size of the cone should be at "
                  "least 2.");

        this->constraintVector.push_back(0.5 * (bCone(0) + bCone(1)));
        this->constraintVector.push_back(0.5 * (bCone(0) - bCone(1)));
        for (int i = 2; i < bCone.rows(); ++i)
        {
            this->constraintVector.push_back(bCone(i));
        }
        this->constraintMatrixRows += bCone.rows();
        this->secondOrderConesSize.push_back(bCone.rows());
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

    // A QuadraticCost encodes cost of the form
    //   0.5 xᵀQx + pᵀx + r
    // Since SCS only supports linear cost, we transform the cost using the epigraph form.
    // In details we introduce a new slack variable y as the upper bound of the cost, with the
    // rotated Lorentz cone constraint 2(y - r - pᵀx) ≥ xᵀQx.
    //    minimize_{x}  0.5 xᵀQx + pᵀx + r
    // becomes
    //    minimize_{x, y}  y
    //    subject to  2(y - r - pᵀx) ≥ xᵀQx.
    bool quadraticCostEmbedding()
    {
        std::vector<int> AiVarIndex(this->mathematicalProgram.numberOfVariables());
        std::iota(std::begin(AiVarIndex), std::end(AiVarIndex), 0);

        for (const auto& [name, cost] : this->mathematicalProgram.getQuadraticCosts())
        {
            const auto numberOfVariables = cost->getNumberOfVariables();

            // we create a vector containing the triplets of the matrix A_cone
            std::vector<Eigen::Triplet<double>> AConeTriplets;

            // Here we add the element in the first column latest row of the matrix A
            AConeTriplets.emplace_back(0, numberOfVariables, 1);

            // Decompose Q in CᵀC
            // Q is always SDP. Please check QuadraticCost::setQ()
            const auto [outcome, C] = choleskyDecomposition(cost->getQ());
            if (!outcome)
            {
                log()->error("[Solver::Impl::quadraticCostEmbedding] Unable to compute the "
                             "Cholesky decomposition for the hessian matrix associated to the "
                             "quadratic cost named: "
                             + std::string(name) + ".");
                return false;
            }

            // get the element different from zero and store it in the triplets
            for (int i = 0; i < C.rows(); ++i)
                for (int j = 0; j < C.cols(); ++j)
                    if (C(i, j) != 0)
                        AConeTriplets.emplace_back(2 + i, j, C(i, j));

            // add the slack variables as optimization variable
            AiVarIndex.push_back(this->optimizationVariables);
            this->optimizationVariables++;

            Eigen::VectorXd bCone = Eigen::VectorXd::Zero(2 + C.cols());
            bCone(0) = -cost->getC();
            bCone(1) = 2;

            // Add the secondOrderCone to the constraints
            this->rotateSecondOrderConeEmbedding(AConeTriplets, bCone, AiVarIndex);

            // increase the gradient with the slack variable
            this->gradient.push_back(1);
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

    if (!m_pimpl->quadraticCostEmbedding())
    {
        log()->error("[Solver::solve] Unable to perform quadratic cost embedding.");
        return false;
    }

    return true;
}
