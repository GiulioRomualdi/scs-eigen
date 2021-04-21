/**
 * @file Solver.cpp
 * @author Giulio Romualdi
 * @copyright Released under the terms of the MIT License.
 * @date 2021
 */

#include <functional>
#include <numeric>

// clang-format off
// scs.h should be included before linsys/amatrix.h, since amatrix.h uses types
// scs_float, scs_int, etc, defined in scs.h
#include <scs.h>
#include <cones.h>
#include <linalg.h>
#include <util.h>
#include <amatrix.h>
// clang-format on

#include <ScsEigen/Logger.h>
#include <ScsEigen/Math.h>
#include <ScsEigen/MathematicalProgram.h>
#include <ScsEigen/Settings.h>
#include <ScsEigen/Solver.h>
#include <ScsEigen/impl/SettingsImpl.h>

#include <Eigen/Sparse>

using namespace ScsEigen;

struct Solver::Impl
{
    Settings settings; /**< The settings of the ScsSolver */
    MathematicalProgram mathematicalProgram; /**< The mathematical problem that should be solved */

    double constant{0}; /**< Constant term in the cost function. */
    std::vector<scs_float> gradient; /**< Gradient term in the cost function. */

    int optimizationVariables{0}; /**< Number of optimization variable */
    std::vector<scs_int> secondOrderConesSize; /**< Sizes of the second order cone. */

    Eigen::SparseMatrix<scs_float, 0, scs_int> constraintMatrix;
    std::vector<Eigen::Triplet<scs_float, scs_int>> constraintMatrixTriplets; /**< Triplets associated to the
                                                                     constraint matrix. */
    unsigned int constraintMatrixRows{0}; /**< Number of the rows of the constraint matrix */
    std::vector<scs_float> constraintVector; /**< vector containing the equality constraints */

    std::unique_ptr<ScsCone, std::function<void(ScsCone*)>> cone;
    std::unique_ptr<ScsData, std::function<void(ScsData*)>> data;
    std::unique_ptr<ScsSolution, std::function<void(ScsSolution*)>> solution;

    Impl()
        : cone(static_cast<ScsCone*>(scs_calloc(1, sizeof(ScsCone))),
               [](ScsCone* ptr) {
                   // scs_free_data takes ScsData and ScsCone
                   // https://github.com/cvxgrp/scs/blob/48dfbe81caad2162c3ce5757faccdb3f3d31e142/src/util.c#L155
                   if (ptr != nullptr)
                       scs_free_data(nullptr, ptr);
               })
        , data(static_cast<ScsData*>(scs_calloc(1, sizeof(ScsData))),
               [](ScsData* ptr) {
                   // scs_free_data takes ScsData and ScsCone
                   // https://github.com/cvxgrp/scs/blob/48dfbe81caad2162c3ce5757faccdb3f3d31e142/src/util.c#L155
                   if (ptr != nullptr)
                       scs_free_data(ptr, nullptr);
               })
        , solution(static_cast<ScsSolution*>(scs_calloc(1, sizeof(ScsSolution))),
                   [](ScsSolution* ptr) {
                       if (ptr != nullptr)
                           scs_free_sol(ptr);
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

            // Set the constraint
            for (int i = 0; i < cost->getB().rows(); ++i)
            {
                AConeTriplets.emplace_back(0, i, -cost->getB()(i));
            }

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

    bool linearConstraintEmbedding()
    {
        // The linear constraint lb ≤ Ax ≤ ub is converted to
        //
        //  Ax + s1 = ub,
        // -Ax + s2 = lb
        // s1, s2  are two vectors that belongs to the positive orthant.
        // If one of the element of lb or ub is equal to plus or minus infinity it is not added in
        // the constraint.
        unsigned int linearConstraintsRowIndex{0};
        for (const auto& [name, constraint] : this->mathematicalProgram.getLinearConstraints())
        {
            const Eigen::Ref<const Eigen::VectorXd> lowerBound = constraint->getLowerBound();
            const Eigen::Ref<const Eigen::VectorXd> upperBound = constraint->getUpperBound();

            const Eigen::Ref<const Eigen::MatrixXd> A = constraint->getA();
            const Eigen::Ref<const LinearConstraint::SparsityPattern> sparsityPattern = constraint->getASparsityPattern();

            // analyze each row of the matrix
            for (int i = 0; i < constraint->getNumberOfConstraints(); ++i)
            {
                const bool isLowerBoundFinite{!std::isinf(lowerBound(i))};
                const bool isUpperBoundFinite{!std::isinf(upperBound(i))};

                // if both of the elements are infinite the row is skipped
                if (isLowerBoundFinite || isUpperBoundFinite)
                {
                    const int lowerBoundRowIndex
                        = this->constraintMatrixRows + linearConstraintsRowIndex;
                    const int upperBoundRowIndex
                        = lowerBoundRowIndex + (isLowerBoundFinite ? 1 : 0);
                    for (int j = 0; j < this->mathematicalProgram.numberOfVariables(); ++j)
                    {
                        if (sparsityPattern(i, j))
                        {
                            if (isLowerBoundFinite)
                            {
                                this->constraintMatrixTriplets.emplace_back(lowerBoundRowIndex,
                                                                            j,
                                                                            -A(i, j));
                            }

                            if (isUpperBoundFinite)
                            {
                                this->constraintMatrixTriplets.emplace_back(upperBoundRowIndex,
                                                                            j,
                                                                            A(i, j));
                            }
                        }
                    }
                    if (isLowerBoundFinite)
                    {
                        this->constraintVector.push_back(-lowerBound(i));
                        linearConstraintsRowIndex++;
                    }
                    if (isUpperBoundFinite)
                    {
                        this->constraintVector.push_back(upperBound(i));
                        linearConstraintsRowIndex++;
                    }
                }
            }
        }

        this->constraintMatrixRows += linearConstraintsRowIndex;
        this->cone->l += linearConstraintsRowIndex;
        return true;
    }

    void prepareScsData()
    {
        this->data->n = this->optimizationVariables;
        this->data->m = this->constraintMatrixRows;

        this->data->A = static_cast<ScsMatrix*>(malloc(sizeof(ScsMatrix)));
        this->data->A->x = static_cast<scs_float*>(
            scs_calloc(this->constraintMatrix.nonZeros(), sizeof(scs_float)));

        this->data->A->i = static_cast<scs_int*>(scs_calloc(this->constraintMatrix.nonZeros(), //
                                                            sizeof(scs_int)));
        this->data->A->p = static_cast<scs_int*>(scs_calloc(this->data->n + 1, sizeof(scs_int)));

        this->data->A->m = this->data->m;
        this->data->A->n = this->data->n;

        std::memcpy(this->data->A->x,
                    this->constraintMatrix.valuePtr(),
                    this->constraintMatrix.nonZeros() * sizeof(scs_float));

        std::memcpy(this->data->A->i,
                    this->constraintMatrix.innerIndexPtr(),
                    this->constraintMatrix.nonZeros() * sizeof(scs_int));

        std::memcpy(this->data->A->p,
                    this->constraintMatrix.outerIndexPtr(),
                    (this->data->n + 1) * sizeof(scs_int));

        this->data->b = static_cast<scs_float*>(scs_calloc(this->constraintVector.size(), //
                                                           sizeof(scs_float)));

        std::memcpy(this->data->b,
                    this->constraintVector.data(),
                    this->constraintVector.size() * sizeof(scs_float));

        this->data->c = static_cast<scs_float*>(scs_calloc(this->optimizationVariables, //
                                                           sizeof(scs_float)));
        std::memcpy(this->data->c,
                    this->gradient.data(),
                    this->optimizationVariables * sizeof(scs_float));

        // Set the parameters to default values.
        this->data->stgs = static_cast<ScsSettings*>(scs_calloc(1, sizeof(ScsSettings)));

        // we should change this
        scs_set_default_settings(this->data.get());
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

    if (!m_pimpl->linearConstraintEmbedding())
    {
        log()->error("[Solver::solve] Unable to perform linear constraint embedding.");
        return false;
    }

    if (!m_pimpl->quadraticCostEmbedding())
    {
        log()->error("[Solver::solve] Unable to perform quadratic cost embedding.");
        return false;
    }

    // set the second-order cone length in the SCS cone struct
    // Here dynamic allocation is performed.
    // The moeory will be automatically deallocated when Impl::cone goes out of scope
    m_pimpl->cone->qsize = m_pimpl->secondOrderConesSize.size();
    m_pimpl->cone->q = static_cast<scs_int*>(scs_calloc(m_pimpl->cone->qsize, sizeof(scs_int)));

    for (unsigned int i = 0; i < m_pimpl->cone->qsize; i++)
    {
        m_pimpl->cone->q[i] = m_pimpl->secondOrderConesSize[i];
    }

    // prepare the constraint matrix
    m_pimpl->constraintMatrix.resize(m_pimpl->constraintMatrixRows, m_pimpl->optimizationVariables);
    m_pimpl->constraintMatrix.setFromTriplets(m_pimpl->constraintMatrixTriplets.begin(),
                                              m_pimpl->constraintMatrixTriplets.end());

    m_pimpl->constraintMatrix.makeCompressed();

    m_pimpl->prepareScsData();

    // TODO (GR) handle the settings
    ScsInfo info{0};

    // solve the problem
    scs(m_pimpl->data.get(), m_pimpl->cone.get(), m_pimpl->solution.get(), &info);

    return true;
}
