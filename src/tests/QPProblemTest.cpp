// Catch2
#include <catch2/catch.hpp>

#include <ScsEigen/Solver.h>
#include <ScsEigen/MathematicalProgram.h>

TEST_CASE("QPProblem - Unconstrained")
{
    constexpr double tolerance = 1e-4;

    ScsEigen::Solver solver;

    Eigen::Matrix2d H;
    H << 3, 2,
        2, 4;
    Eigen::Vector2d gradient;
    gradient << 3, 1;

    solver.mathematicalProgram().setNumberOfVariabels(2);

    REQUIRE(solver.mathematicalProgram()
            .addQuadraticCost(std::make_shared<ScsEigen::QuadraticCost>(H, gradient), "test"));



    REQUIRE(solver.solve());
}
