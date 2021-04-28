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

    solver.mathematicalProgram().setNumberOfVariables(2);

    REQUIRE(solver.mathematicalProgram()
            .addQuadraticCost(std::make_shared<ScsEigen::QuadraticCost>(H, gradient), "test"));

    REQUIRE(solver.solve());
    REQUIRE(solver.solution().isValid());
    REQUIRE(solver.solution().status == ScsEigen::Solution::Status::solved);

    Eigen::Vector2d expectedSolution;
    expectedSolution << -1.2500, 0.3750;
    REQUIRE(solver.solution().solution.isApprox(expectedSolution, tolerance));
}

TEST_CASE("QPProblem - Constrained")
{
    constexpr double tolerance = 1e-4;

    ScsEigen::Solver solver;

    Eigen::Matrix2d H;
    H << 3, 2,
        2, 4;
    Eigen::Vector2d gradient;
    gradient << 3, 1;

    solver.mathematicalProgram().setNumberOfVariables(2);

    REQUIRE(solver.mathematicalProgram().addQuadraticCost( //
        std::make_shared<ScsEigen::QuadraticCost>(H, gradient),
        "test"));

    Eigen::MatrixXd A(3,2);
    A.setZero();
    A(0,0) = 1;
    A(0,1) = 1;
    A(1,0) = 1;
    A(2,1) = 1;

    Eigen::Vector3d lowerBound;
    lowerBound << 1, 0, 0;

    Eigen::Vector3d upperBound;
    upperBound << 1, 0.7, 0.7;

    REQUIRE(solver.mathematicalProgram().addLinearConstraint( //
        std::make_shared<ScsEigen::LinearConstraint>(A, lowerBound, upperBound),
        "linear constraint"));

    REQUIRE(solver.solve());
    REQUIRE(solver.solution().isValid());
    REQUIRE(solver.solution().status == ScsEigen::Solution::Status::solved);

    Eigen::Vector2d expectedSolution;
    expectedSolution << 0.3, 0.7;
    REQUIRE(solver.solution().solution.isApprox(expectedSolution, tolerance));
}
