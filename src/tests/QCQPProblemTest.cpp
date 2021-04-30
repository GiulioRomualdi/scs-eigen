// Catch2
#include <catch2/catch.hpp>

#include <ScsEigen/Solver.h>
#include <ScsEigen/MathematicalProgram.h>

#include <iostream>

TEST_CASE("QCQPProblem")
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
        "Quadratic cost"));

    Eigen::Matrix2d A = 2 * Eigen::Matrix2d::Identity();
    Eigen::Vector2d b;
    b << -4, -5;

    double upperBound = -1;

    REQUIRE(solver.mathematicalProgram().addQuadraticConstraint( //
        std::make_shared<ScsEigen::QuadraticConstraint>(A, b, upperBound),
        "quadratic constraint"));

    REQUIRE(solver.solve());
    REQUIRE(solver.solution().isValid());
    REQUIRE(solver.solution().status == ScsEigen::Solution::Status::solved);

    Eigen::Vector2d expectedSolution;
    expectedSolution << -0.343378301037831, 0.561295504094664;
    REQUIRE(solver.solution().solution.isApprox(expectedSolution, tolerance));
}
