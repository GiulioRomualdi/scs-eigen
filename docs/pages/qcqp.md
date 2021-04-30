# Quadratic Constrained Quadratic Programming problem

**scs-eigen** can be used to solve Quadratic Constrained Quadratic Programming (QCQP) problems in the for
\f[
\begin{array}{rl}
\text{minimize }      &  \frac{1}{2} x^\top P_0 x + q_0^\top x + r \\
\text{subject to }    &  \frac{1}{2} x^\top P_i x + q_i^\top x \le u_i
\end{array}
\f]
where \f$x \in \mathbb{R}^n\f$ is the optimization variable. The objective function is defined by a positive semidefinite matrix \f$P_0 \in S^n_+\f$ and vector \f$q_0 \in \mathbb{R}^n\f$. The quadratic constraints are defined  positive semidefinite matrices\f$P_i \in S^n_+\f$ and vectors \f$q_i \in \mathbb{R}^n\f$.

The following example shows how **scs-eigen** can be used to solve the QCQP problem:

\f[
\begin{array}{rl}
\text{minimize }      &  \frac{1}{2} x^\top \begin{bmatrix} 3 & 2 \\ 2 & 4\end{bmatrix} x + \begin{bmatrix} 3 & 1 \end{bmatrix} x \\
\text{subject to }    &  \frac{1}{2} x^\top \begin{bmatrix} 2 & 0 \\ 0 & 2\end{bmatrix} x + \begin{bmatrix} -4 & -5 \end{bmatrix} x \le -1
\end{array}
\f]

@image html ../images/qcqp.png

First of all you should include ScsEigen

```cpp
#include <ScsEigen/ScsEigen.h>
```

You can also define the Hessian, gradient the constraint matrix and vectors.

```cpp
Eigen::Matrix2d H;
H << 3, 2,
     2, 4;
Eigen::Vector2d gradient;
gradient << 3, 1;

Eigen::Matrix2d A = 2 * Eigen::Matrix2d::Identity();
Eigen::Vector2d b;
b << -4, -5;

double upperBound = -1;

```

Once the matrices used to described the QCQP problem have been defined you can create `ScsEigen::Solver` and initialize the number of variables.

```cpp
ScsEigen::Solver solver;
solver.setNumberOfVariabels(2);
```

Now you can set the constraints and the cost using `ScsEigen::MathematicalProgram` class

```cpp
solver.mathematicalProgram().addQuadraticConstraint(
        std::make_shared<ScsEigen::QuadraticConstraint>(A, b, upperBound),
        "quadratic constraint");

solver.mathematicalProgram().addQuadraticCost(
        std::make_shared<ScsEigen::QuadraticCost>(H, gradient),
        "quadratic cost");

```

You can finally solve the problem and get the solution

```cpp
solver.solve();
Eigen::Vector2d solution = solver.solution().solution;
```

The complete example follows

```cmake
### CMakeLists.txt
project(QCQP)
find_package(ScsEigen REQUIRED)
add_executable(QCQP qcqp.cpp)
target_link_libraries(QCQP ScsEigen::ScsEigen)
```

```cpp
/// qp.cpp

#include <ScsEigen/ScsEigen.h>

#include <Eigen/Dense>

int main()
{
    Eigen::Matrix2d H;
    H << 3, 2,
         2, 4;

    Eigen::Vector2d gradient;
    gradient << 3, 1;

    Eigen::Matrix2d A = 2 * Eigen::Matrix2d::Identity();
    Eigen::Vector2d b;
    b << -4, -5;

    ScsEigen::Solver solver;

    solver.mathematicalProgram().setNumberOVfariables(2);

    if (!solver.mathematicalProgram().addQuadraticCost(
        std::make_shared<ScsEigen::QuadraticCost>(H, gradient),
        "quadratic cost"))
        return EXIT_FAILURE;


    if (!solver.mathematicalProgram().addQuadraticConstraint( //
        std::make_shared<ScsEigen::QuadraticConstraint>(A, b, upperBound),
        "quadratic constraint"))
        return EXIT_FAILURE;

    if (!solver.solve())
        return EXIT_FAILURE;
    if (!solver.solution().isValid())
        return EXIT_FAILURE;
    if (solver.solution().status != ScsEigen::Solution::Status::solved)
        return EXIT_FAILURE;

    Eigen::Vector2d solution = solver.solution().solution;

    return EXIT_SUCCESS;
}
```