# Quadratic Programming problem

**scs-eigen** can be used to solve Quadratic Programming (QP) problems in the for
\f[
\begin{array}{rl}
\text{minimize }      &  \frac{1}{2} x^\top P x + q^\top x + r \\
\text{subject to }    &  l \le A x \le u
\end{array}
\f]
where \f$x \in \mathbb{R}^n\f$ is the optimization variable. The objective function is defined by a positive semidefinite matrix \f$P \in S^n_+\f$ and vector \f$q \in \mathbb{R}^n\f$. The linear constraints are defined by matrix \f$A \in \mathbb{R}^{m \times n}\f$ and vectors \f$l\f$ and \f$u\f$ so that \f$l_i \in \mathbb{R} \cup \{-\infty\}\f$ and \f$u_i \in \mathbb{R} \cup \{\infty\}\f$ for all \f$i \in \{1,...,m\}\f$.

The following example shows how **scs-eigen** can be used to solve the QP problem:

\f[
\begin{array}{rl}
\text{minimize }      &  \frac{1}{2} x^\top \begin{bmatrix} 3 & 2 \\ 2 & 4\end{bmatrix} x + \begin{bmatrix} 3 & 1 \end{bmatrix} x \\
\text{subject to }    &  \begin{bmatrix} 1 \\ 0 \\0  \end{bmatrix} \le \begin{bmatrix} 1  & 1\\ 1 & 0 \\0 & 1  \end{bmatrix} x \le \begin{bmatrix} 1 \\ 0.7 \\0.7  \end{bmatrix}
\end{array}
\f]

@image html ../images/qp.png

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
```

Once the matrices used to described the QP problem have been defined you can create `ScsEigen::Solver` and initialize the number of variables.

```cpp
ScsEigen::Solver solver;
solver.setNumberOfVariabels(2);
```

Now you can set the constraints and the cost using `ScsEigen::MathematicalProgram` class

```cpp
solver.mathematicalProgram().addLinearConstraint(
        std::make_shared<ScsEigen::LinearConstraint>(A, lowerBound, upperBound),
        "linear constraint");

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
project(QP)
find_package(ScsEigen REQUIRED)
add_executable(QP qp.cpp)
target_link_libraries(QP ScsEigen::ScsEigen)
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

    ScsEigen::Solver solver;

    solver.mathematicalProgram().setNumberOVfariables(2);

    if (!solver.mathematicalProgram().addQuadraticCost(
        std::make_shared<ScsEigen::QuadraticCost>(H, gradient),
        "quadratic cost"))
        return EXIT_FAILURE;


    if (!solver.mathematicalProgram().addLinearConstraint(
        std::make_shared<ScsEigen::LinearConstraint>(A, lowerBound, upperBound),
        "linear constraint"))
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