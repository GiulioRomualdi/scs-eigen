# scs-eigen

![License](https://img.shields.io/badge/License-MIT-yellow.svg)

## Dependencies
The project depends only on [`scs`](https://github.com/cvxgrp/scs) and [Eigen3](http://eigen.tuxfamily.org/index.php?title=Main_Page). Please install `scs` using the `cmake-build` system provided in [this repository](https://github.com/dic-iit/scs-cmake-buildsystem).

## Usage
1. Clone the repository

   ```
   git clone https://github.com/GiulioRomualdi/scs-eigen
   ```
2. Build it

   ```
   cd scs-eigen
   mkdir build
   cd build
   cmake -DCMAKE_INSTALL_PREFIX:PATH=<custom-folder> ../
   make
   make install
   ```
3. Add the following environmental variable
   ```
   ScsEigen_DIR=/path/where/you/installed/
   ```

## How to include the library in your CMake project
**scs-eigen** provides native `CMake` support which allows the library to be easily used in `CMake` projects.
**scs-eigen** exports a CMake target called `ScsEigen::ScsEigen` which can be imported using the `find_package` CMake command and used by calling `target_link_libraries` as in the following example:
```cmake
cmake_minimum_required(VERSION 3.0)
project(myproject)
find_package(ScsEigen REQUIRED)
add_executable(example example.cpp)
target_link_libraries(example ScsEigen::ScsEigen)
```

In your code
```cpp
#include <ScsEigen/ScsEigen.h>

...

auto solver = ScsEigen::Solver();

...
```

## How to use the library
**scs-eigen** introduces the `ScsEigen::MathematicalProgram` class that can be used to easily design a convex optimization problem.

`ScsEigen::MathematicalProgram` can be seen as an helper that allows you to easily add convex costraints and convex costs. Once the cost and the constraints have been added to `ScsEigen::MathematicalProgram`, `ScsEigen::Solver` will perform the so called conic-embedding to convert them in a form compatible with `scs`.

We provide a list of self-contained example that can be used as a refernce to build an optimization problem:
- [QP](pages/qp.md) contains an example of Quadratic Programming (QP) problem

## License
Materials in this repository are distributed under the following license:

> All software is licensed under the MIT License. See [LICENSE](https://github.com/GiulioRomualdi/scs-eigen/blob/main/LICENSE) file for details.
