# scs-eigen <a href="./LICENSE"><img src="https://img.shields.io/badge/License-MIT-yellow.svg" alt="Size" /></a>

## Dependencies
The project depends only on [`scs`](https://github.com/cvxgrp/scs) and [Eigen3](http://eigen.tuxfamily.org/index.php?title=Main_Page). Please install `scs` using the `cmake-build` system provide in [this repository](https://github.com/dic-iit/scs-cmake-buildsystem).

## Documentation

The documentation is available online at the accompanying [website](https://giulioromualdi.github.io/scs-eigen/).

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

## How to use the library
**scs-eigen** provides native `CMake` support which allows the library to be easily used in `CMake` projects.
**scs-eigen** exports a CMake target called `ScsEigen::ScsEigen` which can be imported using the `find_package` CMake command and used by calling `target_link_libraries` as in the following example:
```cmake
cmake_minimum_required(VERSION 3.0)
project(myproject)
find_package(ScsEigen REQUIRED)
add_executable(example example.cpp)
target_link_libraries(example ScsEigen::ScsEigen)
```

## License
Materials in this repository are distributed under the following license:

> All software is licensed under the MIT License. See [LICENSE](./LICENSE) file for details.
