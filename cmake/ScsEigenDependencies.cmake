
include(ScsEigenFindOptionalDependencies)

#---------------------------------------------
## Required Dependencies
find_package(Eigen3 3.2.92 REQUIRED)
find_package(scs REQUIRED)

#---------------------------------------------
## Optional Dependencies. Required if you want to enable the tests
find_package(Catch2 QUIET)
checkandset_optional_dependency(Catch2)

find_package(VALGRIND QUIET)
checkandset_optional_dependency(VALGRIND)
