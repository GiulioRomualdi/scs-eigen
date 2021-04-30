# Authors: Giulio Romualdi
# Copyright: Released under the terms of the MIT License.

include(AddScsEigenUnitTest)

add_scseigen_test(
  NAME QPProblem
  SOURCES QPProblemTest.cpp
  LINKS ScsEigen::ScsEigen)

add_scseigen_test(
  NAME QCQPProblem
  SOURCES QCQPProblemTest.cpp
  LINKS ScsEigen::ScsEigen)
