# Authors: Giulio Romualdi
# Copyright: Released under the terms of the MIT License.

set(LIBRARY_NAME ${PROJECT_NAME})

set(${LIBRARY_NAME}_HDR
  include/ScsEigen/Settings.h
  include/ScsEigen/Logger.h
  include/ScsEigen/Cost.h
  include/ScsEigen/LinearCost.h
  include/ScsEigen/MathematicalProgram.h)

set(${LIBRARY_NAME}_SRC
  src/Settings.cpp
  src/Logger.cpp
  src/Cost.cpp
  src/LinearCost.cpp
  src/MathematicalProgram.cpp)

add_library(${LIBRARY_NAME} ${${LIBRARY_NAME}_HDR} ${${LIBRARY_NAME}_SRC})

target_compile_features(${LIBRARY_NAME} PUBLIC cxx_std_17)

target_link_libraries(${LIBRARY_NAME} PUBLIC Eigen3::Eigen)
target_link_libraries(${LIBRARY_NAME} PRIVATE scs::scsdir)

set_target_properties(${LIBRARY_NAME} PROPERTIES
  OUTPUT_NAME ${LIBRARY_NAME}
  VERSION ${PROJECT_VERSION}
  PUBLIC_HEADER "${${LIBRARY_NAME}_HDR}")

# Specify include directories for both compilation and installation process.
# The $<INSTALL_PREFIX> generator expression is useful to ensure to create
# relocatable configuration files, see https://cmake.org/cmake/help/latest/manual/cmake-packages.7.html
#creating-relocatable-packages
target_include_directories(${LIBRARY_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>")


# Specify installation targets, typology and destination folders.
install(TARGETS    ${LIBRARY_NAME}
  EXPORT           ${PROJECT_NAME}
  COMPONENT        runtime
  LIBRARY          DESTINATION "${CMAKE_INSTALL_LIBDIR}"                           COMPONENT shlib
  ARCHIVE          DESTINATION "${CMAKE_INSTALL_LIBDIR}"                           COMPONENT lib
  RUNTIME          DESTINATION "${CMAKE_INSTALL_BINDIR}"                           COMPONENT bin
  PUBLIC_HEADER    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}"       COMPONENT dev
  PRIVATE_HEADER   DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}/impl"  COMPONENT dev)

add_library(${PROJECT_NAME}::${LIBRARY_NAME} ALIAS ${LIBRARY_NAME})
