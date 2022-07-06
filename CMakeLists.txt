# Copyright (c) 2022 Norwegian Defence Research Establishment (FFI)

cmake_minimum_required(VERSION 3.10)
project(ptcee VERSION 1.0.18)

include(${CMAKE_CURRENT_BINARY_DIR}/conan_paths.cmake OPTIONAL)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/module-boilerplate.cmake)

find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Sophus REQUIRED)

file(GLOB_RECURSE SRC_FILES src/*.cpp)
file(GLOB_RECURSE HEADER_FILES include/*.h)

add_library(${PROJECT_NAME}
  ${HEADER_FILES}
  ${SRC_FILES}
  )

target_link_libraries(${PROJECT_NAME}
  PUBLIC Eigen3::Eigen
  PUBLIC GTSAM::GTSAM
  PUBLIC Sophus::Sophus
  )

configure_module("ptcee" "ptcee")
