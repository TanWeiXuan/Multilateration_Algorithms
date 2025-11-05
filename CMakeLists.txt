cmake_minimum_required(VERSION 3.15)

project(MultilaterationAlgorithms)

set(CMAKE_CXX_STANDARD 20)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# libraries
add_subdirectory(libs)

# source files
add_subdirectory(src)

