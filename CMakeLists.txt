cmake_minimum_required(VERSION 3.15)

project(MultilaterationAlgorithms)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

option(MULTILAT_BUILD_CLI "Build CLI test runner" ON)
if(EMSCRIPTEN)
    option(MULTILAT_BUILD_WEBAPP "Build web app" ON)
else()
    option(MULTILAT_BUILD_WEBAPP "Build web app" OFF)
endif()

add_subdirectory(libs)
add_subdirectory(src)
