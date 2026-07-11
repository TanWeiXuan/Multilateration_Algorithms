cmake_minimum_required(VERSION 3.15)

project(MultilaterationAlgorithms)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

option(MULTILAT_BUILD_CLI "Build CLI test runner" ON)
option(MULTILAT_BUILD_WEBAPP "Build web app" OFF)

if(EMSCRIPTEN)
    # Ensure the web target is enabled automatically under emcmake unless
    # explicitly disabled by a user before this point.
    set(MULTILAT_BUILD_WEBAPP ON CACHE BOOL "Build web app" FORCE)
endif()

add_subdirectory(libs)
add_subdirectory(src)
