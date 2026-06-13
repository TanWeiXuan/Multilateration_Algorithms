# Build System

## Overview

The repository uses CMake to build a single C++20 executable named `main`. The top-level project delegates dependency target setup to `libs/` and executable setup to `src/`.

## Files

### `CMakeLists.txt`

- Requires CMake 3.15 or newer.
- Declares the `MultilaterationAlgorithms` project.
- Sets `CMAKE_CXX_STANDARD` to C++20.
- Places runtime binaries in `${CMAKE_BINARY_DIR}/bin`.
- Adds the `libs` and `src` subdirectories.

### `src/CMakeLists.txt`

- Defines the `main` executable.
- Compiles:
  - `src/main.cpp`
  - `src/true_range_multilateration_methods.cpp`
  - `src/tests.cpp`
  - `src/test_helpers.cpp`
- Adds `src/` as a private include directory.
- Links against the vendored `Eigen` and `EigenUnsupported` interface targets.

### `libs/CMakeLists.txt`

- Defines a header-only `Eigen` interface target for `libs/eigen-5.0.0`.
- Defines a header-only `EigenUnsupported` interface target intended to expose Eigen's unsupported modules.

## Typical commands

```bash
cmake -S . -B build
cmake --build build
./build/bin/main
```

## Maintainer notes

- The executable and validation checks are currently coupled: building `main` also builds the algorithms and the test harness into one binary.
- There is no separate library target for the multilateration algorithms yet. Creating one would make the code easier to reuse from external applications and unit tests.
- See [Future plans and known issues](future-plans.md) for a known include-directory issue in the `EigenUnsupported` CMake setup.
