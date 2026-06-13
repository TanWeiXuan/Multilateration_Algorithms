# Multilateration Algorithms Documentation

This repository contains C++ implementations and evaluation utilities for **true-range multilateration** in 3D. Given a set of known anchor positions and measured distances from those anchors to an unknown target, the code estimates the target position and compares several linear, nonlinear, weighted, robust, and lower-bound analysis approaches.

The project is intentionally compact: a single CMake-built executable (`main`) configures a simulation scenario, runs the implemented algorithms over many Monte Carlo trials, and prints error and timing summaries. Eigen 5.0.0 is vendored under `libs/` and is used for dense linear algebra, SVD-based least-squares solves, and unsupported nonlinear optimization routines.

## Repository map

| Area | Purpose | Detailed documentation |
| --- | --- | --- |
| `CMakeLists.txt`, `src/CMakeLists.txt`, `libs/CMakeLists.txt` | Defines the C++20 CMake project, builds the `main` executable, and exposes vendored Eigen targets. | [Build system](build-system.md) |
| `src/main.cpp` | Entry point that defines the default 3D target/anchor simulation, noise settings, random seed, run count, and launches tests. | [Executable and simulation entry point](main.md) |
| `src/true_range_multilateration_methods.h/.cpp` | Core multilateration algorithms and CRLB calculation. | [Algorithms module](algorithms.md) |
| `src/tests.h/.cpp` | Test harness types and orchestration for validating helpers, running scenario sets, and timing algorithm calls. | [Test harness](test-harness.md) |
| `src/test_helpers.h/.cpp` | Random-noise generation, outlier simulation, anchor perturbation, result aggregation, and formatted output helpers. | [Test helpers](test-helpers.md) |
| `libs/eigen-5.0.0/` | Vendored Eigen dependency, including unsupported modules needed by the Levenberg-Marquardt solver. | [Dependencies and vendored libraries](dependencies.md) |
| Project roadmap and known issues | Future improvements, incomplete features, critical bugs, and notes for maintainers. | [Future plans and known issues](future-plans.md) |

## High-level execution flow

1. `main.cpp` creates a `TrueRangeMultilateration::TestParameters` object with a true target position, eight 3D anchors, Gaussian range noise, outlier settings, seed, and number of Monte Carlo runs.
2. `runTests` performs small validation checks for result aggregation and CRLB computation.
3. `runTests` executes three scenario groups: nominal range noise, range noise with anchor-position noise, and range noise with range outliers.
4. For each scenario, `runTest` repeatedly generates noisy inputs, calls one multilateration method, collects estimates, prints error summaries, and reports total/average runtime.
5. Helper functions compute mean absolute error, signed error, second moment, and covariance-like error statistics.

## Implemented estimation methods

- Ordinary least squares based on the Wikipedia true-range multilateration derivation.
- SVD-backed variant of the ordinary least-squares method.
- Nonlinear least squares using Eigen's Levenberg-Marquardt implementation.
- Robust nonlinear least squares using iteratively reweighted least squares and Cauchy-style weighting.
- Yue Wang LLS-I linear least squares.
- Yue Wang LLS-II-2 linear least squares using the shortest range as reference.
- Yue Wang / Chan-Ho two-step weighted linear least squares.
- Cramer-Rao lower-bound support for 3D range-only localization.

## Quick build and run

```bash
cmake -S . -B build
cmake --build build
./build/bin/main
```

The executable performs the simulation configured in `src/main.cpp` and prints results to standard output.
