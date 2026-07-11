# Architecture

## Overview

The repository has one shared C++ core and two frontends. The command-line frontend runs the historical benchmark suite synchronously. The web frontend advances the same simulation logic incrementally so the UI remains responsive.

## Components

| Component | Responsibility |
| --- | --- |
| `src/true_range_multilateration_methods.*` | Estimation algorithms and CRLB calculation. |
| `src/core/simulation_types.h` | Shared algorithm IDs, parameters, results, and display options. This is the canonical home for cross-frontend types. |
| `src/core/algorithm_dispatch.*` | Maps an `AlgorithmId` to the corresponding estimator. |
| `src/core/simulation_runner.*` | Stateful Monte Carlo execution for the web frontend. |
| `src/test_helpers.*` | Measurement generation, aggregation, and console formatting. |
| `src/tests.*` | CLI validation checks and benchmark orchestration. |
| `src/cli/main.cpp` | Native CLI launcher and default scenario. |
| `src/web/*` | Raylib/ImGui application, viewport, platform integration, and Emscripten launcher. |

## Data Flow

1. A frontend creates `TestParameters` and selects an `AlgorithmId`.
2. The dispatcher or CLI harness generates noisy ranges and invokes an estimator.
3. Estimates are compared with `truePosition` and aggregated into `TestResults`.
4. The frontend renders or prints bias, absolute/max error, centered covariance, MSE/second moment, timing, and CRLB information.

The CLI's `runTests` exercises every estimator and includes assertion-based validation. The web app uses `SimulationRunner::step` to bound work per frame.

## Ownership Rules

- Put new reusable estimator behavior in the shared core, never directly in a frontend.
- Add shared configuration or result fields to `simulation_types.h` and update both frontends.
- Keep rendering and input code under `src/web`; it must not become an algorithm dependency.
- Keep `multilat_core` free of Raylib, ImGui, and Emscripten dependencies.
- When adding an algorithm, update `AlgorithmId`, `algorithmDisplayName`, dispatch, CLI coverage, web selection, and algorithm documentation together.

## Build Targets

- `multilat_core`: reusable algorithms and simulation code.
- `main` (`multilat_cli` alias): native CLI and assertion-based test harness.
- `multilat_web` (`multilat_webapp` alias): Emscripten/Raylib/ImGui web application.

See [Build System](build-system.md) for configuration details.
