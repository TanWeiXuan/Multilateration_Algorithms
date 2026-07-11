# Multilateration Algorithms Documentation

This documentation covers the shared C++ multilateration core, native CLI, interactive web application, and the maintenance process that keeps `main` and `pages` synchronized.

## Choose a Starting Point

### Users

1. [Build System](build-system.md) for native and web build commands.
2. [Algorithms](algorithms.md) for implemented estimators and input expectations.
3. [CLI and Default Simulation](main.md) or the [live web application](https://tanweixuan.github.io/Multilateration_Algorithms/).

### Contributors

1. [Contributing](contributing.md) for setup, development loops, and the pull-request checklist.
2. [Architecture](architecture.md) for component boundaries and data flow.
3. [Test Harness](test-harness.md) and [Test Helpers](test-helpers.md) for validation and simulation behavior.

### Maintainers

1. [Branching and Releases](branching-and-releases.md) for the `main`/`pages` contract and deployment procedure.
2. [Web Deployment](web-deployment.md) for Emscripten and GitHub Pages behavior.
3. [Dependencies](dependencies.md), [Troubleshooting](troubleshooting.md), and [Future Plans](future-plans.md).

## Repository Map

| Area | Purpose |
| --- | --- |
| `src/true_range_multilateration_methods.*` | Estimation algorithms and CRLB calculation. |
| `src/core/` | Shared types, algorithm dispatch, and incremental simulation. |
| `src/cli/` and `src/tests.*` | Native launcher, deterministic validation, and benchmark scenarios. |
| `src/web/` | Raylib/ImGui web frontend and Emscripten integration. |
| `libs/` | Vendored Eigen and its CMake interface targets. |
| `external/` | Git submodules used only by the web frontend. |
| `.github/workflows/` | Native validation and Pages build/deployment automation. |

## Quick Native Build

```bash
git submodule update --init --recursive
cmake -S . -B build -DMULTILAT_BUILD_CLI=ON -DMULTILAT_BUILD_WEBAPP=OFF
cmake --build build --config Release --target main
./build/bin/main
```

Use a Debug configuration when relying on the CLI's assertion-based validation checks.
