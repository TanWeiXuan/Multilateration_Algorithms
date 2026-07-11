# Dependencies

## Eigen

Eigen 5.0.0 is vendored under `libs/eigen-5.0.0`. It supplies vector/matrix types, dense decompositions, SVD solvers, eigensolvers, and the unsupported Levenberg-Marquardt implementation.

`libs/CMakeLists.txt` declares:

- `Eigen`, which exposes the main and unsupported Eigen include roots.
- `EigenUnsupported`, which links transitively to `Eigen` for code using unsupported modules.

When updating Eigen, change the vendored directory and CMake include paths together, then run native validation and every estimator because decomposition and unsupported APIs may change.

## Web Submodules

The web frontend uses Git submodules:

| Path | Upstream | Purpose |
| --- | --- | --- |
| `external/raylib` | `raysan5/raylib` | Rendering, input, and browser platform support. |
| `external/imgui` | `ocornut/imgui` | Immediate-mode control panel. |
| `external/rlimgui` | `raylib-extras/rlImGui` | Raylib/ImGui integration. |

Initialize them with:

```bash
git submodule update --init --recursive
```

Dependency updates should be isolated commits that move one submodule pointer at a time when practical. Record the upstream version/commit, build native and web targets, and smoke-test rendering and input before merging.

## Toolchain Requirements

- CMake 3.15 or newer.
- A C++20 compiler and standard library with `std::format`.
- Emscripten for web builds.
- Git submodule support for web dependencies.

The native core and CLI do not link Raylib or ImGui; those dependencies are activated only when `MULTILAT_BUILD_WEBAPP=ON`.
