# Troubleshooting

## Web Dependencies Are Missing

Symptom: CMake reports missing Raylib, ImGui, or rlImGui files, or a submodule directory is empty.

```bash
git submodule sync --recursive
git submodule update --init --recursive
```

Confirm that `.gitmodules` has entries for all three `external/` dependencies and that Git LFS is not being mistaken for submodule support.

## Wrong Compiler or Toolchain in a Build Directory

CMake caches the compiler and toolchain. Do not reuse a native build directory for Emscripten, or vice versa. Configure native builds in `build` and web builds in `build-web`. If configuration remains stale, remove the affected generated build directory and configure it again.

## `std::format` Is Unavailable

Use a C++20 compiler and standard library that implements `std::format`. Confirm the selected compiler in the first CMake configure output. On Windows, prefer a recent Visual Studio toolchain; on Linux, use a recent GCC/libstdc++ or Clang/libc++ combination.

## Eigen or Unsupported Eigen Headers Are Missing

Confirm that `libs/eigen-5.0.0` exists and that configuration runs from the repository root. The project links `multilat_core` to the `Eigen` and `EigenUnsupported` interface targets declared in `libs/CMakeLists.txt`.

## Web Artifacts Are Missing

Build the explicit target:

```bash
cmake --build build-web --config Release --target multilat_web
```

Search below `build-web` for `index.html`. Its containing directory must also contain non-empty `index.js` and `index.wasm`. Reconfigure with `emcmake` and `MULTILAT_BUILD_WEBAPP=ON` if the target is absent.

## The Browser Cannot Load WebAssembly

Serve the artifact directory over HTTP with `emrun` or another static server. Check the browser console for missing `.js` or `.wasm` files and verify that all three artifacts were copied together.

## GitHub Pages Is Stale

Check that the desired commit is on `main`, that `pages` was fast-forwarded to it, and that the `pages-webapp` workflow ran for the `pages` push. Compare pointers with:

```bash
git fetch origin
git rev-parse origin/main origin/pages
git merge-base --is-ancestor origin/pages origin/main
```

If the workflow succeeded but cached content persists, reload without cache and confirm the deployment URL and commit in the Actions environment.

## CLI Validation Fails Only in Debug

The CLI embeds assertion-based validation. A Debug failure may be hidden in Release when `NDEBUG` disables assertions. Treat the Debug failure as real; reproduce it with the deterministic default seed and inspect the first failed validation before running the larger benchmark suite.
