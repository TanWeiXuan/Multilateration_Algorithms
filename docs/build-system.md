# Build System

## Configuration

The CMake 3.15+ project uses C++20 and places native runtime output below the build directory's `bin` folder.

| Option | Default | Effect |
| --- | --- | --- |
| `MULTILAT_BUILD_CLI` | `ON` | Builds the native `main` executable (`multilat_cli` alias). |
| `MULTILAT_BUILD_WEBAPP` | `OFF` | Builds `multilat_web` (`multilat_webapp` alias). Emscripten configuration enables it automatically. |

`multilat_core` contains algorithms, shared simulation types, dispatch, the incremental runner, and test helpers. It links to the vendored `Eigen` and `EigenUnsupported` interface targets. The frontend targets link to this core.

MSVC builds apply `/bigobj` to `multilat_core` because Eigen-heavy translation units can exceed the default COFF section limit.

## Native Build

```bash
cmake -S . -B build -DMULTILAT_BUILD_CLI=ON -DMULTILAT_BUILD_WEBAPP=OFF
cmake --build build --config Release --target main
./build/bin/main
```

Single-configuration generators place the executable at `build/bin/main`. Visual Studio normally uses `build\\bin\\Release\\main.exe` for a Release build.

## Web Build

```bash
git submodule update --init --recursive
emcmake cmake -S . -B build-web -DCMAKE_BUILD_TYPE=Release -DMULTILAT_BUILD_CLI=OFF -DMULTILAT_BUILD_WEBAPP=ON
cmake --build build-web --config Release --target multilat_web
```

Web configuration fails early when Raylib, Dear ImGui, or rlImGui submodule files are absent. Generated `index.html`, `index.js`, and `index.wasm` live together in `build-web/bin`.

## CMake Layout

- Top-level `CMakeLists.txt` defines project options and adds `libs/` and `src/`.
- `libs/CMakeLists.txt` exposes Eigen headers through interface targets.
- `src/CMakeLists.txt` defines `multilat_core`, native CLI, web dependencies, and Emscripten link options.

Never reuse a configured native build directory for Emscripten. See [Troubleshooting](troubleshooting.md) for cache and toolchain problems.
