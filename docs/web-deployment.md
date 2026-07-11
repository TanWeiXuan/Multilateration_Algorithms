# Web Build and Deployment

## Local Build

Initialize the Raylib, Dear ImGui, and rlImGui submodules and activate Emscripten:

```bash
git submodule update --init --recursive
emcmake cmake -S . -B build-web -DCMAKE_BUILD_TYPE=Release -DMULTILAT_BUILD_CLI=OFF -DMULTILAT_BUILD_WEBAPP=ON
cmake --build build-web --config Release --target multilat_web
```

The `multilat_web` target produces `index.html`, `index.js`, and `index.wasm` in `build-web/bin`. The GitHub workflow verifies these artifacts and stages them in `dist-pages`.

Serve the generated files over HTTP; opening `index.html` directly may prevent WebAssembly loading:

```bash
emrun build-web/bin/index.html
```

## Build Composition

- `src/web/web_main.cpp` owns the Emscripten main loop.
- `src/web/emscripten_shell.html` replaces Emscripten's default page shell.
- Raylib provides rendering and window/input abstraction.
- Dear ImGui and rlImGui provide the control panel.
- `multilat_core` supplies algorithms and incremental simulation without depending on the UI stack.

## GitHub Pages Workflow

`.github/workflows/pages-webapp.yml` builds the web target:

- On pull requests targeting `main`, it validates the web build without uploading or deploying.
- On pushes to `pages`, it builds, verifies, uploads, and deploys the Pages artifact.
- A manual run deploys only when invoked from `pages`.

The workflow checks that the HTML, JavaScript, and WebAssembly files exist and are non-empty before upload.

## Release Procedure

Only deploy a commit that has already passed checks on `main`. Fast-forward `pages` according to [Branching and Releases](branching-and-releases.md), watch the `pages-webapp` workflow, then verify:

- The deployment job reports the expected commit.
- The public URL loads without console or WebAssembly errors.
- Algorithm selection and parameter editing work.
- A simulation completes and renders estimates and statistics.
- Mouse/touch pan and zoom still work at representative desktop and mobile sizes.

For failures, see [Troubleshooting](troubleshooting.md).
