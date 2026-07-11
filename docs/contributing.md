# Contributing

## Prerequisites

- Git, including submodule support.
- CMake 3.15 or newer.
- A C++20 compiler and standard library with `std::format` support.
- Emscripten only when changing the web application or web build.

## Repository Setup

```bash
git clone --recurse-submodules https://github.com/TanWeiXuan/Multilateration_Algorithms.git
cd Multilateration_Algorithms
git submodule update --init --recursive
```

Create work branches from an up-to-date `main`. Do not branch from or commit directly to `pages`.

```bash
git switch main
git pull --ff-only
git switch -c feature/short-description
```

## Native Development Loop

```bash
cmake -S . -B build -DMULTILAT_BUILD_CLI=ON -DMULTILAT_BUILD_WEBAPP=OFF
cmake --build build --config Release --target main
./build/bin/main
```

For Visual Studio generators, the executable is normally `build\\bin\\Release\\main.exe`. Use a Debug build while developing because the validation suite uses `assert`, which may be disabled in Release builds.

## Web Development Loop

```bash
emcmake cmake -S . -B build-web -DCMAKE_BUILD_TYPE=Release -DMULTILAT_BUILD_CLI=OFF -DMULTILAT_BUILD_WEBAPP=ON
cmake --build build-web --target multilat_web
emrun build-web/bin/index.html
```

See [Web Deployment](web-deployment.md) for deployment behavior and smoke-test expectations.

## Change Expectations

- Keep algorithms and shared simulation behavior independent of the UI.
- Preserve deterministic validation inputs unless a test explicitly covers randomness.
- Validate sizes, geometry, and numeric inputs when extending public algorithms.
- Update relevant documentation whenever targets, options, types, branch policy, or user-visible behavior changes.
- Avoid committing generated build output.

## Pull Request Checklist

- [ ] The branch is based on current `main` and contains no unrelated changes.
- [ ] Submodules are initialized and unchanged unless the dependency update is intentional.
- [ ] A clean native configure, build, and CLI run succeeds.
- [ ] Web-impacting changes complete a clean Emscripten build and browser smoke test.
- [ ] New behavior has deterministic validation or regression coverage.
- [ ] Documentation links and commands remain correct.
- [ ] The pull request targets `main`, not `pages`.

For repository topology and release responsibilities, read [Architecture](architecture.md) and [Branching and Releases](branching-and-releases.md).
