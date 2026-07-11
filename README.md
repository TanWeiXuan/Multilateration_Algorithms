# Multilateration Algorithms

A collection of [true-range multilateration](https://en.wikipedia.org/wiki/True-range_multilateration) algorithms implemented in C++20 with Eigen.

True-range multilateration estimates an unknown position from measured distances to known anchor positions. This repository provides reusable algorithm and simulation code, a command-line test harness, and an interactive web application.

## Implemented Methods

- `ordinaryLeastSquaresWikipedia`: linearized ordinary least squares. It requires non-coplanar anchors.
- `ordinaryLeastSquaresWikipedia2`: an SVD-backed variant that also supports coplanar layouts.
- `nonLinearLeastSquaresEigenLevenbergMarquardt`: nonlinear refinement with Eigen's Levenberg-Marquardt solver.
- `robustNonLinearLeastSquaresEigenLevenbergMarquardt`: iteratively reweighted nonlinear least squares using Cauchy-style weights.
- `linearLeastSquaresI_YueWang`: the LLS-I method described by Yue Wang (2015).
- `linearLeastSquaresII_2_YueWang`: the shortest-range-reference LLS-II-2 method described by Yue Wang (2015).
- `twoStepWeightedLinearLeastSquaresI_YueWang`: the Yue Wang / Chan-Ho two-step weighted estimator.
- `calculateRangePositionCrlb`: Fisher-information and Cramer-Rao lower-bound analysis, including rank-deficient geometry reporting.

See [Algorithms](docs/algorithms.md) for signatures and input expectations.

## Web App

Use the [interactive web application](https://tanweixuan.github.io/Multilateration_Algorithms/) to compare algorithms with configurable anchors, noise, and outliers. Web source lives alongside the CLI source on `main`; `pages` is only the deployment branch.

## Build and Run

Clone with submodules, then build the native CLI:

```bash
git clone --recurse-submodules https://github.com/TanWeiXuan/Multilateration_Algorithms.git
cd Multilateration_Algorithms
cmake -S . -B build -DMULTILAT_BUILD_CLI=ON -DMULTILAT_BUILD_WEBAPP=OFF
cmake --build build --config Release
./build/bin/main
```

On Windows with a multi-configuration generator, run `build\\bin\\Release\\main.exe`.

Web builds require Emscripten:

```bash
emcmake cmake -S . -B build-web -DMULTILAT_BUILD_CLI=OFF -DMULTILAT_BUILD_WEBAPP=ON
cmake --build build-web --config Release --target multilat_web
```

See [Build System](docs/build-system.md) and [Web Deployment](docs/web-deployment.md) for details.

## Documentation and Contributing

Start with the [documentation index](docs/index.md). New contributors should read the [contribution guide](docs/contributing.md). Maintainers should also review the [branching and release process](docs/branching-and-releases.md), [web deployment guide](docs/web-deployment.md), and [troubleshooting guide](docs/troubleshooting.md).

## License

See [LICENSE](LICENSE).
