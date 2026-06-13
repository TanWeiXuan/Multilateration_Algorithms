# Future Plans, Incomplete Features, Known Issues, and Notes

## Critical bugs / high-priority issues

### `EigenUnsupported` include path target appears misconfigured

`libs/CMakeLists.txt` defines an `EigenUnsupported` target, but the unsupported include directory is added to the `Eigen` target rather than to `EigenUnsupported`. The intended command likely should attach `libs/eigen-5.0.0/unsupported` to `EigenUnsupported`.

Impact:

- Builds may still work if headers are found through other include paths, but the target naming does not match the apparent intent.
- Consumers linking only `EigenUnsupported` should receive the unsupported include directory directly and explicitly.

### Limited input validation in algorithm entry points

Most algorithm functions assume valid, size-compatible inputs. Production callers should guard against:

- `anchorPositions.size() != ranges.size()`.
- Too few anchors.
- Degenerate anchor geometry.
- Non-finite ranges or positions.
- Non-positive standard deviations for robust weighting or CRLB calculations.

## Incomplete features

### No standalone library target

The algorithms are compiled directly into the `main` executable. A reusable library target would make downstream integration and isolated testing easier.

### No dedicated unit-test framework

Current validation is embedded in `runTests` and uses `assert`. A dedicated test target using a framework such as Catch2, doctest, or GoogleTest would improve maintainability and CI integration.

### No command-line interface

Simulation parameters are currently hard-coded in `src/main.cpp`. A CLI could expose target position, anchor layout, noise model, run count, seed, and selected algorithms without requiring recompilation.

### Limited documentation of mathematical derivations

The README and docs summarize algorithm intent, but a deeper mathematical reference would be useful for validating formulas, sign conventions, weighting assumptions, and expected failure modes.

## Future plans

- Split core algorithms into a `multilateration` library target.
- Add a `tests` executable with deterministic unit tests and regression scenarios.
- Add input validation and error-return strategies for public APIs.
- Add benchmark tooling that separates measurement-generation time from solver runtime.
- Add command-line parameter parsing for scenario configuration.
- Add JSON or CSV result export for plotting and downstream analysis.
- Add 2D-specific variants or template the implementation by dimension.
- Add uncertainty reporting beyond CRLB, such as empirical confidence ellipsoids.
- Compare algorithm accuracy against CRLB in the printed summaries.
- Add CI jobs for configure, build, run, and formatting checks.

## Additional notes

- The robust nonlinear method uses an iteratively reweighted approach and Cauchy-style weights. Tuning `robustLossParam` can materially affect outlier performance.
- The default scenario uses anchors above and below the target, which is geometrically stronger than a fully coplanar setup.
- Monte Carlo output depends on random seeding. The default seed is deterministic, which is helpful for regression comparisons.
- The repository mentions a web app on the `pages` branch; that source is not part of this branch's main source tree.
