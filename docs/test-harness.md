# Test Harness

## Responsibilities

`src/tests.cpp` orchestrates the native benchmark scenarios and contains small deterministic validation checks. Shared data types now live in `src/core/simulation_types.h`; `src/tests.h` only declares the CLI harness API.

## Shared Types

- `AlgorithmId`: stable frontend-neutral identifiers for estimators.
- `TestParameters`: target, anchors, noise/outlier model, seed, run count, and selected algorithm.
- `TestResults`: mean absolute and signed errors, maximum error, centered covariance, and error second moment/MSE.
- `CrlbResult`: CRLB/Fisher matrices, rank, validity, pseudoinverse flag, and warning.
- `PrintOptions`: controls console result formatting.

## Validation Checks

Before benchmark scenarios, `runTests` checks:

- Empty estimates return zero-initialized results.
- Biased and unbiased samples produce the expected bias, covariance, and MSE.
- Exact-anchor wrappers agree, and isotropic anchor uncertainty produces the expected closed-form scaling.
- Anisotropic line-of-sight projection and correlated cross-anchor covariance produce independently calculated Fisher information.
- Increasing anchor uncertainty reduces information and increases the CRLB trace.
- Rank-deficient geometry reports pseudoinverse use and a warning.
- Invalid range noise, scalar anchor noise, covariance dimensions, finite values, symmetry, and definiteness are rejected.
- A fixed-seed simulation check verifies that ranges use physical anchors while the estimator receives noisy anchor coordinates.

These checks use `assert`; run a Debug build when validation must not be compiled out.

## Scenario Coverage

The CLI executes every estimator with nominal range noise, anchor-position noise, and range outliers. `runTest` generates ranges from `TestParameters::anchorPositions`, treats those as the physical and mean anchor layout, and supplies independently perturbed coordinates to the estimator when anchor noise is enabled. It then stores estimates, aggregates results, and reports total and per-run timing.

The web frontend does not call `runTests`; it uses `SimulationRunner` to execute bounded batches per frame. Changes to shared numerical behavior should be covered in the CLI checks and smoke-tested in the web frontend.

## Adding Coverage

Keep validation deterministic and small. Add regression checks next to the affected behavior, then ensure `runTests` invokes each check exactly once. A future dedicated unit-test target remains desirable; see [Future Plans](future-plans.md).
