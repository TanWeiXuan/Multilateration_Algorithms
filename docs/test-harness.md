# Test Harness

## Files

- `src/tests.h`
- `src/tests.cpp`

## Purpose

The test harness defines shared test data structures and orchestrates the repository's simulation-based comparisons.

## Core types

### `TestParameters`

Configuration for one simulation run group:

- `truePosition`: target position to estimate.
- `anchorPositions`: known anchor positions.
- `rangeNoiseStdDev`: Gaussian range-noise standard deviation.
- `rangeOutlierRatio`: probability that a generated range receives an outlier offset.
- `rangeOutlierMagnitude`: maximum uniform outlier magnitude.
- `anchorPosNoiseStdDev`: Gaussian noise applied to anchor positions during tests.
- `randomSeed`: optional seed for reproducible random generation.
- `numRuns`: Monte Carlo sample count per algorithm.

### `TestResults`

Aggregated error statistics:

- Mean absolute error per axis.
- Maximum absolute error per axis.
- Mean signed error per axis.
- Error second moment matrix.
- Error covariance matrix.

### `CrlbResult`

Result type for lower-bound analysis:

- CRLB matrix.
- Fisher information matrix.
- Validity and pseudoinverse flags.
- Fisher matrix rank.
- Warning text.

### `PrintOptions`

Controls which result summaries are printed.

### `MultilaterationMethod`

A callable type alias for algorithms that accept anchor positions and ranges and return an `Eigen::Vector3d` estimate.

## Validation checks

Before running benchmark-style scenarios, `runTests` executes small validation checks for:

- Result aggregation in `computeResults`.
- Basic CRLB validity for a nondegenerate four-anchor configuration.

These checks use `assert`, so they are disabled if the binary is compiled with `NDEBUG`.

## Scenario groups

`runTests` executes each implemented algorithm across three scenario groups:

1. Range noise without outliers.
2. Range noise plus anchor-position noise.
3. Range noise plus range outliers.

Each scenario prints per-algorithm result summaries and timing information.

## Timing behavior

`runTest` times only the repeated generation of noisy inputs, algorithm calls, and estimate storage for a single algorithm/scenario pair. It then prints total elapsed time and average time per run.
