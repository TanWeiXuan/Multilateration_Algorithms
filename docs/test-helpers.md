# Test Helpers

## Files

- `src/test_helpers.h`
- `src/test_helpers.cpp`

## Purpose

The helper module supports simulation setup, random measurement generation, anchor perturbation, result aggregation, and formatted output.

## Random generation

### `makeRandomEngine`

Creates a `std::mt19937_64` random engine. If a seed is supplied, the engine is deterministic. Otherwise, it is seeded from `std::random_device`.

### Range noise helpers

The `generateNoisyRange` overloads compute the true Euclidean distance between the target and an anchor, then add Gaussian noise. The outlier-aware overload additionally adds a uniformly distributed positive outlier offset with probability `rangeOutlierRatio`.

The `generateNoisyRanges` overloads apply range generation across all anchors and can consume a full `TestParameters` object.

### Anchor noise helpers

The `generateNoisyAnchorPosition` and `generateNoisyAnchorPositions` helpers add independent Gaussian noise to anchor coordinates. They are used by scenario groups that simulate inaccurate anchor placement.

## Result aggregation

### `computeResults`

Computes per-axis errors for all estimated positions relative to `params.truePosition`, then derives:

- Mean signed error.
- Mean absolute error.
- Maximum absolute error.
- Error second moment matrix.
- Error covariance matrix.

### `computeAndPrintResults`

Convenience wrapper that computes `TestResults` and passes them to `printResults`.

## Printing helpers

### `printTestParams`

Prints the active target position, anchor list, range-noise settings, outlier settings, anchor-position noise, random seed, and run count.

### `printResults`

Prints selected result fields based on `PrintOptions`, including covariance diagonal-only output by default.
