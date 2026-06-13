# Algorithms Module

## Files

- `src/true_range_multilateration_methods.h`
- `src/true_range_multilateration_methods.cpp`

## Purpose

This module implements the core true-range multilateration methods and a Cramer-Rao lower-bound calculation. All public algorithms live in the `TrueRangeMultilateration` namespace and use Eigen vectors/matrices.

## Public API

### `ordinaryLeastSquaresWikipedia`

Solves a linearized true-range multilateration system based on the general derivation described by the Wikipedia true-range multilateration article. It constructs an overdetermined linear system and solves normal equations explicitly.

Important behavior:

- Simple and direct implementation.
- Can fail or become unstable for rank-deficient anchor layouts, such as coplanar anchors.
- Uses explicit `(AᵀA).inverse()`, so it is less numerically robust than SVD-based alternatives.

### `ordinaryLeastSquaresWikipedia2`

Uses the same linearized system as `ordinaryLeastSquaresWikipedia`, but solves it with Eigen's `BDCSVD`.

Important behavior:

- More robust for ill-conditioned or coplanar anchor configurations.
- Used as the initial estimate for nonlinear methods.

### `nonLinearLeastSquaresEigenLevenbergMarquardt`

Uses Eigen's unsupported Levenberg-Marquardt implementation to minimize residuals of the form:

```text
modeled_range(anchor_i, x) - measured_range_i
```

Important behavior:

- Starts from `ordinaryLeastSquaresWikipedia2`.
- Uses numerical differentiation around a local functor.
- Refines position by optimizing the original nonlinear range equations rather than a linearized surrogate.

### `robustNonLinearLeastSquaresEigenLevenbergMarquardt`

Robust nonlinear least squares using an iteratively reweighted least-squares loop.

Important behavior:

- Starts from `ordinaryLeastSquaresWikipedia2`.
- Uses whitened residuals divided by `rangeStdDev`.
- Updates Cauchy-style weights based on residual magnitude.
- Clamps weights to avoid numerical issues.
- Runs up to ten outer reweighting iterations with convergence checks based on LM cost-function change.

### `linearLeastSquaresI_YueWang`

Implements the LLS-I approach from Yue Wang's 2015 localization paper. It augments the unknown state with a dummy range-squared variable and solves a linear least-squares system with `BDCSVD`.

### `linearLeastSquaresII_2_YueWang`

Implements the LLS-II-2 approach from Yue Wang's 2015 localization paper. It chooses the shortest measured range as a reference and subtracts that reference equation from the other equations to remove nonlinear terms.

### `twoStepWeightedLinearLeastSquaresI_YueWang`

Implements a two-step weighted least-squares procedure associated with Yue Wang and the Chan-Ho estimator lineage.

Step 1:

- Builds a weighted version of the LLS-I system.
- Uses range standard deviations to scale rows.
- Solves for an intermediate four-element vector.

Step 2:

- Applies the constraint `R² = x² + y² + z²`.
- Solves a refined system for squared coordinate estimates.
- Restores coordinate signs from the first-step estimate.

### `calculateRangePositionCrlb`

Computes a 3D Cramer-Rao lower-bound result for range-only localization under independent Gaussian range noise.

Returns a `CrlbResult` containing:

- Fisher information matrix.
- CRLB matrix.
- Validity flag.
- Rank information.
- Whether a pseudoinverse was used.
- Warning text for invalid or rank-deficient cases.

## Internal helpers

The implementation file contains anonymous-namespace helpers for squaring values, signs, vector summation, point-rank calculation, and Eigen's Levenberg-Marquardt functor shape.

## Input expectations

Most algorithm functions assume compatible inputs:

- `anchorPositions.size()` should match `ranges.size()`.
- 3D localization generally requires enough anchors with sufficient geometric diversity.
- Range standard deviations should be positive when used for weighting or CRLB calculations.

The current code performs limited runtime validation, so callers should validate input sizes and noise parameters before invoking algorithms in production contexts.
