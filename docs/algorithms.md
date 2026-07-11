# Algorithms

The public estimators are declared in `src/true_range_multilateration_methods.h`, implemented in the corresponding `.cpp`, and contained in `TrueRangeMultilateration`.

## Estimators

### `ordinaryLeastSquaresWikipedia`

Builds a linearized system and solves normal equations explicitly. It is simple but can fail for rank-deficient layouts such as coplanar anchors.

### `ordinaryLeastSquaresWikipedia2`

Solves the linearized system with Eigen `BDCSVD`. It is more tolerant of ill-conditioned or coplanar layouts and supplies the initial estimate for nonlinear methods.

### `nonLinearLeastSquaresEigenLevenbergMarquardt`

Uses Eigen's unsupported Levenberg-Marquardt implementation and numerical differentiation to minimize modeled-minus-measured range residuals.

### `robustNonLinearLeastSquaresEigenLevenbergMarquardt`

Wraps nonlinear least squares in an iteratively reweighted loop with Cauchy-style weights. `rangeStdDev` whitens residuals and `robustLossParam` controls down-weighting.

### `linearLeastSquaresI_YueWang`

Implements Yue Wang's LLS-I formulation by augmenting the unknown state with a range-squared variable and solving with `BDCSVD`.

### `linearLeastSquaresII_2_YueWang`

Uses the shortest measured range as a reference and subtracts its equation from the other range equations.

### `twoStepWeightedLinearLeastSquaresI_YueWang`

First solves a range-standard-deviation-weighted LLS-I system, then applies the constraint `R² = x² + y² + z²` to refine squared coordinate estimates and restore their signs.

## CRLB Analysis

`calculateRangePositionCrlb` computes a local first-order Fisher information matrix and symmetric CRLB. Its overloads support:

- exact anchors through `calculateRangePositionCrlb(anchorPositions, evaluationPosition, rangeStdDev)`;
- independent isotropic anchor-coordinate uncertainty through an additional scalar `anchorPositionStdDev`; and
- anisotropic and correlated anchor errors through a full `3N x 3N` `anchorPositionCovariance`.

The uncertain-anchor overloads use the effective range covariance $S=R+BC_aB^\top$ and information $J_x=U^\top S^{-1}U$. The implementation solves a symmetric linear system instead of explicitly forming an inverse.

- `rangeStdDev` must be positive and finite.
- Scalar anchor-position standard deviation must be finite and nonnegative.
- A full covariance must have the correct dimensions, contain finite values, and be symmetric positive semidefinite within the documented numerical tolerance.
- Anchors at the evaluation position are skipped and reported.
- Full-rank geometry returns the ordinary inverse bound.
- Rank-deficient geometry returns a pseudoinverse representation, sets `usedPseudoInverse`, reports rank and warning text, and notes that the true bound is unbounded in missing directions.
- No usable anchors or eigendecomposition failure returns `valid == false`.

See [Cramer-Rao Bound with Anchor-Position Uncertainty](crlb.md) for the derivation, line-of-sight interpretation, API examples, assumptions, and limitations.

## Input Expectations

Unless noted above, estimators currently expect:

- Matching `anchorPositions` and `ranges` lengths.
- Enough geometrically diverse anchors for the chosen method.
- Finite positions, ranges, and positive weighting standard deviations.

Most estimators do not yet return structured input errors. Production callers must validate inputs before dispatch; see [Future Plans](future-plans.md).

When adding an estimator, update the shared `AlgorithmId`, display-name mapping, dispatcher, CLI scenarios, web selector, deterministic coverage, and this document.
