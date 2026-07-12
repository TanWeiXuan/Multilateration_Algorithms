# Cramer-Rao Bound with Anchor-Position Uncertainty

This project computes a local first-order Cramer-Rao lower bound (CRLB) for a 3D true-range position estimate. It supports exact anchors, independent isotropic anchor-coordinate uncertainty, and a full covariance with correlations between anchors.

## Exact-Anchor Model

Let the tag position be $x \in \mathbb{R}^3$ and let anchor $i$ have position $a_i \in \mathbb{R}^3$. Its range measurement is

$$
z_i = \lVert x-a_i \rVert + v_i,
$$

where the current shared range-noise API assumes

$$
v \sim \mathcal{N}(0,R), \qquad R=\sigma_r^2 I.
$$

At the evaluation position, define the anchor-to-tag unit vector

$$
u_i = \frac{x-a_i}{\lVert x-a_i \rVert}
$$

and stack its transpose into the rows of $U \in \mathbb{R}^{N \times 3}$. The range Jacobian with respect to the tag position is $\partial r_i/\partial x=u_i^\top$. With exact anchors, the Fisher information is

$$
J_x = U^\top R^{-1}U = \frac{1}{\sigma_r^2}U^\top U.
$$

For full-rank geometry, the position covariance of an unbiased estimator satisfies

$$
\operatorname{Cov}(\hat{x}) \succeq \operatorname{CRLB}_x = J_x^{-1}.
$$

## Uncertain-Anchor Model

Now let $\bar a_i$ be the nominal or mean anchor coordinate and model the true coordinate as

$$
a_i=\bar a_i+\delta a_i.
$$

Stack all anchor perturbations as

$$
\delta a=
\begin{bmatrix}
\delta a_1\\
\vdots\\
\delta a_N
\end{bmatrix}
\sim \mathcal{N}(0,C_a),
\qquad C_a \in \mathbb{R}^{3N \times 3N}.
$$

The unit vectors are evaluated using $x$ and the nominal anchors $\bar a_i$. The range Jacobian with respect to anchor $i$ is

$$
\frac{\partial r_i}{\partial a_i}=-u_i^\top.
$$

Define $B \in \mathbb{R}^{N \times 3N}$ so row $i$ is zero except for columns $3i$ through $3i+2$, which contain $-u_i^\top$. The local linearized model is

$$
\delta z \approx U\,\delta x+B\,\delta a+v.
$$

The matrices and units are:

| Symbol | Dimensions | Meaning and units |
| --- | ---: | --- |
| $U$ | $N \times 3$ | Tag-position range Jacobian; dimensionless |
| $B$ | $N \times 3N$ | Stacked anchor-position range Jacobian; dimensionless |
| $R$ | $N \times N$ | Range covariance, in m$^2$ |
| $C_a$ | $3N \times 3N$ | Anchor-coordinate covariance in the global anchor frame, in m$^2$ |
| $S$ | $N \times N$ | Effective range covariance, in m$^2$ |

Marginalizing the Gaussian anchor perturbation gives

$$
S=R+BC_aB^\top
$$

and the equivalent Fisher information for the tag position is

$$
\boxed{J_x=U^\top S^{-1}U}.
$$

The implementation does not form $S^{-1}$. It solves $SX=U$ with a symmetric Cholesky factorization and computes $J_x=U^\top X$.

### Nuisance-Parameter and Schur-Complement View

Anchor perturbations can instead be treated as uncertain nuisance parameters. When $C_a$ is nonsingular, combining the measurement information with the Gaussian anchor prior gives the joint information blocks

$$
J_{xx}=U^\top R^{-1}U,
$$

$$
J_{xa}=U^\top R^{-1}B,
$$

$$
J_{aa}=B^\top R^{-1}B+C_a^{-1}.
$$

Eliminating the nuisance parameters produces the Schur complement

$$
J_x^{\mathrm{eff}}=J_{xx}-J_{xa}J_{aa}^{-1}J_{ax}.
$$

The Woodbury identity gives

$$
(R+BC_aB^\top)^{-1}
=R^{-1}-R^{-1}B(C_a^{-1}+B^\top R^{-1}B)^{-1}B^\top R^{-1},
$$

so the Schur-complement expression is exactly $U^\top S^{-1}U$. The covariance form remains directly usable for positive-semidefinite, singular $C_a$; it is also obtained as the continuous limit of nonsingular covariances.

## Independent Anchors and Line-of-Sight Projection

For independent anchor errors,

$$
C_a=\operatorname{blkdiag}(\Sigma_{a_1},\ldots,\Sigma_{a_N}),
$$

and the effective range covariance is diagonal. Therefore

$$
\boxed{
J_x=\sum_{i=1}^{N}
\frac{u_i u_i^\top}
{\sigma_r^2+u_i^\top\Sigma_{a_i}u_i}
}.
$$

The scalar $u_i^\top\Sigma_{a_i}u_i$ is the anchor-coordinate variance projected onto the anchor-to-tag line of sight. Uncertainty parallel to the line of sight contributes fully to first-order range variance. Uncertainty perpendicular to it contributes zero at first order, although it can matter through higher-order nonlinear effects.

For independent isotropic anchor uncertainty, $\Sigma_{a_i}=\sigma_a^2I$, so

$$
u_i^\top\Sigma_{a_i}u_i=\sigma_a^2
$$

and

$$
\boxed{
J_x=\frac{1}{\sigma_r^2+\sigma_a^2}
\sum_{i=1}^{N}u_i u_i^\top
}.
$$

This is the model used by the scalar API and web application: every X, Y, and Z anchor coordinate has an independent zero-mean Gaussian error with standard deviation `anchorPosNoiseStdDev`.

## Rank Deficiency and Bound Interpretation

For full-rank $J_x$, `crlb` is the ordinary inverse. If the geometry has an unobservable direction, the true covariance bound is unbounded in that direction. The returned `crlb` then contains an eigenvalue-based pseudoinverse representation only; `usedPseudoInverse` is set, `rank` reports the observable dimension, and `warning` explains the limitation. A zero entry along an unobservable eigendirection must not be interpreted as zero uncertainty.

For a full-rank result:

- `crlb(i, i)` is the lower bound on variance along axis $i$, in $m^2$.
- $\sqrt{\operatorname{crlb}(i,i)}$ is the corresponding per-axis lower-bound standard deviation, in metres.
- A scalar position-error bound can be reported as

  $$
  \mathrm{PEB}=\sqrt{\operatorname{tr}(\operatorname{CRLB})}.
  $$

## C++ API

The exact-anchor overload is source-compatible with the original API:

```cpp
CrlbResult calculateRangePositionCrlb(
    const std::vector<Eigen::Vector3d>& anchorPositions,
    const Eigen::Vector3d& evaluationPosition,
    double rangeStdDev
);
```

The scalar overload models independent isotropic anchor-coordinate errors:

```cpp
const CrlbResult result = calculateRangePositionCrlb(
    anchors,
    evaluationPosition,
    0.05,  // range standard deviation, m
    0.10   // anchor-coordinate standard deviation, m
);
```

The general overload accepts correlations and anisotropic covariance blocks:

```cpp
Eigen::MatrixXd anchorCovariance = Eigen::MatrixXd::Zero(3 * anchors.size(), 3 * anchors.size());
anchorCovariance.block<3, 3>(0, 0) = firstAnchorCovariance;
anchorCovariance.block<3, 3>(0, 3) = crossAnchorCovariance;
anchorCovariance.block<3, 3>(3, 0) = crossAnchorCovariance.transpose();

const CrlbResult result = calculateRangePositionCrlb(
    anchors,
    evaluationPosition,
    0.05,
    anchorCovariance
);
```

`anchorPositionCovariance` must be a finite $3N \times 3N$ matrix in the same global coordinate frame and anchor stacking order as `anchorPositions`. It must be symmetric and positive semidefinite. Symmetry and covariance eigenvalues are checked with the scale-aware tolerance

$$
10^{-10}\max(1,\max_{ij}|C_{a,ij}|).
$$

Asymmetry or negative eigenvalues beyond that tolerance are rejected. Negative eigenvalues within the tolerance are treated as floating-point roundoff and projected to zero. `rangeStdDev` must be finite and strictly positive; the scalar `anchorPositionStdDev` must be finite and nonnegative.

Anchors within $10^{-12}$ metres of the evaluation position are skipped because their range direction is undefined. The result warning reports this condition.

## Simulation Meaning

In `TestParameters`, `anchorPositions` are both the true physical positions used to generate ranges and the mean surveyed layout. For each Monte Carlo estimate, `anchorPosNoiseStdDev` independently perturbs the X, Y, and Z coordinates supplied to the estimator. Ranges are still generated from the unperturbed physical anchors. Anchor noise therefore represents coordinate or survey error, not physical anchor motion that also changes the measured ranges.

The CRLB treats the anchor uncertainty as static during one position estimate. Repeated simulation runs draw independent survey-error realizations to characterize performance over that uncertainty distribution.

## Assumptions and Limitations

This is an equivalent or hybrid local information bound obtained by treating anchor coordinates as uncertain nuisance quantities. It is not an exact nonlinear Bayesian CRLB for arbitrarily large anchor uncertainty. In particular, it assumes:

- zero-mean Gaussian range noise;
- a known range-noise standard deviation shared by all measurements;
- Gaussian anchor-coordinate errors with known covariance;
- range noise independent of anchor-coordinate errors;
- unbiased estimators and the standard CRLB regularity assumptions;
- local linearization at the evaluation position and nominal anchor means;
- anchor errors small enough for the first-order approximation to be meaningful;
- no range outliers, non-line-of-sight bias, or other systematic range bias;
- no uncertainty in the evaluation position used to calculate the bound; and
- static anchor uncertainty during a single estimate.

Correlated range noise and per-measurement range variances are not exposed by the current shared range-noise API. Robust estimators operating under outliers are not claimed to attain this Gaussian CRLB.
