#pragma once

#include <string>
#include <vector>

#include "tests.h"

#include <Eigen/Dense>

#include "core/simulation_types.h"

namespace TrueRangeMultilateration
{

/**
 * @brief Method from https://en.wikipedia.org/wiki/True-range_multilateration#General_Multilateration
 * Uses ordinary least squares to solve the linearised problem
 * @param anchorPositions Position of anchors (Note: solver fails, if anchors are coplanar)
 * @param ranges 
 * @return Eigen::Vector3d Estimated position
 */
Eigen::Vector3d ordinaryLeastSquaresWikipedia(
    const std::vector<Eigen::Vector3d>& anchorPositions,
    const std::vector<double>& ranges
);

/**
 * @brief Method from https://en.wikipedia.org/wiki/True-range_multilateration#General_Multilateration
 * Uses ordinary least squares to solve the linearised problem using Eigen's BDCSVD
 * @param anchorPositions Position of anchors (Works even if anchors are coplanar)
 * @param ranges 
 * @return Eigen::Vector3d Estimated position
 */
Eigen::Vector3d ordinaryLeastSquaresWikipedia2(
    const std::vector<Eigen::Vector3d>& anchorPositions,
    const std::vector<double>& ranges
);

/**
 * @brief Method using Eigen's Levenberg-Marquardt implementation to solve the non-linear least squares problem
 * @param anchorPositions 
 * @param ranges 
 * @return Eigen::Vector3d Estimated position
 */
Eigen::Vector3d nonLinearLeastSquaresEigenLevenbergMarquardt(
    const std::vector<Eigen::Vector3d>& anchorPositions,
    const std::vector<double>& ranges
);

/**
 * @brief Robust method using Eigen's Levenberg-Marquardt implementation to solve the non-linear least squares problem
 * with robust loss functions using an iteratively reweighted least squares approach
 * @param anchorPositions 
 * @param ranges 
 * @param rangeStdDev Standard deviation of the range measurements (used for whitening)
 * @param robustLossParam Parameter for the robust loss function (e.g., for Cauchy loss, this is the scale parameter)
 * @return Eigen::Vector3d Estimated position
 */
Eigen::Vector3d robustNonLinearLeastSquaresEigenLevenbergMarquardt(
    const std::vector<Eigen::Vector3d>& anchorPositions,
    const std::vector<double>& ranges,
    const double rangeStdDev,
    const double robustLossParam
);

/**
 * @brief LLS-I method from "Linear least squares localization in sensor networks" by Yue Wang. (2015)
 * @param anchorPositions 
 * @param ranges 
 * @return Eigen::Vector3d Estimated position
 */
Eigen::Vector3d linearLeastSquaresI_YueWang(
    const std::vector<Eigen::Vector3d>& anchorPositions,
    const std::vector<double>& ranges
);

/**
 * @brief LLS-II-2 method from "Linear least squares localization in sensor networks" by Yue Wang. (2015)
 * @param anchorPositions 
 * @param ranges 
 * @return Eigen::Vector3d Estimated position
 */
Eigen::Vector3d linearLeastSquaresII_2_YueWang(
    const std::vector<Eigen::Vector3d>& anchorPositions,
    const std::vector<double>& ranges
);

/**
 * @brief TS-WLLS-I method from "Linear least squares localization in sensor networks" by Yue Wang (2015),
 * originally developed and proposed in "A Simple and Efficient Estimatorfor Hyperbolic Location" by Y. T. Chan and K. C. Ho (1994)
 * @param anchorPositions 
 * @param ranges 
 * @param rangeStdDevs Standard deviations of the range measurements (NOTE: rangeStdDevs.size() == ranges.size()
 * @return Eigen::Vector3d Estimated position
*/
Eigen::Vector3d twoStepWeightedLinearLeastSquaresI_YueWang(
    const std::vector<Eigen::Vector3d>& anchorPositions,
    const std::vector<double>& ranges,
    const std::vector<double>& rangeStdDevs
);

/**
 * @brief Computes the Cramer-Rao lower bound for 3D true-range multilateration.
 *
 * This source-compatible overload treats the supplied anchor positions as exact.
 * It is equivalent to calling the full-covariance overload with a zero 3N x 3N
 * anchor-position covariance matrix.
 *
 * @param anchorPositions Exact 3D positions of anchors, in metres
 * @param evaluationPosition 3D position where the bound is evaluated
 * @param rangeStdDev Shared standard deviation of independent Gaussian range noise
 * @return CrlbResult containing the Fisher information matrix, CRLB, rank, and any warning
 */
CrlbResult calculateRangePositionCrlb(
    const std::vector<Eigen::Vector3d>& anchorPositions,
    const Eigen::Vector3d& evaluationPosition,
    double rangeStdDev
);

/**
 * @brief Computes the local CRLB with independent isotropic anchor-position uncertainty.
 *
 * Each anchor coordinate is modeled as an independent, zero-mean Gaussian error
 * with the shared standard deviation @p anchorPositionStdDev. A value of zero
 * reproduces the exact-anchor overload.
 *
 * @param anchorPositions Nominal/mean 3D anchor positions, in metres
 * @param evaluationPosition 3D position where the bound is evaluated, in metres
 * @param rangeStdDev Shared standard deviation of independent Gaussian range noise, in metres
 * @param anchorPositionStdDev Shared anchor-coordinate standard deviation, in metres; finite and nonnegative
 * @return CrlbResult containing the Fisher information matrix, CRLB, rank, and any warning
 */
CrlbResult calculateRangePositionCrlb(
    const std::vector<Eigen::Vector3d>& anchorPositions,
    const Eigen::Vector3d& evaluationPosition,
    double rangeStdDev,
    double anchorPositionStdDev
);

/**
 * @brief Computes the local CRLB with a general anchor-position covariance.
 *
 * The anchor perturbations are stacked as [delta_a1; ...; delta_aN]. The
 * corresponding covariance must be a finite, symmetric, positive-semidefinite
 * 3N x 3N matrix in the anchors' global coordinate frame, with units of m^2.
 * Off-diagonal blocks may describe correlations between different anchors.
 * The implementation uses the first-order effective range covariance
 * S = sigma_r^2 I + B C_a B^T and J = U^T S^-1 U without explicitly forming
 * S^-1. See docs/crlb.md for the derivation and limitations.
 *
 * @param anchorPositions Nominal/mean 3D anchor positions, in metres
 * @param evaluationPosition 3D position where the bound is evaluated, in metres
 * @param rangeStdDev Shared standard deviation of independent Gaussian range noise, in metres
 * @param anchorPositionCovariance Full 3N x 3N anchor-position covariance, in square metres
 * @return CrlbResult containing the Fisher information matrix, CRLB, rank, and any warning
 */
CrlbResult calculateRangePositionCrlb(
    const std::vector<Eigen::Vector3d>& anchorPositions,
    const Eigen::Vector3d& evaluationPosition,
    double rangeStdDev,
    const Eigen::MatrixXd& anchorPositionCovariance
);

} // namespace TrueRangeMultilateration


// END OF FILE //
