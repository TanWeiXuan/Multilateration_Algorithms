#pragma once

#include <vector>

#include <Eigen/Dense>

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

} // namespace TrueRangeMultilateration


// END OF FILE //
