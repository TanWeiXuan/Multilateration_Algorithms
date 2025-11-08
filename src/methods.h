#pragma once

#include <vector>

#include <Eigen/Dense>

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
 * @brief Method using Eigen's Levenberg-Marquardt implementation to solve the non-linear least squares problem
 * @param anchorPositions 
 * @param ranges 
 * @return Eigen::Vector3d Estimated position
 */
Eigen::Vector3d nonLinearLeastSquaresEigenLevenbergMarquardt(
    const std::vector<Eigen::Vector3d>& anchorPositions,
    const std::vector<double>& ranges
);

// END OF FILE //
