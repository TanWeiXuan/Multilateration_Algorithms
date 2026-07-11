#pragma once

#include <vector>

#include <Eigen/Dense>

#include "simulation_types.h"

namespace TrueRangeMultilateration {

Eigen::Vector3d runAlgorithm(
    AlgorithmId algorithm,
    const std::vector<Eigen::Vector3d>& anchorPositions,
    const std::vector<double>& ranges,
    double rangeNoiseStdDev,
    double robustLossParam = 5.0
);

}  // namespace TrueRangeMultilateration
