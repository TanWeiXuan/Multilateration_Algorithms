#pragma once

#include <random>
#include <vector>
#include <optional>
#include <cstdint>

#include <Eigen/Dense>

#include "tests.h"

std::mt19937_64 makeRandomEngine(std::optional<uint64_t> seed);

double generateNoisyRange(
    const Eigen::Vector3d& truePosition,
    const Eigen::Vector3d& anchorPosition,
    double rangeNoiseStdDev,
    std::mt19937_64& rng
);

std::vector<double> generateNoisyRanges(
    const Eigen::Vector3d& truePosition,
    const std::vector<Eigen::Vector3d>& anchorPositions,
    double rangeNoiseStdDev,
    std::mt19937_64& rng
);

void printTestParams(const TestParameters& params);

// END OF FILE //
