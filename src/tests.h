#pragma once

#include <vector>
#include <cstdint>
#include <optional>

#include <Eigen/Dense>

struct TestParameters
{
    Eigen::Vector3d truePosition = Eigen::Vector3d::Zero();
    std::vector<Eigen::Vector3d> anchorPositions = {};
    double rangeNoiseStdDev = 0.0;
    std::optional<uint64_t> randomSeed = std::nullopt;
};


void runTest1(const TestParameters& params);

// END OF FILE //
