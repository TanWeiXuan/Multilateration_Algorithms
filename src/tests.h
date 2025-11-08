#pragma once

#include <vector>
#include <cstdint>
#include <optional>
#include <functional>

#include <Eigen/Dense>

struct TestParameters
{
    Eigen::Vector3d truePosition = Eigen::Vector3d::Zero();
    std::vector<Eigen::Vector3d> anchorPositions = {};
    double rangeNoiseStdDev = 0.0;
    std::optional<uint64_t> randomSeed = std::nullopt;
    size_t numRuns = 1;
};

typedef std::function< Eigen::Vector3d(const std::vector<Eigen::Vector3d>&, const std::vector<double>&) > MultilaterationFunction;

void runTests(const TestParameters& params);

void runTest(
    const TestParameters& params, 
    MultilaterationFunction multilaterationMethod
);

// END OF FILE //
