#pragma once

#include <vector>
#include <cstdint>
#include <optional>
#include <functional>

#include <Eigen/Dense>

struct TestParameters {
    // Position of the target to be estimated
    Eigen::Vector3d truePosition = Eigen::Vector3d::Zero();

    // Positions of the anchors
    std::vector<Eigen::Vector3d> anchorPositions = {};

    // Standard deviation of the range noise
    double rangeNoiseStdDev = 0.0;

    // Ratio of outlier ranges
    double rangeOutlierRatio = 0.0;

    // Outlier ranges are simulated using a uniform distribution of values between [0, rangeOutlierMagnitude]
    double rangeOutlierMagnitude = 0.0;

    // Standard deviation of the anchor position noise
    double anchorPosNoiseStdDev = 0.0;

    // Random seed for noise and outliers
    std::optional<uint64_t> randomSeed = std::nullopt;

    // Number of runs per test
    size_t numRuns = 1;
};

struct TestResults {
    // Mean error in each axis
    Eigen::Vector3d meanError = Eigen::Vector3d::Zero();

    // Maximum error in each axis
    Eigen::Vector3d maxError = Eigen::Vector3d::Zero();

    // Error covariance matrix
    Eigen::Matrix3d errorCovariance = Eigen::Matrix3d::Zero();
};

typedef std::function< Eigen::Vector3d(const std::vector<Eigen::Vector3d>&, const std::vector<double>&) > MultilaterationFunction;

void runTests(const TestParameters& params);

void runTest(
    const TestParameters& params, 
    MultilaterationFunction multilaterationMethod
);

// END OF FILE //
