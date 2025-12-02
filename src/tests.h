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
    // Mean absolute error in each axis
    Eigen::Vector3d meanAbsError = Eigen::Vector3d::Zero();
    // Maximum error in each axis
    Eigen::Vector3d maxError = Eigen::Vector3d::Zero();
    // Error covariance matrix
    Eigen::Matrix3d errorCovariance = Eigen::Matrix3d::Zero();
};

struct PrintOptions {
    // Print mean absolute error
    bool printMeanAbsError = true;
    // Print maximum error
    bool printMaxError = true;
    // Print error covariance matrix
    bool printErrorCovariance = true;
    // Print only diagonal values of covariance matrix (if printErrorCovariance is true)
    bool printCovarianceDiagonalOnly = true;
};

typedef std::function< Eigen::Vector3d(const std::vector<Eigen::Vector3d>&, const std::vector<double>&) > MultilaterationFunction;

void runTests(const TestParameters& params);

void runMultilaterationTest(
    const TestParameters& params, 
    MultilaterationFunction multilaterationMethod
);

// END OF FILE //
