#pragma once

#include <vector>
#include <cstdint>
#include <optional>
#include <functional>
#include <string>

#include <Eigen/Dense>

namespace TrueRangeMultilateration
{

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
    // Mean of the signed error (x, y, z) over all runs
    Eigen::Vector3d meanSignedError = Eigen::Vector3d::Zero();
    // Second moment of the error (E[xxᵀ]); used for computing CRLB
    Eigen::Matrix3d errorSecondMoment = Eigen::Matrix3d::Zero();
    // Error covariance matrix
    Eigen::Matrix3d errorCovariance = Eigen::Matrix3d::Zero();
};

struct CrlbResult {
    // 3×3 Cramer–Rao lower bound matrix (m²)
    Eigen::Matrix3d crlb = Eigen::Matrix3d::Zero();
    // 3×3 Fisher information matrix used to compute the CRLB
    Eigen::Matrix3d fisherInformation = Eigen::Matrix3d::Zero();
    // Whether the bound is valid (e.g. enough anchors and finite noise)
    bool valid = false;
    // Whether a pseudoinverse was used because the FIM was rank deficient
    bool usedPseudoInverse = false;
    // Rank of the Fisher information matrix
    int rank = 0;
    // Warning message if something goes wrong
    std::string warning;
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

typedef std::function< Eigen::Vector3d(const std::vector<Eigen::Vector3d>&, const std::vector<double>&) > 
    MultilaterationMethod;

void runTests(const TestParameters& params);

void runTest(
    const TestParameters& params, 
    MultilaterationMethod multilaterationMethod
);

} // namespace TrueRangeMultilateration

// END OF FILE //
