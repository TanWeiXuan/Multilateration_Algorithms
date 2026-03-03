#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Dense>

namespace TrueRangeMultilateration {

enum class AlgorithmId {
    OrdinaryLeastSquaresWikipedia = 0,
    OrdinaryLeastSquaresWikipediaBdcsvd,
    NonLinearLeastSquaresEigenLm,
    RobustNonLinearLeastSquaresEigenLm,
    LinearLeastSquaresIYueWang,
    LinearLeastSquaresII2YueWang,
    TwoStepWeightedLinearLeastSquaresIYueWang,
};

struct TestParameters {
    Eigen::Vector3d truePosition = Eigen::Vector3d::Zero();
    std::vector<Eigen::Vector3d> anchorPositions = {};
    double rangeNoiseStdDev = 0.0;
    double rangeOutlierRatio = 0.0;
    double rangeOutlierMagnitude = 0.0;
    double anchorPosNoiseStdDev = 0.0;
    std::optional<uint64_t> randomSeed = std::nullopt;
    size_t numRuns = 1;
    AlgorithmId algorithm = AlgorithmId::OrdinaryLeastSquaresWikipedia;
};

struct TestResults {
    Eigen::Vector3d meanAbsError = Eigen::Vector3d::Zero();
    Eigen::Vector3d maxError = Eigen::Vector3d::Zero();
    Eigen::Matrix3d errorCovariance = Eigen::Matrix3d::Zero();
};

struct PrintOptions {
    bool printMeanAbsError = true;
    bool printMaxError = true;
    bool printErrorCovariance = true;
    bool printCovarianceDiagonalOnly = true;
};

std::string algorithmDisplayName(AlgorithmId algorithm);

}  // namespace TrueRangeMultilateration
