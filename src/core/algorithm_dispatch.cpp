#include "algorithm_dispatch.h"

#include <stdexcept>

#include "../true_range_multilateration_methods.h"

namespace TrueRangeMultilateration {

std::string algorithmDisplayName(const AlgorithmId algorithm) {
    switch (algorithm) {
        case AlgorithmId::OrdinaryLeastSquaresWikipedia:
            return "Ordinary Least Squares (Wikipedia)";
        case AlgorithmId::OrdinaryLeastSquaresWikipediaBdcsvd:
            return "Ordinary Least Squares (Wikipedia + BDCSVD)";
        case AlgorithmId::NonLinearLeastSquaresEigenLm:
            return "Nonlinear Least Squares (Eigen LM)";
        case AlgorithmId::RobustNonLinearLeastSquaresEigenLm:
            return "Robust Nonlinear Least Squares (Eigen LM + IRLS/Cauchy)";
        case AlgorithmId::LinearLeastSquaresIYueWang:
            return "LLS-I (Yue Wang)";
        case AlgorithmId::LinearLeastSquaresII2YueWang:
            return "LLS-II-2 (Yue Wang)";
        case AlgorithmId::TwoStepWeightedLinearLeastSquaresIYueWang:
            return "Two-Step Weighted LLS-I (Yue Wang / Chan-Ho)";
    }

    return "Unknown";
}

Eigen::Vector3d runAlgorithm(
    const AlgorithmId algorithm,
    const std::vector<Eigen::Vector3d>& anchorPositions,
    const std::vector<double>& ranges,
    const double rangeNoiseStdDev,
    const double robustLossParam
) {
    switch (algorithm) {
        case AlgorithmId::OrdinaryLeastSquaresWikipedia:
            return ordinaryLeastSquaresWikipedia(anchorPositions, ranges);
        case AlgorithmId::OrdinaryLeastSquaresWikipediaBdcsvd:
            return ordinaryLeastSquaresWikipedia2(anchorPositions, ranges);
        case AlgorithmId::NonLinearLeastSquaresEigenLm:
            return nonLinearLeastSquaresEigenLevenbergMarquardt(anchorPositions, ranges);
        case AlgorithmId::RobustNonLinearLeastSquaresEigenLm:
            return robustNonLinearLeastSquaresEigenLevenbergMarquardt(
                anchorPositions, ranges, rangeNoiseStdDev, robustLossParam);
        case AlgorithmId::LinearLeastSquaresIYueWang:
            return linearLeastSquaresI_YueWang(anchorPositions, ranges);
        case AlgorithmId::LinearLeastSquaresII2YueWang:
            return linearLeastSquaresII_2_YueWang(anchorPositions, ranges);
        case AlgorithmId::TwoStepWeightedLinearLeastSquaresIYueWang:
            return twoStepWeightedLinearLeastSquaresI_YueWang(
                anchorPositions,
                ranges,
                std::vector<double>(ranges.size(), rangeNoiseStdDev));
    }

    throw std::runtime_error("Invalid algorithm id");
}

}  // namespace TrueRangeMultilateration
