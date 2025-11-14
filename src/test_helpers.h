#pragma once

#include <random>
#include <vector>
#include <optional>
#include <cstdint>

#include <Eigen/Dense>

#include "tests.h"

/**
 * @brief Creates a random number generator engine.
 * 
 * @param seed Optional seed value. If not provided, uses std::random_device for non-deterministic seed.
 * @return std::mt19937_64 A Mersenne Twister pseudo-random generator with 64-bit output.
 */
std::mt19937_64 makeRandomEngine(std::optional<uint64_t> seed);

/**
 * @brief Generates a noisy range measurement between a true position and an anchor position.
 * 
 * Calculates the Euclidean distance between the true position and anchor position,
 * then adds Gaussian noise with the specified standard deviation.
 * 
 * @param truePosition The actual 3D position of the target.
 * @param anchorPosition The 3D position of the anchor/reference point.
 * @param rangeNoiseStdDev Standard deviation of the Gaussian noise to be added to the range.
 * @param rng Random number generator engine used for noise generation.
 * @return double The noisy range measurement in the same units as the input positions.
 */
double generateNoisyRange(
    const Eigen::Vector3d& truePosition,
    const Eigen::Vector3d& anchorPosition,
    double rangeNoiseStdDev,
    std::mt19937_64& rng
);

/**
 * @brief Generates a noisy range measurement with potential outliers.
 * 
 * Similar to the basic generateNoisyRange, but with added capability to simulate outlier
 * measurements. With probability specified by rangeOutlierRatio, an additional uniform
 * random noise in the range [0, rangeOutlierMagnitude] is added to simulate measurement outliers.
 * 
 * @param truePosition The actual 3D position of the target.
 * @param anchorPosition The 3D position of the anchor/reference point.
 * @param rangeNoiseStdDev Standard deviation of the Gaussian noise to be added to the range.
 * @param rangeOutlierRatio Probability (0.0 to 1.0) that this measurement will be an outlier.
 * @param rangeOutlierMagnitude Maximum magnitude of additional uniform noise for outliers.
 * @param rng Random number generator engine used for noise generation.
 * @return double The noisy range measurement, potentially with outlier noise.
 */
double generateNoisyRange(
    const Eigen::Vector3d& truePosition,
    const Eigen::Vector3d& anchorPosition,
    double rangeNoiseStdDev,
    double rangeOutlierRatio,
    double rangeOutlierMagnitude,
    std::mt19937_64& rng
);

/**
 * @brief Generates noisy range measurements to multiple anchor positions.
 * 
 * Generates a vector of noisy range measurements from the true position to each
 * anchor in the provided list, adding Gaussian noise to each measurement.
 * 
 * @param truePosition The actual 3D position of the target.
 * @param anchorPositions Vector of 3D positions of all anchors/reference points.
 * @param rangeNoiseStdDev Standard deviation of the Gaussian noise to be added to each range.
 * @param rng Random number generator engine used for noise generation.
 * @return std::vector<double> Vector of noisy range measurements corresponding to each anchor.
 */
std::vector<double> generateNoisyRanges(
    const Eigen::Vector3d& truePosition,
    const std::vector<Eigen::Vector3d>& anchorPositions,
    double rangeNoiseStdDev,
    std::mt19937_64& rng
);

/**
 * @brief Generates noisy range measurements to multiple anchors with potential outliers.
 * 
 * Generates a vector of noisy range measurements from the true position to each
 * anchor in the provided list, with Gaussian noise and potential outlier measurements.
 * Each measurement has a probability of being an outlier as specified by rangeOutlierRatio.
 * 
 * @param truePosition The actual 3D position of the target.
 * @param anchorPositions Vector of 3D positions of all anchors/reference points.
 * @param rangeNoiseStdDev Standard deviation of the Gaussian noise to be added to each range.
 * @param rangeOutlierRatio Probability (0.0 to 1.0) that each measurement will be an outlier.
 * @param rangeOutlierMagnitude Maximum magnitude of additional uniform noise for outliers.
 * @param rng Random number generator engine used for noise generation.
 * @return std::vector<double> Vector of noisy range measurements corresponding to each anchor.
 */
std::vector<double> generateNoisyRanges(
    const Eigen::Vector3d& truePosition,
    const std::vector<Eigen::Vector3d>& anchorPositions,
    double rangeNoiseStdDev,
    double rangeOutlierRatio,
    double rangeOutlierMagnitude,
    std::mt19937_64& rng
);

/**
 * @brief Generates noisy range measurements using parameters from a TestParameters struct.
 * 
 * Convenience function that extracts necessary parameters from a TestParameters object
 * to generate noisy range measurements with potential outliers.
 * 
 * @param params TestParameters struct containing true position, anchor positions, noise parameters, and outlier parameters.
 * @param rng Random number generator engine used for noise generation.
 * @return std::vector<double> Vector of noisy range measurements corresponding to each anchor in params.
 */
std::vector<double> generateNoisyRanges(
    const TestParameters& params, 
    std::mt19937_64& rng
);

/**
 * @brief Generates a noisy anchor position by adding Gaussian noise to each coordinate.
 * 
 * Simulates uncertainty in anchor position by adding independent Gaussian noise
 * to each of the three coordinates (x, y, z) of the true anchor position.
 * The noise follows a multivariate normal distribution with a diagonal covariance matrix.
 * 
 * @param trueAnchorPosition The actual 3D position of the anchor.
 * @param anchorPosNoiseStdDev Standard deviation of the Gaussian noise for each coordinate.
 * @param rng Random number generator engine used for noise generation.
 * @return Eigen::Vector3d The noisy anchor position.
 */
Eigen::Vector3d generateNoisyAnchorPosition(
    const Eigen::Vector3d& trueAnchorPosition,
    double anchorPosNoiseStdDev,
    std::mt19937_64& rng
);

/**
 * @brief Generates noisy positions for multiple anchors.
 * 
 * Applies generateNoisyAnchorPosition to each anchor in the provided list,
 * simulating position uncertainty for all anchors using the same noise parameters.
 * 
 * @param trueAnchorPositions Vector of actual 3D positions of all anchors.
 * @param anchorPosNoiseStdDev Standard deviation of the Gaussian noise for each coordinate.
 * @param rng Random number generator engine used for noise generation.
 * @return std::vector<Eigen::Vector3d> Vector of noisy anchor positions.
 */
std::vector<Eigen::Vector3d> generateNoisyAnchorPositions(
    const std::vector<Eigen::Vector3d>& trueAnchorPositions,
    double anchorPosNoiseStdDev,
    std::mt19937_64& rng
);

/**
 * @brief Prints test parameters to standard output in a formatted manner.
 * 
 * Displays comprehensive information about test parameters including true position,
 * anchor positions (up to 8 displayed), noise parameters, outlier parameters,
 * random seed status, and number of runs. Useful for debugging and documenting test configurations.
 * 
 * @param params TestParameters struct containing all test configuration parameters to be printed.
 */
void printTestParams(const TestParameters& params);

/**
 * @brief Computes and prints statistics for position estimation results.
 * 
 * Calculates and displays error statistics comparing estimated positions to the true position,
 * including mean error, maximum error per axis, and error covariance matrix. The statistics
 * are computed across all estimated positions provided.
 * 
 * @param estimatedPositions Vector of estimated 3D positions from multiple test runs.
 * @param params TestParameters struct containing the true position for error calculation.
 */
void computeAndPrintResults(
    const std::vector<Eigen::Vector3d>& estimatedPositions,
    const TestParameters& params
);

// END OF FILE //
