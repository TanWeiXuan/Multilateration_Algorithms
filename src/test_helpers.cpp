#include "test_helpers.h"

#include <algorithm>
#include <iostream>
#include <format>

std::mt19937_64 makeRandomEngine(std::optional<uint64_t> seed)
{
    return std::mt19937_64(seed.value_or(std::random_device{}()));
}

double generateNoisyRange(
    const Eigen::Vector3d& truePosition,
    const Eigen::Vector3d& anchorPosition,
    double rangeNoiseStdDev,
    std::mt19937_64& rng
)
{
    std::normal_distribution<double> noiseDist(0.0, rangeNoiseStdDev);
    return (truePosition - anchorPosition).norm() + noiseDist(rng);
}

double generateNoisyRange(
    const Eigen::Vector3d& truePosition,
    const Eigen::Vector3d& anchorPosition,
    double rangeNoiseStdDev,
    double rangeOutlierRatio,
    double rangeOutlierMagnitude,
    std::mt19937_64& rng
)
{
    std::uniform_real_distribution<double> outlierDist(0.0, 1.0);
    std::uniform_real_distribution<double> noiseDist(0.0, rangeOutlierMagnitude);

    if (outlierDist(rng) < rangeOutlierRatio)
    {
        return noiseDist(rng);
    }
    else
    {
        return generateNoisyRange(truePosition, anchorPosition, rangeNoiseStdDev, rng);
    }
}

std::vector<double> generateNoisyRanges(
    const Eigen::Vector3d& truePosition,
    const std::vector<Eigen::Vector3d>& anchorPositions,
    double rangeNoiseStdDev,
    std::mt19937_64& rng
)
{
    std::vector<double> ranges;
    ranges.reserve(anchorPositions.size());

    for(const Eigen::Vector3d& anchorPos : anchorPositions)
    {
        ranges.emplace_back(
            generateNoisyRange(truePosition, anchorPos, rangeNoiseStdDev, rng)
        );
    }

    return ranges;
}

std::vector<double> generateNoisyRanges(
    const Eigen::Vector3d& truePosition,
    const std::vector<Eigen::Vector3d>& anchorPositions,
    double rangeNoiseStdDev,
    double rangeOutlierRatio,
    double rangeOutlierMagnitude,
    std::mt19937_64& rng
)
{
    std::vector<double> ranges;
    ranges.reserve(anchorPositions.size());

    for(const Eigen::Vector3d& anchorPos : anchorPositions)
    {
        ranges.emplace_back(
            generateNoisyRange(truePosition, anchorPos, rangeNoiseStdDev, rangeOutlierRatio, rangeOutlierMagnitude, rng)
        );
    }

    return ranges;
}

std::vector<double> generateNoisyRanges(
    const TestParameters& params, 
    std::mt19937_64& rng
)
{
    return generateNoisyRanges(
        params.truePosition,
        params.anchorPositions,
        params.rangeNoiseStdDev,
        params.rangeOutlierRatio,
        params.rangeOutlierMagnitude,
        rng
    );
}

void printTestParams(const TestParameters& params)
{
    std::cout << "Test Parameters:\n";
    std::cout << std::format("  True Position: [{:.2f}, {:.2f}, {:.2f}]\n",  params.truePosition.x(),  params.truePosition.y(),  params.truePosition.z());

    std::cout << "  Anchor Positions:\n";
    for(size_t i = 0; i < params.anchorPositions.size(); ++i)
    {
        if(i == 8)
        {
            std::cout << "    ... ...\n";
            break;
        }
        const Eigen::Vector3d& pos = params.anchorPositions[i];
        std::cout << std::format("    {}: [{:.2f}, {:.2f}, {:.2f}]\n", i, pos.x(), pos.y(), pos.z());
    }

    std::cout << std::format("  Range Noise Std Dev: {:.2f}\n", params.rangeNoiseStdDev);
    std::cout << std::format("  Range Outlier Ratio: {:.2f}\n", params.rangeOutlierRatio);
    std::cout << std::format("  Range Outlier Magnitude: {:.2f}\n", params.rangeOutlierMagnitude);

    if(params.randomSeed.has_value())
    {
        std::cout << std::format("  Random Seed: {}\n", params.randomSeed.value());
    }
    else
    {
        std::cout << "  Random Seed: Not specified, using std::random_device\n";
    }

    std::cout << std::format("  Number of Runs: {}\n", params.numRuns);
}

void computeAndPrintResults(
    const std::vector<Eigen::Vector3d>& estimatedPositions,
    const TestParameters& params
)
{
    Eigen::Vector3d err = Eigen::Vector3d::Zero();
    Eigen::Vector3d maxErr = Eigen::Vector3d::Zero();
    
    for(const Eigen::Vector3d& estPos : estimatedPositions)
    {
        Eigen::Vector3d diff = estPos - params.truePosition;
        err += diff.cwiseAbs();
        
        // Track maximum error in each axis
        maxErr.x() = std::max(maxErr.x(), std::abs(diff.x()));
        maxErr.y() = std::max(maxErr.y(), std::abs(diff.y()));
        maxErr.z() = std::max(maxErr.z(), std::abs(diff.z()));
    }
    err /= static_cast<double>(estimatedPositions.size());

    Eigen::Matrix3d errCov = Eigen::Matrix3d::Zero();
    for (const auto& estPos : estimatedPositions) {
        Eigen::Vector3d diff = (estPos - params.truePosition) - err;
        errCov += diff * diff.transpose();
    }
    errCov /= static_cast<double>(estimatedPositions.size());

    std::cout << "Results:\n";
    std::cout << std::format("  Mean Error: [{:.2f}, {:.2f}, {:.2f}] (m)\n", err.x(), err.y(), err.z());
    std::cout << std::format("  Max Error: [{:.2f}, {:.2f}, {:.2f}] (m)\n", maxErr.x(), maxErr.y(), maxErr.z());
    std::cout << "  Error Covariance Matrix (m^2):\n";
    std::cout << std::format("    [{:.4f}, {:.4f}, {:.4f}]\n", errCov(0,0), errCov(0,1), errCov(0,2));
    std::cout << std::format("    [{:.4f}, {:.4f}, {:.4f}]\n", errCov(1,0), errCov(1,1), errCov(1,2));
    std::cout << std::format("    [{:.4f}, {:.4f}, {:.4f}]\n", errCov(2,0), errCov(2,1), errCov(2,2));
}

// END OF FILE //
