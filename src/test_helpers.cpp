#include "test_helpers.h"

#include <algorithm>
#include <iostream>
#include <format>
#include <random>

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
    std::uniform_real_distribution<double> outlierNoiseDist(0.0, rangeOutlierMagnitude);

    if (outlierDist(rng) < rangeOutlierRatio)
    {
        return generateNoisyRange(truePosition, anchorPosition, rangeNoiseStdDev, rng) + outlierNoiseDist(rng);
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
    const TrueRangeMultilateration::TestParameters& params, 
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


Eigen::Vector3d generateNoisyAnchorPosition(
    const Eigen::Vector3d& trueAnchorPosition,
    double anchorPosNoiseStdDev,
    std::mt19937_64& rng
)
{
    // Generate noise from multivariate normal distribution with diagonal covariance matrix
    // where diagonal elements equal anchorPosNoiseStdDev^2
    std::normal_distribution<double> noiseDist(0.0, anchorPosNoiseStdDev);
    
    Eigen::Vector3d noise(
        noiseDist(rng),
        noiseDist(rng),
        noiseDist(rng)
    );
    
    return trueAnchorPosition + noise;
}


std::vector<Eigen::Vector3d> generateNoisyAnchorPositions(
    const std::vector<Eigen::Vector3d>& trueAnchorPositions,
    double anchorPosNoiseStdDev,
    std::mt19937_64& rng
)
{
    std::vector<Eigen::Vector3d> noisyPositions;
    noisyPositions.reserve(trueAnchorPositions.size());
    
    for(const Eigen::Vector3d& truePos : trueAnchorPositions)
    {
        noisyPositions.emplace_back(
            generateNoisyAnchorPosition(truePos, anchorPosNoiseStdDev, rng)
        );
    }
    
    return noisyPositions;
}


std::vector<Eigen::Vector3d> generateNoisyAnchorPositions(
    const TrueRangeMultilateration::TestParameters& params,
    std::mt19937_64& rng
)
{
    return generateNoisyAnchorPositions(
        params.anchorPositions,
        params.anchorPosNoiseStdDev,
        rng
    );
}


void printTestParams(const TrueRangeMultilateration::TestParameters& params)
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


TrueRangeMultilateration::TestResults computeResults(
    const std::vector<Eigen::Vector3d>& estimatedPositions,
    const TrueRangeMultilateration::TestParameters& params
)
{
    TrueRangeMultilateration::TestResults results;
    
    Eigen::Vector3d err = Eigen::Vector3d::Zero();
    Eigen::Vector3d maxErr = Eigen::Vector3d::Zero();
    
    for(const Eigen::Vector3d& estPos : estimatedPositions)
    {
        Eigen::Vector3d diff = estPos - params.truePosition;
        err += diff.cwiseAbs();
        
        // Track maximum error in each axis
        maxErr = maxErr.cwiseMax(diff.cwiseAbs());
    }
    err /= static_cast<double>(estimatedPositions.size());

    Eigen::Matrix3d errCov = Eigen::Matrix3d::Zero();
    for (const auto& estPos : estimatedPositions) {
        Eigen::Vector3d diff = (estPos - params.truePosition) - err;
        errCov += diff * diff.transpose();
    }
    errCov /= static_cast<double>(estimatedPositions.size());

    results.meanAbsError = err;
    results.maxError = maxErr;
    results.errorCovariance = errCov;

    return results;
}


void printResults(const TrueRangeMultilateration::TestResults& results, const TrueRangeMultilateration::PrintOptions& options)
{
    std::cout << "Results:\n";
    
    if (options.printMeanAbsError)
    {
        std::cout << std::format("  Mean Absolute Error: [{:.2f}, {:.2f}, {:.2f}] (m)\n", 
            results.meanAbsError.x(), results.meanAbsError.y(), results.meanAbsError.z());
    }
    
    if (options.printMaxError)
    {
        std::cout << std::format("  Max Error in Each Axis: [{:.2f}, {:.2f}, {:.2f}] (m)\n", 
            results.maxError.x(), results.maxError.y(), results.maxError.z());
    }
    
    if (options.printErrorCovariance)
    {
        if (options.printCovarianceDiagonalOnly)
        {
            std::cout << std::format("  Error Covariance Diagonal (m^2): [{:.4f}, {:.4f}, {:.4f}]\n", 
                results.errorCovariance(0,0), results.errorCovariance(1,1), results.errorCovariance(2,2));
        }
        else
        {
            std::cout << "  Error Covariance Matrix (m^2):\n";
            std::cout << std::format("    [{:.4f}, {:.4f}, {:.4f}]\n", 
                results.errorCovariance(0,0), results.errorCovariance(0,1), results.errorCovariance(0,2));
            std::cout << std::format("    [{:.4f}, {:.4f}, {:.4f}]\n", 
                results.errorCovariance(1,0), results.errorCovariance(1,1), results.errorCovariance(1,2));
            std::cout << std::format("    [{:.4f}, {:.4f}, {:.4f}]\n", 
                results.errorCovariance(2,0), results.errorCovariance(2,1), results.errorCovariance(2,2));
        }
    }
}


void computeAndPrintResults(
    const std::vector<Eigen::Vector3d>& estimatedPositions,
    const TrueRangeMultilateration::TestParameters& params,
    const TrueRangeMultilateration::PrintOptions& options
)
{
    TrueRangeMultilateration::TestResults results = computeResults(estimatedPositions, params);
    printResults(results, options);
}

// END OF FILE //
