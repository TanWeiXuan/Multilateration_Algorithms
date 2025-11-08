#include "test_helpers.h"

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

// END OF FILE //
