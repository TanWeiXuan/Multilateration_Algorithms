#include "tests.h"
#include "test_helpers.h"
#include "methods.h"

#include <iostream>
#include <format>

#include <Eigen/Dense>

void runTest1(const TestParameters& params)
{
    std::cout << "Test 1:\n";
    printTestParams(params);

    std::mt19937_64 rng = makeRandomEngine(params.randomSeed);

    std::vector<Eigen::Vector3d> estimatedPositions;
    estimatedPositions.reserve(params.numRuns);

    for (size_t i = 0; i < params.numRuns; i++)
    {
        std::vector<double> noisyRanges = 
        generateNoisyRanges(params.truePosition, params.anchorPositions, params.rangeNoiseStdDev, rng);

        estimatedPositions.emplace_back(
            ordinaryLeastSquaresWikipedia(params.anchorPositions, noisyRanges)
        );
    }

    std::cout << "\n";

}

// END OF FILE //
