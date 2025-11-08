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

    std::vector<double> noisyRanges = 
        generateNoisyRanges(params.truePosition, params.anchorPositions, params.rangeNoiseStdDev, rng);

    Eigen::Vector3d estPos = ordinaryLeastSquaresWikipedia(params.anchorPositions, noisyRanges);

    std::cout << estPos.transpose() << "\n";
}

// END OF FILE //
