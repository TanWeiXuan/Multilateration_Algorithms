#include "tests.h"
#include "test_helpers.h"
#include "methods.h"

#include <iostream>
#include <format>

#include <Eigen/Dense>

void runTests(const TestParameters& params)
{
    std::cout << "Running Tests...\n";
    printTestParams(params);

    std::cout << "\nTest 1 (Ordinary Least Squares - Wikipedia Method):\n";
    runTest(params, ordinaryLeastSquaresWikipedia);

    std::cout << "\nTest 2 (Non-Linear Least Squares - Eigen Levenberg-Marquardt):\n";
    runTest(params, nonLinearLeastSquaresEigenLevenbergMarquardt);

    std::cout << "\nAll tests completed.\n";
}

void runTest(
    const TestParameters& params, 
    MultilaterationFunction multilaterationMethod
)
{
    std::mt19937_64 rng = makeRandomEngine(params.randomSeed);

    std::vector<Eigen::Vector3d> estimatedPositions;
    estimatedPositions.reserve(params.numRuns);

    for (size_t i = 0; i < params.numRuns; i++)
    {
        std::vector<double> noisyRanges = 
        generateNoisyRanges(params.truePosition, params.anchorPositions, params.rangeNoiseStdDev, rng);

        estimatedPositions.emplace_back(
            multilaterationMethod(params.anchorPositions, noisyRanges)
        );
    }

    computeAndPrintResults(estimatedPositions, params);
}

// END OF FILE //
