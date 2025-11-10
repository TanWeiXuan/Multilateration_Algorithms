#include "tests.h"
#include "test_helpers.h"
#include "methods.h"

#include <iostream>
#include <format>

#include <Eigen/Dense>

void runTests(const TestParameters& params)
{
    std::cout << "Running Tests...\n";

    TestParameters testParams = params;
    printTestParams(testParams);

    // No ouliers
    std::cout << std::format("\nTest Set 1 -- Std Dev: {:.2f}m, No Outliers\n", testParams.rangeNoiseStdDev);
    std::cout << "\nTest 1.1 (Ordinary Least Squares - Wikipedia Method):\n";
    runTest(testParams, ordinaryLeastSquaresWikipedia);

    std::cout << "\nTest 1.2 (Non-Linear Least Squares - Eigen Levenberg-Marquardt):\n";
    runTest(testParams, nonLinearLeastSquaresEigenLevenbergMarquardt);

    std::cout << "\nTest 1.3 (Robust Non-Linear Least Squares - Eigen Levenberg-Marquardt):\n";
    auto robustNllsEigenLM = std::bind(robustNonLinearLeastSquaresEigenLevenbergMarquardt,
            std::placeholders::_1, std::placeholders::_2, testParams.rangeNoiseStdDev, 5.0
        );
    runTest(testParams, robustNllsEigenLM);

    testParams.rangeOutlierRatio = 0.1;
    std::cout << std::format("\n\nTest Set 2 -- Std Dev: {:.2f}m, Outliers: {:.1f}%\n", 
        testParams.rangeNoiseStdDev, 
        testParams.rangeOutlierRatio * 100.0
    );

    // With 10% outliers
    std::cout << "\nTest 2.1 (Ordinary Least Squares - Wikipedia Method):\n";
    runTest(testParams, ordinaryLeastSquaresWikipedia);

    std::cout << "\nTest 2.2 (Non-Linear Least Squares - Eigen Levenberg-Marquardt):\n";
    runTest(testParams, nonLinearLeastSquaresEigenLevenbergMarquardt);

    std::cout << "\nTest 2.3 (Robust Non-Linear Least Squares - Eigen Levenberg-Marquardt):\n";
    runTest(testParams, robustNllsEigenLM);

    std::cout << "\nAll tests completed.\n";
}

void runTest(
    const TestParameters& params, 
    MultilaterationFunction multilaterationMethod
)
{
    std::mt19937_64 rng = makeRandomEngine(params.randomSeed);

    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> estimatedPositions;
    estimatedPositions.reserve(params.numRuns);

    for (size_t i = 0; i < params.numRuns; i++)
    {
        std::vector<double> noisyRanges = 
        generateNoisyRanges(params, rng);

        estimatedPositions.emplace_back(
            multilaterationMethod(params.anchorPositions, noisyRanges)
        );
    }

    computeAndPrintResults(estimatedPositions, params);
}

// END OF FILE //
