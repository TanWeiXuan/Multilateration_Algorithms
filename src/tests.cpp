#include "tests.h"
#include "test_helpers.h"
#include "true_range_multilateration_methods.h"

#include <chrono>
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

    std::cout << "\nTest 1.2 (Ordinary Least Squares - Wikipedia Method with BDCSVD):\n";
    runTest(testParams, ordinaryLeastSquaresWikipedia2);

    std::cout << "\nTest 1.3 (Non-Linear Least Squares - Eigen Levenberg-Marquardt):\n";
    runTest(testParams, nonLinearLeastSquaresEigenLevenbergMarquardt);

    std::cout << "\nTest 1.4 (Robust Non-Linear Least Squares - Eigen Levenberg-Marquardt):\n";
    auto robustNllsEigenLM = std::bind(robustNonLinearLeastSquaresEigenLevenbergMarquardt,
            std::placeholders::_1, std::placeholders::_2, testParams.rangeNoiseStdDev, 5.0
        );
    runTest(testParams, robustNllsEigenLM);

    std::cout << "\nTest 1.5 (Linear Least Squares - LLS-I from Y. Wang. 2015):\n";
    runTest(testParams, linearLeastSquaresI_YueWang);

    std::cout << "\nTest 1.6 (Linear Least Squares - LLS-II-2 from Y. Wang. 2015):\n";
    runTest(testParams, linearLeastSquaresII_2_YueWang);

    // Test Set 2: No ranging outliers, but anchor position noise
    testParams.rangeOutlierRatio = 0.0;
    testParams.anchorPosNoiseStdDev = 0.25;
    std::cout << std::format("\n\nTest Set 2 -- Range Std Dev: {:.2f}m, Anchor Position Std Dev: {:.2f}m, No Outliers\n", 
        testParams.rangeNoiseStdDev, 
        testParams.anchorPosNoiseStdDev
    );

    std::cout << "\nTest 2.1 (Ordinary Least Squares - Wikipedia Method):\n";
    runTest(testParams, ordinaryLeastSquaresWikipedia);

    std::cout << "\nTest 2.2 (Ordinary Least Squares - Wikipedia Method with BDCSVD):\n";
    runTest(testParams, ordinaryLeastSquaresWikipedia2);

    std::cout << "\nTest 2.3 (Non-Linear Least Squares - Eigen Levenberg-Marquardt):\n";
    runTest(testParams, nonLinearLeastSquaresEigenLevenbergMarquardt);

    std::cout << "\nTest 2.4 (Robust Non-Linear Least Squares - Eigen Levenberg-Marquardt):\n";
    runTest(testParams, robustNllsEigenLM);

    std::cout << "\nTest 2.5 (Linear Least Squares - LLS-I from Y. Wang. 2015):\n";
    runTest(testParams, linearLeastSquaresI_YueWang);

    std::cout << "\nTest 2.6 (Linear Least Squares - LLS-II-2 from Y. Wang. 2015):\n";
    runTest(testParams, linearLeastSquaresII_2_YueWang);

    // Test Set 3: With ranging outliers
    testParams.rangeOutlierRatio = 0.1;
    testParams.anchorPosNoiseStdDev = 0.0;
    std::cout << std::format("\n\nTest Set 3 -- Std Dev: {:.2f}m, Outliers: {:.1f}%\n", 
        testParams.rangeNoiseStdDev, 
        testParams.rangeOutlierRatio * 100.0
    );

    // With 10% outliers
    std::cout << "\nTest 3.1 (Ordinary Least Squares - Wikipedia Method):\n";
    runTest(testParams, ordinaryLeastSquaresWikipedia);

    std::cout << "\nTest 3.2 (Ordinary Least Squares - Wikipedia Method with BDCSVD):\n";
    runTest(testParams, ordinaryLeastSquaresWikipedia2);

    std::cout << "\nTest 3.3 (Non-Linear Least Squares - Eigen Levenberg-Marquardt):\n";
    runTest(testParams, nonLinearLeastSquaresEigenLevenbergMarquardt);

    std::cout << "\nTest 3.4 (Robust Non-Linear Least Squares - Eigen Levenberg-Marquardt):\n";
    runTest(testParams, robustNllsEigenLM);

    std::cout << "\nTest 3.5 (Linear Least Squares - LLS-I from Y. Wang. 2015):\n";
    runTest(testParams, linearLeastSquaresI_YueWang);

    std::cout << "\nTest 3.6 (Linear Least Squares - LLS-II-2 from Y. Wang. 2015):\n";
    runTest(testParams, linearLeastSquaresII_2_YueWang);

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

    auto t0 = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < params.numRuns; i++)
    {
        // Generate noisy anchor positions if anchor position noise is specified
        std::vector<Eigen::Vector3d> anchorPositions = params.anchorPositions;
        if (params.anchorPosNoiseStdDev > 0.0)
        {
            anchorPositions = generateNoisyAnchorPositions(params.anchorPositions, params.anchorPosNoiseStdDev, rng);
        }

        std::vector<double> noisyRanges = generateNoisyRanges(params, rng);

        estimatedPositions.emplace_back(
            multilaterationMethod(anchorPositions, noisyRanges)
        );
    }
    auto t1 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = t1 - t0;

    computeAndPrintResults(estimatedPositions, params);

    std::cout << std::format("  Total Time for {} runs: {:.3f} ms\n", params.numRuns, elapsed.count());
    std::cout << std::format("  Average Time per run: {:.4f} ms\n", (elapsed.count() * 1000.0) / static_cast<double>(params.numRuns));
}

// END OF FILE //
