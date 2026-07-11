#include "tests.h"
#include "test_helpers.h"
#include "true_range_multilateration_methods.h"
#include "core/simulation_runner.h"

#include <cassert>
#include <chrono>
#include <cmath>
#include <iostream>
#include <format>
#include <limits>

#include <Eigen/Dense>

namespace TrueRangeMultilateration
{


namespace {


void assertApprox(double actual, double expected)
{
    assert(std::abs(actual - expected) < 1e-12);
}

void runComputeResultsValidationTests()
{
    TestParameters params;
    params.truePosition = Eigen::Vector3d::Zero();

    {
        const TestResults results = computeResults({}, params);
        assert(results.meanSignedError.isZero());
        assert(results.errorCovariance.isZero());
        assert(results.errorSecondMoment.isZero());
    }

    {
        const std::vector<Eigen::Vector3d> estimatedPositions = {
            Eigen::Vector3d(1.0, 0.0, 0.0),
            Eigen::Vector3d(3.0, 0.0, 0.0),
        };
        const TestResults results = computeResults(estimatedPositions, params);

        assertApprox(results.meanSignedError.x(), 2.0);
        assertApprox(results.meanAbsError.x(), 2.0);
        assertApprox(results.errorCovariance(0, 0), 1.0);
        assertApprox(results.errorSecondMoment(0, 0), 5.0);
    }

    {
        const std::vector<Eigen::Vector3d> estimatedPositions = {
            Eigen::Vector3d(-1.0, 0.0, 0.0),
            Eigen::Vector3d(1.0, 0.0, 0.0),
        };
        const TestResults results = computeResults(estimatedPositions, params);

        assertApprox(results.meanSignedError.x(), 0.0);
        assertApprox(results.meanAbsError.x(), 1.0);
        assertApprox(results.errorCovariance(0, 0), 1.0);
        assertApprox(results.errorSecondMoment(0, 0), 1.0);
    }

    std::cout << "computeResults validation tests passed.\n" << std::flush;
}

void runCrlbValidationTests()
{
    const std::vector<Eigen::Vector3d> defaultAnchors = {
        Eigen::Vector3d(-5.0, -5.0, 10.0),
        Eigen::Vector3d(-5.0,  5.0, 10.0),
        Eigen::Vector3d( 5.0,  5.0, 10.0),
        Eigen::Vector3d( 5.0, -5.0, 10.0),
        Eigen::Vector3d(-5.0, -5.0,  0.0),
        Eigen::Vector3d(-5.0,  5.0,  0.0),
        Eigen::Vector3d( 5.0,  5.0,  0.0),
        Eigen::Vector3d( 5.0, -5.0,  0.0),
    };
    const Eigen::Vector3d truePosition(0.0, 0.0, 5.0);

    constexpr double rangeStdDev = 0.5;
    const Eigen::Index covarianceDimension = 3 * static_cast<Eigen::Index>(defaultAnchors.size());
    const Eigen::MatrixXd zeroCovariance = Eigen::MatrixXd::Zero(covarianceDimension, covarianceDimension);

    // Backward compatibility: exact-anchor, zero-isotropic, and zero-covariance APIs agree.
    const CrlbResult exactResult = calculateRangePositionCrlb(defaultAnchors, truePosition, rangeStdDev);
    const CrlbResult zeroIsotropicResult = calculateRangePositionCrlb(
        defaultAnchors,
        truePosition,
        rangeStdDev,
        0.0
    );
    const CrlbResult zeroCovarianceResult = calculateRangePositionCrlb(
        defaultAnchors,
        truePosition,
        rangeStdDev,
        zeroCovariance
    );
    assert(exactResult.valid);
    assert(exactResult.rank == 3);
    assert(!exactResult.usedPseudoInverse);
    assert(exactResult.crlb.isApprox(exactResult.crlb.transpose(), 1e-12));
    assert(exactResult.crlb.diagonal().minCoeff() >= -1e-12);
    assert(exactResult.fisherInformation.isApprox(zeroIsotropicResult.fisherInformation, 1e-12));
    assert(exactResult.crlb.isApprox(zeroIsotropicResult.crlb, 1e-12));
    assert(exactResult.fisherInformation.isApprox(zeroCovarianceResult.fisherInformation, 1e-12));
    assert(exactResult.crlb.isApprox(zeroCovarianceResult.crlb, 1e-12));

    // Independent isotropic uncertainty has the closed-form scalar scaling.
    constexpr double anchorPositionStdDev = 0.25;
    const CrlbResult isotropicResult = calculateRangePositionCrlb(
        defaultAnchors,
        truePosition,
        rangeStdDev,
        anchorPositionStdDev
    );
    const double informationScale =
        (rangeStdDev * rangeStdDev)
        / (rangeStdDev * rangeStdDev + anchorPositionStdDev * anchorPositionStdDev);
    const double crlbScale = 1.0 / informationScale;
    assert(isotropicResult.valid);
    assert(isotropicResult.fisherInformation.isApprox(
        informationScale * exactResult.fisherInformation,
        1e-11
    ));
    assert(isotropicResult.crlb.isApprox(crlbScale * exactResult.crlb, 1e-11));

    // Only the line-of-sight projection of an independent anchor covariance contributes.
    const std::vector<Eigen::Vector3d> axisAnchors = {
        Eigen::Vector3d(-1.0,  0.0,  0.0),
        Eigen::Vector3d( 0.0, -1.0,  0.0),
        Eigen::Vector3d( 0.0,  0.0, -1.0),
        Eigen::Vector3d( 1.0,  0.0,  0.0),
        Eigen::Vector3d( 0.0,  1.0,  0.0),
        Eigen::Vector3d( 0.0,  0.0,  1.0),
    };
    Eigen::MatrixXd anisotropicCovariance = Eigen::MatrixXd::Zero(18, 18);
    anisotropicCovariance.block<3, 3>(0, 0) = Eigen::Vector3d(4.0, 0.0, 0.0).asDiagonal();
    anisotropicCovariance.block<3, 3>(3, 3) = Eigen::Vector3d(9.0, 0.0, 0.0).asDiagonal();
    const CrlbResult anisotropicResult = calculateRangePositionCrlb(
        axisAnchors,
        Eigen::Vector3d::Zero(),
        2.0,
        anisotropicCovariance
    );
    Eigen::Matrix3d expectedAnisotropicInformation = Eigen::Matrix3d::Zero();
    expectedAnisotropicInformation.diagonal() << 3.0 / 8.0, 1.0 / 2.0, 1.0 / 2.0;
    assert(anisotropicResult.valid);
    assert(anisotropicResult.fisherInformation.isApprox(expectedAnisotropicInformation, 1e-12));

    // A positive-semidefinite cross-anchor block must produce the general U^T S^-1 U result.
    Eigen::MatrixXd correlatedCovariance = 0.04 * Eigen::MatrixXd::Identity(
        covarianceDimension,
        covarianceDimension
    );
    correlatedCovariance.block<3, 3>(0, 3) = 0.01 * Eigen::Matrix3d::Identity();
    correlatedCovariance.block<3, 3>(3, 0) = 0.01 * Eigen::Matrix3d::Identity();
    Eigen::MatrixXd U(defaultAnchors.size(), 3);
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(defaultAnchors.size(), covarianceDimension);
    for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(defaultAnchors.size()); ++i) {
        const Eigen::Vector3d delta = truePosition - defaultAnchors[static_cast<size_t>(i)];
        const Eigen::Vector3d u = delta.normalized();
        U.row(i) = u.transpose();
        B.block<1, 3>(i, 3 * i) = -u.transpose();
    }
    Eigen::MatrixXd S = rangeStdDev * rangeStdDev
        * Eigen::MatrixXd::Identity(defaultAnchors.size(), defaultAnchors.size())
        + B * correlatedCovariance * B.transpose();
    S = 0.5 * (S + S.transpose());
    const Eigen::Matrix3d expectedCorrelatedInformation = U.transpose() * S.llt().solve(U);
    const CrlbResult correlatedResult = calculateRangePositionCrlb(
        defaultAnchors,
        truePosition,
        rangeStdDev,
        correlatedCovariance
    );
    assert(correlatedResult.valid);
    assert(correlatedResult.fisherInformation.isApprox(expectedCorrelatedInformation, 1e-11));

    // Increasing isotropic anchor uncertainty decreases information and increases the bound.
    const CrlbResult lowerUncertaintyResult = calculateRangePositionCrlb(
        defaultAnchors,
        truePosition,
        rangeStdDev,
        0.1
    );
    const CrlbResult higherUncertaintyResult = calculateRangePositionCrlb(
        defaultAnchors,
        truePosition,
        rangeStdDev,
        0.6
    );
    assert(lowerUncertaintyResult.valid && higherUncertaintyResult.valid);
    assert(higherUncertaintyResult.crlb.trace() > lowerUncertaintyResult.crlb.trace());
    assert(higherUncertaintyResult.fisherInformation.trace()
        < lowerUncertaintyResult.fisherInformation.trace());

    const std::vector<Eigen::Vector3d> insufficientAnchors = {
        Eigen::Vector3d(-5.0, 0.0, 0.0),
        Eigen::Vector3d( 5.0, 0.0, 0.0),
    };
    const CrlbResult degenerateResult = calculateRangePositionCrlb(insufficientAnchors, truePosition, 0.05);
    assert(degenerateResult.valid);
    assert(degenerateResult.usedPseudoInverse);
    assert(degenerateResult.rank < 3);
    assert(!degenerateResult.warning.empty());

    std::vector<Eigen::Vector3d> coincidentAnchors = defaultAnchors;
    coincidentAnchors.push_back(truePosition);
    const CrlbResult coincidentResult = calculateRangePositionCrlb(
        coincidentAnchors,
        truePosition,
        rangeStdDev
    );
    assert(coincidentResult.valid);
    assert(coincidentResult.usedPseudoInverse);
    assert(!coincidentResult.warning.empty());

    const std::vector<Eigen::Vector3d> noUsableAnchors(4, truePosition);
    const CrlbResult noUsableResult = calculateRangePositionCrlb(
        noUsableAnchors,
        truePosition,
        rangeStdDev
    );
    assert(!noUsableResult.valid);
    assert(!noUsableResult.warning.empty());

    const CrlbResult invalidNoiseResult = calculateRangePositionCrlb(defaultAnchors, truePosition, 0.0);
    assert(!invalidNoiseResult.valid);
    assert(!invalidNoiseResult.warning.empty());

    const CrlbResult nonFiniteNoiseResult = calculateRangePositionCrlb(
        defaultAnchors,
        truePosition,
        std::numeric_limits<double>::infinity()
    );
    assert(!nonFiniteNoiseResult.valid);
    assert(!nonFiniteNoiseResult.warning.empty());

    auto assertInvalidCovariance = [&](const Eigen::MatrixXd& covariance) {
        const CrlbResult invalidResult = calculateRangePositionCrlb(
            defaultAnchors,
            truePosition,
            rangeStdDev,
            covariance
        );
        assert(!invalidResult.valid);
        assert(!invalidResult.warning.empty());
    };

    const CrlbResult negativeAnchorStdDevResult = calculateRangePositionCrlb(
        defaultAnchors,
        truePosition,
        rangeStdDev,
        -0.1
    );
    assert(!negativeAnchorStdDevResult.valid);
    assert(!negativeAnchorStdDevResult.warning.empty());

    const CrlbResult infiniteAnchorStdDevResult = calculateRangePositionCrlb(
        defaultAnchors,
        truePosition,
        rangeStdDev,
        std::numeric_limits<double>::infinity()
    );
    assert(!infiniteAnchorStdDevResult.valid);
    assert(!infiniteAnchorStdDevResult.warning.empty());

    assertInvalidCovariance(Eigen::MatrixXd::Zero(covarianceDimension - 1, covarianceDimension - 1));

    Eigen::MatrixXd nanCovariance = zeroCovariance;
    nanCovariance(0, 0) = std::numeric_limits<double>::quiet_NaN();
    assertInvalidCovariance(nanCovariance);

    Eigen::MatrixXd infiniteCovariance = zeroCovariance;
    infiniteCovariance(0, 0) = std::numeric_limits<double>::infinity();
    assertInvalidCovariance(infiniteCovariance);

    Eigen::MatrixXd nonsymmetricCovariance = zeroCovariance;
    nonsymmetricCovariance(0, 1) = 1.0;
    assertInvalidCovariance(nonsymmetricCovariance);

    Eigen::MatrixXd indefiniteCovariance = zeroCovariance;
    indefiniteCovariance(0, 0) = -1.0;
    assertInvalidCovariance(indefiniteCovariance);

    Eigen::MatrixXd roundoffCovariance = zeroCovariance;
    roundoffCovariance(0, 0) = -1e-12;
    assert(calculateRangePositionCrlb(
        defaultAnchors,
        truePosition,
        rangeStdDev,
        roundoffCovariance
    ).valid);

    std::vector<Eigen::Vector3d> nonFiniteAnchors = defaultAnchors;
    nonFiniteAnchors[0].x() = std::numeric_limits<double>::infinity();
    const CrlbResult nonFiniteAnchorResult = calculateRangePositionCrlb(
        nonFiniteAnchors,
        truePosition,
        rangeStdDev
    );
    assert(!nonFiniteAnchorResult.valid);
    assert(!nonFiniteAnchorResult.warning.empty());

    const CrlbResult nonFiniteEvaluationResult = calculateRangePositionCrlb(
        defaultAnchors,
        Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0),
        rangeStdDev
    );
    assert(!nonFiniteEvaluationResult.valid);
    assert(!nonFiniteEvaluationResult.warning.empty());

    std::cout << "CRLB validation tests passed.\n" << std::flush;
}

void runSimulationAnchorNoiseRegressionTest()
{
    TestParameters params;
    params.truePosition = Eigen::Vector3d(0.25, -0.4, 0.75);
    params.anchorPositions = {
        Eigen::Vector3d(-2.0, -2.0, -1.0),
        Eigen::Vector3d( 2.0, -2.0,  0.5),
        Eigen::Vector3d(-2.0,  2.0,  1.0),
        Eigen::Vector3d( 2.0,  2.0,  2.0),
        Eigen::Vector3d( 0.0,  0.0, -2.0),
    };
    params.rangeNoiseStdDev = 0.01;
    params.anchorPosNoiseStdDev = 0.3;
    params.randomSeed = 123456;
    params.numRuns = 1;
    params.algorithm = AlgorithmId::OrdinaryLeastSquaresWikipedia;

    std::mt19937_64 expectedRng = makeRandomEngine(params.randomSeed);
    const std::vector<double> expectedRanges = generateNoisyRanges(
        params.truePosition,
        params.anchorPositions,
        params.rangeNoiseStdDev,
        params.rangeOutlierRatio,
        params.rangeOutlierMagnitude,
        expectedRng
    );
    const std::vector<Eigen::Vector3d> expectedEstimatorAnchors = generateNoisyAnchorPositions(
        params.anchorPositions,
        params.anchorPosNoiseStdDev,
        expectedRng
    );
    const Eigen::Vector3d expectedEstimate = ordinaryLeastSquaresWikipedia(
        expectedEstimatorAnchors,
        expectedRanges
    );

    bool hasAnchorRangeMismatch = false;
    for (size_t i = 0; i < expectedRanges.size(); ++i) {
        const double solverAnchorRange = (params.truePosition - expectedEstimatorAnchors[i]).norm();
        hasAnchorRangeMismatch = hasAnchorRangeMismatch
            || std::abs(solverAnchorRange - expectedRanges[i]) > 1e-6;
    }
    assert(hasAnchorRangeMismatch);

    SimulationRunner runner;
    runner.begin(params);
    runner.step(1);
    assert(runner.status() == SimulationRunner::Status::Completed);
    assert(runner.estimatedPositions().size() == 1);
    assert(runner.estimatedPositions().front().isApprox(expectedEstimate, 1e-12));

    std::cout << "Simulation anchor-noise regression test passed.\n" << std::flush;
}

} // namespace


void runTests(const TestParameters& params)
{
    std::cout << "Running Tests...\n";
    runCrlbValidationTests();
    runSimulationAnchorNoiseRegressionTest();
    runComputeResultsValidationTests();

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

    std::cout << "\nTest 1.7 (Two-Step Weighted Linear Least Squares - LLS-I from Y. Wang. 2015):\n";
    auto tsWeightedLLSMethod = std::bind(twoStepWeightedLinearLeastSquaresI_YueWang,
            std::placeholders::_1, std::placeholders::_2, std::vector<double>(testParams.anchorPositions.size(), testParams.rangeNoiseStdDev)
        );
    runTest(testParams, tsWeightedLLSMethod);

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

    std::cout << "\nTest 2.7 (Two-Step Weighted Linear Least Squares - LLS-I from Y. Wang. 2015):\n";
    runTest(testParams, tsWeightedLLSMethod);

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

    std::cout << "\nTest 3.7 (Two-Step Weighted Linear Least Squares - LLS-I from Y. Wang. 2015):\n";
    runTest(testParams, tsWeightedLLSMethod);

    std::cout << "\nAll tests completed.\n";
}

void runTest(
    const TestParameters& params, 
    MultilaterationMethod multilaterationMethod
)
{
    std::mt19937_64 rng = makeRandomEngine(params.randomSeed);

    std::vector<Eigen::Vector3d> estimatedPositions;
    estimatedPositions.reserve(params.numRuns);

    auto t0 = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < params.numRuns; i++)
    {
        // Ranges use the physical anchors; only the coordinates supplied to the
        // estimator receive independent survey/coordinate noise.
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

} // namespace TrueRangeMultilateration

// END OF FILE //
