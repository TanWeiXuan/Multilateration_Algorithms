#include <iostream>
#include <format>

#include <Eigen/Dense>

#include "tests.h"

int main(int argc, char const *argv[])
{
    std::cout << std::format("Multilateration Methods.\n\n");

    TestParameters testParams;
    testParams.truePosition = Eigen::Vector3d(0.0, 0.0, 5.0);
    testParams.anchorPositions = {
        Eigen::Vector3d( -5.0, -5.0, 10.0 ),
        Eigen::Vector3d( -5.0,  5.0, 10.0 ),
        Eigen::Vector3d(  5.0,  5.0, 10.0 ),
        Eigen::Vector3d(  5.0, -5.0, 10.0 ),
        Eigen::Vector3d( -5.0, -5.0, 0.0 ),
        Eigen::Vector3d( -5.0,  5.0, 0.0 ),
        Eigen::Vector3d(  5.0,  5.0, 0.0 ),
        Eigen::Vector3d(  5.0, -5.0, 0.0 )
    };
    testParams.rangeNoiseStdDev = 0.25;
    testParams.randomSeed = 42;
    testParams.numRuns = 1'000'000;
    testParams.rangeOutlierRatio = 0.0;
    testParams.rangeOutlierMagnitude = 100.0;

    runTests(testParams);

    return 0;
}

// END OF FILE //
