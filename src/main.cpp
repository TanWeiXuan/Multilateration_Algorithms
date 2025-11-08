#include <iostream>
#include <format>

#include <Eigen/Dense>

#include "tests.h"

int main(int argc, char const *argv[])
{
    std::cout << std::format("Multilateration Methods.\n");

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
    testParams.numRuns = 10,000;

    runTest1(testParams);

    return 0;
}

// END OF FILE //
