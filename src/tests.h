#pragma once

#include <functional>
#include <vector>

#include <Eigen/Dense>

#include "core/simulation_types.h"

namespace TrueRangeMultilateration {

typedef std::function<Eigen::Vector3d(const std::vector<Eigen::Vector3d>&, const std::vector<double>&)>
    MultilaterationMethod;

void runTests(const TestParameters& params);

void runTest(const TestParameters& params, MultilaterationMethod multilaterationMethod);

}  // namespace TrueRangeMultilateration

// END OF FILE //
