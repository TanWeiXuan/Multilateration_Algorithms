#pragma once

#include <random>
#include <vector>
#include <optional>
#include <cstdint>

#include <Eigen/Dense>

#include "tests.h"

std::mt19937_64 makeRandomEngine(std::optional<uint64_t> seed);

void printTestParams(const TestParameters& params);

// END OF FILE //
