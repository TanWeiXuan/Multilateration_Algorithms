#pragma once

#include <random>
#include <vector>
#include <optional>
#include <cstdint>

#include <Eigen/Dense>

std::mt19937_64 makeRandomEngine(std::optional<uint64_t> seed);

// END OF FILE //
