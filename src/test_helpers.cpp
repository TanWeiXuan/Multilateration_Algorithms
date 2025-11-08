#include "test_helpers.h"

std::mt19937_64 makeRandomEngine(std::optional<uint64_t> seed)
{
    return std::mt19937_64(seed.value_or(std::random_device{}()));
}

// END OF FILE //
