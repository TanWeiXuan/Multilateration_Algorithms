#include "methods.h"

#include <functional>

namespace // anonymous namespace for helper functions
{
    template<typename T>
    T sq(const T& x)
    {
        return x * x;
    }

    template<typename VecType, typename RetType = VecType>
    RetType sumOver(
        const std::vector<VecType>& vec, 
        const std::function<RetType(const VecType&)>& fn = [](const VecType& x){ return x; }
    )
    {
        RetType sum = RetType(0);
        for(const VecType& v : vec)
        {
            sum += fn(v);
        }
        return sum;
    }
} // namespace anonymous

// END OF FILE //
