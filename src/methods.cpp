#include "methods.h"

#include <functional>
#include <iostream>

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


Eigen::Vector3d ordinaryLeastSquaresWikipedia(
    const std::vector<Eigen::Vector3d>& anchorPositions,
    const std::vector<double>& ranges
)
{
    const size_t N = ranges.size();
    const double N_inv = 1.0 / static_cast<double>(N);

    Eigen::MatrixXd A(N, 3);
    Eigen::VectorXd b(N);

    double squaredMeanRange = N_inv * sumOver(ranges, std::function(sq<double>));

    std::function squaredNorm = [](const Eigen::Vector3d& v) { return v.squaredNorm(); };
    double meanSquaredNormAnchorPos = N_inv * sumOver(anchorPositions, squaredNorm);

    Eigen::Vector3d anchorPosCentroid = N_inv * sumOver(anchorPositions);

    for(size_t i = 0; i < N; ++i)
    {
        Eigen::Vector3d p_i = anchorPositions[i];
        double d_i = ranges[i];

        A.row(i) = 2.0 * (p_i - anchorPosCentroid).transpose();
        b(i) = sq(d_i) - squaredMeanRange - p_i.squaredNorm() + meanSquaredNormAnchorPos;
    }

    Eigen::MatrixXd A_T = A.transpose();

    // Solve using pseudo-inverse
    // See https://libeigen.gitlab.io/eigen/docs-nightly/group__LeastSquares.html for better methods to solve overdetermined Ax = b
    Eigen::Vector3d posEstimate = (A_T * A).inverse() * A_T * b;

    return posEstimate;
}

// END OF FILE //
