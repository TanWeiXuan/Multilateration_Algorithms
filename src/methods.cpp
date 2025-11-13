#include "methods.h"

#include <format>
#include <functional>
#include <iostream>

#include <unsupported/Eigen/NonLinearOptimization>

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
        RetType sum = RetType();
        for(const VecType& v : vec)
        {
            sum += fn(v);
        }
        return sum;
    }

    size_t computeRank(const std::vector<Eigen::Vector3d>& points, double tol = 1e-8)
    {
        const size_t N = points.size();
        
        // Edge cases
        if (N <= 1) 
        {
            return 0;
        }
        // Compute centroid
        Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
        for (const auto& p : points) 
        {
            centroid += p;
        }
        centroid /= static_cast<double>(N);

        Eigen::MatrixXd A(N, 3);
        for (size_t i = 0; i < N; ++i) 
        {
            A.row(i) = (points[i] - centroid).transpose();
        }
        
        Eigen::BDCSVD<Eigen::MatrixXd, Eigen::ComputeThinU | Eigen::ComputeThinV> svd(A);
        const Eigen::VectorXd& singularValues = svd.singularValues();
        
        // Count singular values above tolerance
        size_t rank = 0;
        for (Eigen::Index i = 0; i < singularValues.size(); ++i) 
        {
            if (singularValues(i) > tol) {
                rank++;
            }
        }
        
        return rank;
    }

    // Generic functor for Eigen's Levenberg-Marquardt Solver, copied from Eigen's tests
    template <typename Scalar_, int NX = Eigen::Dynamic, int NY = Eigen::Dynamic>
    struct EigenLmFunctor {
        typedef Scalar_ Scalar;
        enum { 
            InputsAtCompileTime = NX, 
            ValuesAtCompileTime = NY 
        };
        typedef Eigen::Matrix<Scalar, InputsAtCompileTime, 1> InputType;
        typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, 1> ValueType;
        typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, InputsAtCompileTime> JacobianType;

        const int m_inputs, m_values;

        EigenLmFunctor() 
        : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime) 
        {
            // empty
        }

        EigenLmFunctor(int inputs, int values) 
        : m_inputs(inputs), m_values(values) 
        {
            // empty
        }

        int inputs() const { return m_inputs; }
        int values() const { return m_values; }

        // you should define that in the subclass :
        // void operator() (const InputType& x, ValueType* v, JacobianType* _j=0) const;
    };

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

    double meanSquaredRange = N_inv * sumOver(ranges, std::function(sq<double>));

    std::function squaredNorm = [](const Eigen::Vector3d& v) { return v.squaredNorm(); };
    double meanSquaredNormAnchorPos = N_inv * sumOver(anchorPositions, squaredNorm);

    Eigen::Vector3d anchorPosCentroid = N_inv * sumOver(anchorPositions);

    for(size_t i = 0; i < N; ++i)
    {
        Eigen::Vector3d p_i = anchorPositions[i];
        double d_i = ranges[i];

        A.row(i) = 2.0 * (anchorPosCentroid - p_i).transpose();
        b(i) = sq(d_i) - meanSquaredRange - p_i.squaredNorm() + meanSquaredNormAnchorPos;
    }

    Eigen::MatrixXd A_T = A.transpose();

    // Solve using pseudo-inverse
    // See https://libeigen.gitlab.io/eigen/docs-nightly/group__LeastSquares.html for better methods to solve overdetermined Ax = b
    Eigen::Vector3d posEstimate = (A_T * A).inverse() * A_T * b;

    return posEstimate;
}

Eigen::Vector3d ordinaryLeastSquaresWikipedia2(
    const std::vector<Eigen::Vector3d>& anchorPositions,
    const std::vector<double>& ranges
)
{
    const size_t N = ranges.size();
    const double N_inv = 1.0 / static_cast<double>(N);

    Eigen::MatrixXd A(N, 3);
    Eigen::VectorXd b(N);

    double meanSquaredRange = N_inv * sumOver(ranges, std::function(sq<double>));

    std::function squaredNorm = [](const Eigen::Vector3d& v) { return v.squaredNorm(); };
    double meanSquaredNormAnchorPos = N_inv * sumOver(anchorPositions, squaredNorm);

    Eigen::Vector3d anchorPosCentroid = N_inv * sumOver(anchorPositions);

    for(size_t i = 0; i < N; ++i)
    {
        Eigen::Vector3d p_i = anchorPositions[i];
        double d_i = ranges[i];

        A.row(i) = 2.0 * (anchorPosCentroid - p_i).transpose();
        b(i) = sq(d_i) - meanSquaredRange - p_i.squaredNorm() + meanSquaredNormAnchorPos;
    }

    // Solve using BDCSVD for better numerical stability, especially when anchors are coplanar
    Eigen::BDCSVD<Eigen::MatrixXd, Eigen::ComputeThinU | Eigen::ComputeThinV> svd(A);
    Eigen::Vector3d posEstimate = svd.solve(b);

    return posEstimate;
}

Eigen::Vector3d nonLinearLeastSquaresEigenLevenbergMarquardt(
    const std::vector<Eigen::Vector3d>& anchorPositions,
    const std::vector<double>& ranges
)
{
    struct MultilaterationFunctor : EigenLmFunctor<double>
    {
        const std::vector<Eigen::Vector3d>& mAnchorPositions;
        const std::vector<double>& mRanges;

        MultilaterationFunctor(
            const std::vector<Eigen::Vector3d>& anchorPositions,
            const std::vector<double>& ranges
        )
        : EigenLmFunctor<double>(3, static_cast<int>(ranges.size())),
          mAnchorPositions(anchorPositions),
          mRanges(ranges)
        {
            // empty
        }

        int operator()(const Eigen::VectorXd& x, Eigen::VectorXd& fvec) const
        {
            const size_t N = mRanges.size();
            for(size_t i = 0; i < N; ++i)
            {
                double modeledRange = (x - mAnchorPositions[i]).norm();
                fvec(i) = modeledRange - mRanges[i];
            }
            return 0;
        }
    };

    // Initial guess
    Eigen::VectorXd posEstimate = ordinaryLeastSquaresWikipedia2(anchorPositions, ranges);

    MultilaterationFunctor functor(anchorPositions, ranges);
    Eigen::NumericalDiff<MultilaterationFunctor> numDiff(functor);
    Eigen::LevenbergMarquardt<Eigen::NumericalDiff<MultilaterationFunctor>, double> lmSolver(numDiff);
    lmSolver.parameters.maxfev = 1000;

    lmSolver.minimize(posEstimate);

    return posEstimate;
}

Eigen::Vector3d robustNonLinearLeastSquaresEigenLevenbergMarquardt(
    const std::vector<Eigen::Vector3d>& anchorPositions,
    const std::vector<double>& ranges,
    const double rangeStdDev,
    const double robustLossParam
)
{
    struct RobustMultilaterationFunctor : EigenLmFunctor<double>
    {
        const std::vector<Eigen::Vector3d>& mAnchorPositions;
        const std::vector<double>& mRanges;
        const double mRobustLossParam;
        const double mRangeStdDev;

        RobustMultilaterationFunctor(
            const std::vector<Eigen::Vector3d>& anchorPositions,
            const std::vector<double>& ranges,
            const double rangeStdDev,
            const double robustLossParam
        )
        : EigenLmFunctor<double>(3, static_cast<int>(ranges.size())),
          mAnchorPositions(anchorPositions),
          mRanges(ranges),
          mRangeStdDev(rangeStdDev),
          mRobustLossParam(robustLossParam)
        {
            // empty
        }

        int operator()(const Eigen::VectorXd& x, Eigen::VectorXd& fvec) const
        {
            const size_t N = mRanges.size();
            for(size_t i = 0; i < N; ++i)
            {
                double modeledRange = (x - mAnchorPositions[i]).norm();
                // Whitened residual
                double whitenedResidual = (modeledRange - mRanges[i]) / mRangeStdDev;
                double u = sq(whitenedResidual);

                // Cauchy weight: w = 1 / (1 + (r/c)^2)
                double c = mRobustLossParam;
                double weight = 1.0 / (1.0 + u / (c * c));
                double sqrtWeight = std::sqrt(std::max(weight, 1e-9));;

                fvec(i) = whitenedResidual;
                fvec(i) *= sqrtWeight;
            }
            return 0;
        }

        int df(const Eigen::VectorXd& x, Eigen::MatrixXd& J) const
        {
            const size_t N = mRanges.size();
            J.resize(N, 3);

            for(size_t i = 0; i < N; ++i)
            {
                double modeledRange = (x - mAnchorPositions[i]).norm();

                if (modeledRange < 1e-8) {
                    J.row(i).setZero();
                    continue;
                }

                double residual = modeledRange - mRanges[i];
                double whitened_residual = residual / mRangeStdDev;
                double u = sq(whitened_residual);

                // Cauchy weight: w = 1 / (1 + (r/c)^2)
                double c = mRobustLossParam;
                double weight = 1.0 / (1.0 + u / (c * c));
                double sqrtWeight = std::sqrt(std::max(weight, 1e-9));

                Eigen::Vector3d drdx = (1.0 / mRangeStdDev) * (x - mAnchorPositions[i]) / modeledRange;
                J.row(i) = drdx;
                J.row(i) *= sqrtWeight;
            }
            return 0;
        }
    };

    // Initial guess
    Eigen::VectorXd posEstimate = ordinaryLeastSquaresWikipedia2(anchorPositions, ranges);

    RobustMultilaterationFunctor functor(anchorPositions, ranges, rangeStdDev, robustLossParam);
    Eigen::LevenbergMarquardt<RobustMultilaterationFunctor, double> lmSolver(functor);
    lmSolver.parameters.maxfev = 10'000;

    lmSolver.minimize(posEstimate);

    return posEstimate;
}

// END OF FILE //
