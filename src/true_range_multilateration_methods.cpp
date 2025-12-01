#include "true_range_multilateration_methods.h"

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
        if(vec.empty())
        {
            return RetType();
        }
        RetType sum = fn(vec[0]);
        for(size_t i = 1; i < vec.size(); ++i)
        {
            sum += fn(vec[i]);
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
    struct WeightedMultilaterationFunctor : EigenLmFunctor<double>
    {
        const std::vector<Eigen::Vector3d>& mAnchorPositions;
        const std::vector<double>& mRanges;
        const std::vector<double>& mSqrtWeights;
        const double mRangeStdDev;

        WeightedMultilaterationFunctor(
            const std::vector<Eigen::Vector3d>& anchorPositions,
            const std::vector<double>& ranges,
            const std::vector<double>& sqrtWeights,
            const double rangeStdDev
        )
        : EigenLmFunctor<double>(3, static_cast<int>(ranges.size())),
          mAnchorPositions(anchorPositions),
          mRanges(ranges),
          mSqrtWeights(sqrtWeights),
          mRangeStdDev(rangeStdDev)
        {
            // empty
        }

    int operator()(const Eigen::VectorXd& x, Eigen::VectorXd& fvec) const
    {
        const size_t N = mRanges.size();
        for (size_t i = 0; i < N; ++i)
        {
            double modeledRange = (x - mAnchorPositions[i]).norm();
            double weightedResidual = (modeledRange - mRanges[i]) / mRangeStdDev;
            fvec(i) = mSqrtWeights[i] * weightedResidual;
        }
        return 0;
    }

    int df(const Eigen::VectorXd& x, Eigen::MatrixXd& J) const
    {
        const size_t N = mRanges.size();
        J.resize(N, 3);

        for (size_t i = 0; i < N; ++i)
        {
            double modeledRange = (x - mAnchorPositions[i]).norm();
            if (modeledRange < 1e-12) {
                J.row(i).setZero();
                continue;
            }
            Eigen::Vector3d drdx = (1.0 / mRangeStdDev) * (x - mAnchorPositions[i]) / modeledRange;
            J.row(i) = mSqrtWeights[i] * drdx.transpose();
        }
        return 0;
    }
    };

    // Initial guess
    Eigen::VectorXd posEstimate = ordinaryLeastSquaresWikipedia2(anchorPositions, ranges);

    const size_t N = ranges.size();
    std::vector<double> sqrtWeights(N, 1.0);

    const size_t maxOuterIterations = 10;

    double prevFnorm = std::numeric_limits<double>::max();
    for(size_t iter = 0; iter < maxOuterIterations; ++iter)
    {
        WeightedMultilaterationFunctor functor(anchorPositions, ranges, sqrtWeights, rangeStdDev);
        Eigen::LevenbergMarquardt<WeightedMultilaterationFunctor> lm(functor);
        lm.parameters.maxfev = 1000;
        lm.minimize(posEstimate);

        std::vector<double> residuals(N);
        for (size_t i = 0; i < N; ++i)
        {
            // Compute weights using Cauchy loss function for next iteration
            double modeledRange = (posEstimate - anchorPositions[i]).norm();
            double whitenedResidual = (modeledRange - ranges[i]) / rangeStdDev;
            double r = residuals[i] = whitenedResidual;
            double c = robustLossParam;
            double w = 1.0 / (1.0 + sq(r / c));
            sqrtWeights[i] = std::sqrt(std::max(w, 1e-9)); // Clamp weights to avoid numerical issues
        }

        // Convergence checks
        if(std::abs(lm.fnorm - prevFnorm) < 1e-6) break; // Absolute change in cost function
        if(std::abs((lm.fnorm - prevFnorm) / std::max(prevFnorm, 1e-9)) < 1e-6) break; // Relative change in cost function

        prevFnorm = lm.fnorm;
    }

    return posEstimate;
}

Eigen::Vector3d linearLeastSquaresI_YueWang(
    const std::vector<Eigen::Vector3d>& anchorPositions,
    const std::vector<double>& ranges
)
{
    const size_t N = ranges.size();
    Eigen::MatrixXd A(N, 4);
    Eigen::VectorXd b(N);

    for(size_t i = 0; i < N; ++i)
    {
        Eigen::Vector3d p_i = anchorPositions[i];
        double d_i = ranges[i];

        A(i, 0) = -2.0 * p_i.x();
        A(i, 1) = -2.0 * p_i.y();
        A(i, 2) = -2.0 * p_i.z();
        A(i, 3) = 1.0;

        b(i) = sq(d_i) - p_i.squaredNorm();
    }

    // Solve using BDCSVD for better numerical stability, especially when anchors are coplanar
    Eigen::BDCSVD<Eigen::MatrixXd, Eigen::ComputeThinU | Eigen::ComputeThinV> svd(A);
    Eigen::VectorXd x = svd.solve(b);

    return x.block<3,1>(0,0);
}

Eigen::Vector3d linearLeastSquaresII_2_YueWang(
    const std::vector<Eigen::Vector3d>& anchorPositions,
    const std::vector<double>& ranges
)
{
    const size_t N = ranges.size();
    Eigen::MatrixXd A(N - 1, 3);
    Eigen::VectorXd b(N - 1);

    // Select shorstest range as reference
    size_t refIndex = 0;
    double minRange = ranges[0];
    for(size_t i = 1; i < N; ++i)
    {
        if(ranges[i] < minRange)
        {
            minRange = ranges[i];
            refIndex = i;
        }
    }

    Eigen::Vector3d x_r = anchorPositions[refIndex];
    double d_r = ranges[refIndex];

    for(size_t i = 0, ii = 0; i < N; ++i)
    {
        if(i == refIndex) continue;

        Eigen::Vector3d p_i = anchorPositions[i];
        double d_i = ranges[i];

        A(ii, 0) = 2.0 * (p_i.x() - x_r.x());
        A(ii, 1) = 2.0 * (p_i.y() - x_r.y());
        A(ii, 2) = 2.0 * (p_i.z() - x_r.z());

        b(ii) = sq(d_r) - sq(d_i) - x_r.squaredNorm() + p_i.squaredNorm();

        ++ii;
    }

    // Solve using BDCSVD for better numerical stability, especially when anchors are coplanar
    Eigen::BDCSVD<Eigen::MatrixXd, Eigen::ComputeThinU | Eigen::ComputeThinV> svd(A);
    Eigen::Vector3d posEstimate = svd.solve(b);

    return posEstimate;
}

// END OF FILE //
