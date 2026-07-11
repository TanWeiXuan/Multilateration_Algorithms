#include "true_range_multilateration_methods.h"

#include <algorithm>
#include <cmath>
#include <format>
#include <functional>
#include <iostream>
#include <limits>

#include <unsupported/Eigen/NonLinearOptimization>

namespace // anonymous namespace for helper functions
{
    template<typename T>
    T sq(const T& x)
    {
        return x * x;
    }

    template <typename T>
    int signum(T val) {
        return (T(0) < val) - (val < T(0));
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

namespace TrueRangeMultilateration
{

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

Eigen::Vector3d twoStepWeightedLinearLeastSquaresI_YueWang(
    const std::vector<Eigen::Vector3d>& anchorPositions,
    const std::vector<double>& ranges,
    const std::vector<double>& rangeStdDevs
)
{
    // 1st step: Weighted Linear Least Squares I (Yue Wang)
    const size_t N = ranges.size();
    Eigen::MatrixXd A(N, 4);
    Eigen::VectorXd b(N);

    for(size_t i = 0; i < N; ++i)
    {
        Eigen::Vector3d p_i = anchorPositions[i];
        double d_i = ranges[i];
        double C_i = std::max((4 * sq(d_i) * sq(rangeStdDevs[i])), 1e-9); // Prevent division by zero
        double w_i = 1.0 / std::sqrt(C_i);

        A(i, 0) = -2.0 * p_i.x() * w_i;
        A(i, 1) = -2.0 * p_i.y() * w_i;
        A(i, 2) = -2.0 * p_i.z() * w_i;
        A(i, 3) = 1.0 * w_i;

        b(i) = (sq(d_i) - p_i.squaredNorm()) * w_i;
    }

    Eigen::BDCSVD<Eigen::MatrixXd, Eigen::ComputeThinU | Eigen::ComputeThinV> svd(A);
    Eigen::Vector4d lamda_WLLS = svd.solve(b);

    // 2nd step: Refinement utilising the constraint of the dummy variable, R^2 = x^2 + y^2 + z^2
    Eigen::Matrix4d S = A.transpose() * A; // S == (A^T * C^-1 * A), in the paper

    Eigen::Matrix4d K = Eigen::Matrix4d::Zero();
    K(0, 0) = 2.0 * lamda_WLLS(0);
    K(1, 1) = 2.0 * lamda_WLLS(1);
    K(2, 2) = 2.0 * lamda_WLLS(2);
    K(3, 3) = -1.0;

    Eigen::Matrix<double, 4, 3> G;
    G << 1.0, 0.0, 0.0,
         0.0, 1.0, 0.0,
         0.0, 0.0, 1.0,
         1.0, 1.0, 1.0;

    Eigen::Matrix4d phi = K * S.inverse() * K;
    Eigen::Matrix4d phi_inv = phi.inverse();

    Eigen::Matrix<double, 4, 1> h;
    h << sq(lamda_WLLS(0)),
         sq(lamda_WLLS(1)),
         sq(lamda_WLLS(2)),
         lamda_WLLS(3);

    Eigen::Matrix3d GT_phiInv_G = G.transpose() * phi_inv * G;
    Eigen::Vector3d GT_phiInv_h = G.transpose() * phi_inv * h;

    Eigen::Vector3d z_hat = GT_phiInv_G.ldlt().solve(GT_phiInv_h);

    for(int i = 0; i < 3; ++i)
    {
        if(z_hat(i) < 0.0) z_hat(i) = 0.0; 
    }

    Eigen::Vector3d posEstimate;
    posEstimate << std::sqrt(z_hat(0)) * signum(lamda_WLLS(0)),
                   std::sqrt(z_hat(1)) * signum(lamda_WLLS(1)),
                   std::sqrt(z_hat(2)) * signum(lamda_WLLS(2));

    return posEstimate;
}

CrlbResult calculateRangePositionCrlb(
    const std::vector<Eigen::Vector3d>& anchorPositions,
    const Eigen::Vector3d& evaluationPosition,
    double rangeStdDev
)
{
    return calculateRangePositionCrlb(
        anchorPositions,
        evaluationPosition,
        rangeStdDev,
        0.0
    );
}

CrlbResult calculateRangePositionCrlb(
    const std::vector<Eigen::Vector3d>& anchorPositions,
    const Eigen::Vector3d& evaluationPosition,
    double rangeStdDev,
    double anchorPositionStdDev
)
{
    if (anchorPositionStdDev < 0.0 || !std::isfinite(anchorPositionStdDev)) {
        CrlbResult result;
        result.usedPseudoInverse = true;
        result.warning = "Anchor-position standard deviation must be nonnegative and finite.";
        return result;
    }

    if (anchorPositions.size() > static_cast<size_t>(std::numeric_limits<Eigen::Index>::max() / 3)) {
        CrlbResult result;
        result.usedPseudoInverse = true;
        result.warning = "Too many anchors to construct the anchor-position covariance matrix.";
        return result;
    }

    const Eigen::Index covarianceDimension = 3 * static_cast<Eigen::Index>(anchorPositions.size());
    Eigen::MatrixXd anchorPositionCovariance = Eigen::MatrixXd::Zero(
        covarianceDimension,
        covarianceDimension
    );
    if (anchorPositionStdDev > 0.0) {
        const double anchorVariance = anchorPositionStdDev * anchorPositionStdDev;
        if (!std::isfinite(anchorVariance)) {
            CrlbResult result;
            result.usedPseudoInverse = true;
            result.warning = "Anchor-position variance overflowed; use a smaller standard deviation.";
            return result;
        }
        anchorPositionCovariance.diagonal().setConstant(anchorVariance);
    }

    return calculateRangePositionCrlb(
        anchorPositions,
        evaluationPosition,
        rangeStdDev,
        anchorPositionCovariance
    );
}

CrlbResult calculateRangePositionCrlb(
    const std::vector<Eigen::Vector3d>& anchorPositions,
    const Eigen::Vector3d& evaluationPosition,
    double rangeStdDev,
    const Eigen::MatrixXd& anchorPositionCovariance
)
{
    CrlbResult result;

    auto appendWarning = [&result](const std::string& warning) {
        if (!result.warning.empty()) {
            result.warning += " ";
        }
        result.warning += warning;
    };

    if (rangeStdDev <= 0.0 || !std::isfinite(rangeStdDev)) {
        result.valid = false;
        result.usedPseudoInverse = true;
        appendWarning("Range standard deviation must be positive and finite.");
        return result;
    }

    if (anchorPositions.size() > static_cast<size_t>(std::numeric_limits<Eigen::Index>::max() / 3)) {
        result.usedPseudoInverse = true;
        appendWarning("Too many anchors to validate the anchor-position covariance matrix.");
        return result;
    }

    if (!evaluationPosition.allFinite()) {
        result.usedPseudoInverse = true;
        appendWarning("CRLB evaluation position must contain only finite coordinates.");
        return result;
    }

    for (const Eigen::Vector3d& anchorPosition : anchorPositions) {
        if (!anchorPosition.allFinite()) {
            result.usedPseudoInverse = true;
            appendWarning("Anchor positions must contain only finite coordinates.");
            return result;
        }
    }

    const Eigen::Index anchorCount = static_cast<Eigen::Index>(anchorPositions.size());
    const Eigen::Index covarianceDimension = 3 * anchorCount;
    if (anchorPositionCovariance.rows() != covarianceDimension ||
        anchorPositionCovariance.cols() != covarianceDimension) {
        result.usedPseudoInverse = true;
        appendWarning(std::format(
            "Anchor-position covariance must have dimensions {} x {} for {} anchors.",
            covarianceDimension,
            covarianceDimension,
            anchorCount
        ));
        return result;
    }

    if (!anchorPositionCovariance.allFinite()) {
        result.usedPseudoInverse = true;
        appendWarning("Anchor-position covariance must contain only finite values.");
        return result;
    }

    Eigen::MatrixXd validatedCovariance = anchorPositionCovariance;
    if (covarianceDimension > 0) {
        const double covarianceScale = std::max(
            1.0,
            anchorPositionCovariance.cwiseAbs().maxCoeff()
        );
        constexpr double covarianceToleranceFactor = 1e-10;
        const double covarianceTolerance = covarianceToleranceFactor * covarianceScale;
        const double maxAsymmetry =
            (anchorPositionCovariance - anchorPositionCovariance.transpose()).cwiseAbs().maxCoeff();
        if (maxAsymmetry > covarianceTolerance) {
            result.usedPseudoInverse = true;
            appendWarning("Anchor-position covariance is materially nonsymmetric.");
            return result;
        }

        validatedCovariance = 0.5 * (
            anchorPositionCovariance + anchorPositionCovariance.transpose()
        );
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> covarianceEigensolver(validatedCovariance);
        if (covarianceEigensolver.info() != Eigen::Success) {
            result.usedPseudoInverse = true;
            appendWarning("Failed to decompose the anchor-position covariance matrix.");
            return result;
        }

        const Eigen::VectorXd covarianceEigenvalues = covarianceEigensolver.eigenvalues();
        if (covarianceEigenvalues.minCoeff() < -covarianceTolerance) {
            result.usedPseudoInverse = true;
            appendWarning("Anchor-position covariance is not positive semidefinite.");
            return result;
        }

        // Project only roundoff-sized negative eigenvalues to zero. The documented
        // 1e-10 * max(1, max |C_a|) threshold is also used for symmetry validation.
        if (covarianceEigenvalues.minCoeff() < 0.0) {
            validatedCovariance = covarianceEigensolver.eigenvectors()
                * covarianceEigenvalues.cwiseMax(0.0).asDiagonal()
                * covarianceEigensolver.eigenvectors().transpose();
            validatedCovariance = 0.5 * (validatedCovariance + validatedCovariance.transpose());
        }
    }

    if (anchorPositions.size() < 4) {
        result.usedPseudoInverse = true;
        appendWarning("Fewer than 4 anchors cannot fully constrain a 3D true-range position; displaying pseudo-inverse CRLB.");
    }

    const double rangeVariance = rangeStdDev * rangeStdDev;
    if (!std::isfinite(rangeVariance)) {
        result.usedPseudoInverse = true;
        appendWarning("Range variance overflowed; use a smaller standard deviation.");
        return result;
    }

    constexpr double minRange = 1e-12;
    Eigen::MatrixXd U(anchorCount, 3);
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(anchorCount, covarianceDimension);
    Eigen::Index usableAnchorCount = 0;

    for (Eigen::Index anchorIndex = 0; anchorIndex < anchorCount; ++anchorIndex) {
        const Eigen::Vector3d& anchorPosition = anchorPositions[static_cast<size_t>(anchorIndex)];
        const Eigen::Vector3d delta = evaluationPosition - anchorPosition;
        const double rho = delta.norm();
        if (!delta.allFinite() || !std::isfinite(rho)) {
            result.usedPseudoInverse = true;
            appendWarning("Anchor-to-evaluation geometry overflowed during CRLB calculation.");
            return result;
        }
        if (rho <= minRange) {
            result.usedPseudoInverse = true;
            appendWarning("An anchor is too close to the CRLB evaluation position and was skipped.");
            continue;
        }

        const Eigen::Vector3d u = delta / rho;
        U.row(usableAnchorCount) = u.transpose();
        B.block<1, 3>(usableAnchorCount, 3 * anchorIndex) = -u.transpose();
        ++usableAnchorCount;
    }

    if (usableAnchorCount == 0) {
        result.valid = false;
        result.usedPseudoInverse = true;
        appendWarning("No usable anchors remain after input validation.");
        return result;
    }

    U.conservativeResize(usableAnchorCount, Eigen::NoChange);
    B.conservativeResize(usableAnchorCount, Eigen::NoChange);

    Eigen::MatrixXd effectiveRangeCovariance =
        rangeVariance * Eigen::MatrixXd::Identity(usableAnchorCount, usableAnchorCount)
        + B * validatedCovariance * B.transpose();
    effectiveRangeCovariance = 0.5 * (
        effectiveRangeCovariance + effectiveRangeCovariance.transpose()
    );
    if (!effectiveRangeCovariance.allFinite()) {
        result.usedPseudoInverse = true;
        appendWarning("Effective range covariance contains nonfinite values.");
        return result;
    }

    Eigen::LLT<Eigen::MatrixXd> covarianceFactorization(effectiveRangeCovariance);
    if (covarianceFactorization.info() != Eigen::Success) {
        result.usedPseudoInverse = true;
        appendWarning("Failed to factor the effective range covariance matrix.");
        return result;
    }

    const Eigen::MatrixXd weightedJacobian = covarianceFactorization.solve(U);
    if (covarianceFactorization.info() != Eigen::Success || !weightedJacobian.allFinite()) {
        result.usedPseudoInverse = true;
        appendWarning("Failed to solve with the effective range covariance matrix.");
        return result;
    }

    result.fisherInformation = U.transpose() * weightedJacobian;
    result.fisherInformation = 0.5 * (
        result.fisherInformation + result.fisherInformation.transpose()
    );
    if (!result.fisherInformation.allFinite()) {
        result.usedPseudoInverse = true;
        appendWarning("Fisher information matrix contains nonfinite values.");
        return result;
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(result.fisherInformation);
    if (eigensolver.info() != Eigen::Success) {
        result.valid = false;
        result.usedPseudoInverse = true;
        appendWarning("Failed to decompose the Fisher information matrix.");
        return result;
    }

    const Eigen::Vector3d eigenvalues = eigensolver.eigenvalues();
    const Eigen::Matrix3d eigenvectors = eigensolver.eigenvectors();
    const double maxAbsEigenvalue = eigenvalues.cwiseAbs().maxCoeff();
    const double tolerance = 1e-12 * std::max(1.0, maxAbsEigenvalue);

    Eigen::Vector3d inverseEigenvalues = Eigen::Vector3d::Zero();
    result.rank = 0;
    for (Eigen::Index i = 0; i < eigenvalues.size(); ++i) {
        if (eigenvalues(i) > tolerance) {
            inverseEigenvalues(i) = 1.0 / eigenvalues(i);
            ++result.rank;
        }
    }

    result.crlb = eigenvectors * inverseEigenvalues.asDiagonal() * eigenvectors.transpose();
    result.crlb = 0.5 * (result.crlb + result.crlb.transpose());
    result.valid = true;

    if (result.rank < 3) {
        result.usedPseudoInverse = true;
        appendWarning("Fisher information matrix is rank deficient; displaying pseudo-inverse CRLB. The true covariance bound is unbounded in one or more directions.");
    }

    return result;
}

} // namespace TrueRangeMultilateration

// END OF FILE //
