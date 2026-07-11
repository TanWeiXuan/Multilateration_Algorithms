#include "simulation_runner.h"

#include "algorithm_dispatch.h"
#include "../test_helpers.h"

namespace TrueRangeMultilateration {

void SimulationRunner::begin(const TestParameters& params) {
    params_ = params;
    status_ = Status::Running;
    currentRun_ = 0;
    errorMessage_.clear();
    estimatedPositions_.clear();
    estimatedPositions_.reserve(params_.numRuns);
    results_ = TestResults{};
    rng_ = makeRandomEngine(params_.randomSeed);
    startedAt_ = std::chrono::steady_clock::now();
    endedAt_ = startedAt_;
}

void SimulationRunner::step(const size_t maxIterationsPerFrame) {
    if (status_ != Status::Running) {
        return;
    }

    try {
        const size_t end = std::min(params_.numRuns, currentRun_ + maxIterationsPerFrame);
        for (; currentRun_ < end; ++currentRun_) {
            const auto noisyRanges = generateNoisyRanges(
                params_.truePosition,
                params_.anchorPositions,
                params_.rangeNoiseStdDev,
                params_.rangeOutlierRatio,
                params_.rangeOutlierMagnitude,
                rng_);

            std::vector<Eigen::Vector3d> estimatedAnchorPositions = params_.anchorPositions;
            if (params_.anchorPosNoiseStdDev > 0.0) {
                estimatedAnchorPositions = generateNoisyAnchorPositions(
                    params_.anchorPositions,
                    params_.anchorPosNoiseStdDev,
                    rng_);
            }

            estimatedPositions_.push_back(runAlgorithm(
                params_.algorithm,
                estimatedAnchorPositions,
                noisyRanges,
                params_.rangeNoiseStdDev));
        }

        if (currentRun_ >= params_.numRuns) {
            finalize();
        }
    } catch (const std::exception& ex) {
        errorMessage_ = ex.what();
        status_ = Status::Error;
    }
}

void SimulationRunner::finalize() {
    if (estimatedPositions_.empty()) {
        status_ = Status::Error;
        errorMessage_ = "No estimates produced";
        return;
    }

    results_ = computeResults(estimatedPositions_, params_);
    status_ = Status::Completed;
    endedAt_ = std::chrono::steady_clock::now();
}

void SimulationRunner::cancel() {
    status_ = Status::Idle;
    currentRun_ = 0;
    estimatedPositions_.clear();
    errorMessage_.clear();
}

double SimulationRunner::progress() const {
    if (params_.numRuns == 0) {
        return 0.0;
    }
    return static_cast<double>(currentRun_) / static_cast<double>(params_.numRuns);
}

double SimulationRunner::elapsedMs() const {
    const auto end = (status_ != Status::Running) ? endedAt_ : std::chrono::steady_clock::now();
    return std::chrono::duration<double, std::milli>(end - startedAt_).count();
}

}  // namespace TrueRangeMultilateration
