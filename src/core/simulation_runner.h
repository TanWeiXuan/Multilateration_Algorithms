#pragma once

#include <chrono>
#include <random>
#include <vector>

#include <Eigen/Dense>

#include "simulation_types.h"

namespace TrueRangeMultilateration {

class SimulationRunner {
  public:
    enum class Status { Idle, Running, Completed, Error };

    void begin(const TestParameters& params);
    void step(size_t maxIterationsPerFrame);
    void finalize();
    void cancel();

    [[nodiscard]] Status status() const { return status_; }
    [[nodiscard]] size_t currentRun() const { return currentRun_; }
    [[nodiscard]] size_t totalRuns() const { return params_.numRuns; }
    [[nodiscard]] double progress() const;
    [[nodiscard]] const TestResults& results() const { return results_; }
    [[nodiscard]] const std::vector<Eigen::Vector3d>& estimatedPositions() const { return estimatedPositions_; }
    [[nodiscard]] double elapsedMs() const;
    [[nodiscard]] const std::string& errorMessage() const { return errorMessage_; }

  private:
    TestParameters params_{};
    Status status_ = Status::Idle;
    size_t currentRun_ = 0;
    std::mt19937_64 rng_;
    std::vector<Eigen::Vector3d> estimatedPositions_;
    TestResults results_{};
    std::chrono::steady_clock::time_point startedAt_{};
    std::chrono::steady_clock::time_point endedAt_{};
    std::string errorMessage_{};
};

}  // namespace TrueRangeMultilateration
