#pragma once

#include <vector>

#include <Eigen/Dense>

#include "../core/simulation_runner.h"
#include "viewport2d.h"

struct Anchor {
    int id = 0;
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
};

class WebApp {
  public:
    WebApp();
    void runFrame();

  private:
    void handleViewportInput();
    void drawScene() const;
    void drawPanel();
    void fitView();

    std::vector<Anchor> anchors_;
    TrueRangeMultilateration::TestParameters params_;
    TrueRangeMultilateration::SimulationRunner runner_;
    Viewport2D viewport_;
    bool showEstimates_ = true;
    int maxRenderedEstimates_ = 1000;
    bool fitRequested_ = true;
};
