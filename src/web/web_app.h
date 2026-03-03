#pragma once

#include <vector>

#include <Eigen/Dense>
#include <raylib.h>

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
    void drawGrid() const;
    void drawPanel();
    void fitView();

    std::vector<Anchor> anchors_;
    TrueRangeMultilateration::TestParameters params_;
    TrueRangeMultilateration::SimulationRunner runner_;
    Viewport2D viewport_;
    bool showEstimates_ = true;
    int maxRenderedEstimates_ = 1000;
    bool fitRequested_ = true;
    bool touchPanActive_ = false;
    bool touchPinchActive_ = false;
    Vector2 lastTouchPosition_ = {0.0F, 0.0F};
    Vector2 lastTouchMidpoint_ = {0.0F, 0.0F};
    float lastTouchDistance_ = 0.0F;
};
