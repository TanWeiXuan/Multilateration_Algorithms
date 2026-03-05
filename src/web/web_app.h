#pragma once

#include <vector>

#include <Eigen/Dense>
#include <imgui.h>
#include <raylib.h>

#include "../core/simulation_runner.h"
#include "viewport2d.h"
#include "web_platform.h"

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
    float computeUiScale() const;
    void applyUiScale(float scale);
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
    ImGuiStyle baseStyle_{};
    float appliedUiScale_ = 1.0F;
    bool baseStyleCaptured_ = false;
    mutable const char* uiScaleTierLabel_ = "desktop";
    bool showUiDebugInfo_ = false;
    WebViewportMetrics frameMetrics_{};
};
