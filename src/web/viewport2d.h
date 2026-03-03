#pragma once

#include <vector>

#include <raylib.h>

class Viewport2D {
  public:
    Rectangle canvas{20.0F, 20.0F, 900.0F, 600.0F};
    Vector2 center{0.0F, 0.0F};
    float zoom = 30.0F;

    [[nodiscard]] Vector2 worldToScreen(const Vector2& world) const;
    [[nodiscard]] Vector2 screenToWorld(const Vector2& screen) const;
    void panByScreenDelta(const Vector2& delta);
    void zoomAtScreenPoint(float zoomFactor, const Vector2& screenPoint);
    void fitPoints(const std::vector<Vector2>& points, float paddingRatio = 0.15F);
};
