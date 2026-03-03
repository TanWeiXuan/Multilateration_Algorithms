#include "viewport2d.h"

#include <algorithm>

Vector2 Viewport2D::worldToScreen(const Vector2& world) const {
    return {
        canvas.x + (world.x - center.x) * zoom + canvas.width * 0.5F,
        canvas.y + (center.y - world.y) * zoom + canvas.height * 0.5F,
    };
}

Vector2 Viewport2D::screenToWorld(const Vector2& screen) const {
    return {
        center.x + (screen.x - (canvas.x + canvas.width * 0.5F)) / zoom,
        center.y - (screen.y - (canvas.y + canvas.height * 0.5F)) / zoom,
    };
}

void Viewport2D::panByScreenDelta(const Vector2& delta) {
    center.x -= delta.x / zoom;
    center.y += delta.y / zoom;
}

void Viewport2D::zoomAtScreenPoint(const float zoomFactor, const Vector2& screenPoint) {
    const Vector2 before = screenToWorld(screenPoint);
    zoom = std::clamp(zoom * zoomFactor, 1.0F, 400.0F);
    const Vector2 after = screenToWorld(screenPoint);
    center.x += before.x - after.x;
    center.y += before.y - after.y;
}

void Viewport2D::fitPoints(const std::vector<Vector2>& points, const float paddingRatio) {
    if (points.empty()) {
        return;
    }

    float minX = points.front().x;
    float minY = points.front().y;
    float maxX = points.front().x;
    float maxY = points.front().y;

    for (const auto& p : points) {
        minX = std::min(minX, p.x);
        minY = std::min(minY, p.y);
        maxX = std::max(maxX, p.x);
        maxY = std::max(maxY, p.y);
    }

    center = {(minX + maxX) * 0.5F, (minY + maxY) * 0.5F};

    const float spanX = std::max(maxX - minX, 0.1F);
    const float spanY = std::max(maxY - minY, 0.1F);
    const float targetSpanX = spanX * (1.0F + paddingRatio * 2.0F);
    const float targetSpanY = spanY * (1.0F + paddingRatio * 2.0F);
    zoom = std::min(canvas.width / targetSpanX, canvas.height / targetSpanY);
    zoom = std::clamp(zoom, 1.0F, 400.0F);
}
