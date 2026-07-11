#pragma once

struct WebViewportMetrics {
    float cssWidth = 0.0F;
    float cssHeight = 0.0F;
    float framebufferWidth = 0.0F;
    float framebufferHeight = 0.0F;
    float devicePixelRatio = 1.0F;
    bool touchDevice = false;
};

void updateWebViewportMetrics(const WebViewportMetrics& metrics);
WebViewportMetrics getWebViewportMetrics();
