#include "web_platform.h"

namespace {
WebViewportMetrics gWebViewportMetrics{};
}

void updateWebViewportMetrics(const WebViewportMetrics& metrics) {
    gWebViewportMetrics = metrics;
}

WebViewportMetrics getWebViewportMetrics() {
    return gWebViewportMetrics;
}
