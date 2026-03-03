#include <raylib.h>
#include <rlImGui.h>

#include "web_app.h"

#if defined(__EMSCRIPTEN__)
#include <emscripten/emscripten.h>
#endif

namespace {
WebApp* gApp = nullptr;

void frame() {
    gApp->runFrame();
}
}  // namespace

int main() {
    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE);
    InitWindow(1280, 720, "Multilateration Algorithms Web App");
    SetTargetFPS(60);
    rlImGuiSetup(true);

    WebApp app;
    gApp = &app;

#if defined(__EMSCRIPTEN__)
    emscripten_set_main_loop(frame, 0, true);
#else
    while (!WindowShouldClose()) {
        frame();
    }
#endif

    rlImGuiShutdown();
    CloseWindow();
    return 0;
}
