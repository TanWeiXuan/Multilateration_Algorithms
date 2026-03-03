#include <raylib.h>
#include <rlImGui.h>

#include "web_app.h"

#if defined(__EMSCRIPTEN__)
#include <emscripten/emscripten.h>
#include <imgui.h>

EM_JS(int, isMobileTouchDevice, (), {
    return (typeof window !== 'undefined') &&
           (('ontouchstart' in window) ||
            (window.matchMedia && window.matchMedia('(pointer: coarse)').matches));
});

EM_JS(void, syncMobileKeyboard, (int shouldShow), {
    if (typeof document === 'undefined') {
        return;
    }

    let input = document.getElementById('imgui-mobile-keyboard-proxy');
    if (!input) {
        input = document.createElement('input');
        input.id = 'imgui-mobile-keyboard-proxy';
        input.type = 'text';
        input.autocapitalize = 'none';
        input.autocomplete = 'off';
        input.autocorrect = 'off';
        input.spellcheck = false;
        input.setAttribute('inputmode', 'decimal');
        input.style.position = 'fixed';
        input.style.left = '-1000px';
        input.style.top = '-1000px';
        input.style.width = '1px';
        input.style.height = '1px';
        input.style.opacity = '0';
        input.style.pointerEvents = 'none';
        document.body.appendChild(input);
    }

    if (shouldShow) {
        if (document.activeElement !== input) {
            input.focus({preventScroll: true});
        }
        return;
    }

    if (document.activeElement === input) {
        input.blur();
    }

    if (Module && Module.canvas && document.activeElement !== Module.canvas) {
        Module.canvas.focus({preventScroll: true});
    }
});
#endif

namespace {
WebApp* gApp = nullptr;
#if defined(__EMSCRIPTEN__)
bool gMobileKeyboardEnabled = false;
#endif

void frame() {
#if defined(__EMSCRIPTEN__)
    if (gMobileKeyboardEnabled) {
        const ImGuiIO& io = ImGui::GetIO();
        syncMobileKeyboard(io.WantTextInput ? 1 : 0);
    }
#endif
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
    gMobileKeyboardEnabled = isMobileTouchDevice() != 0;
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
