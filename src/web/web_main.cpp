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

EM_JS(void, initMobileKeyboardProxy, (), {
    if (typeof document === 'undefined') {
        return;
    }

    if (Module.__imguiMobileKeyboardInitialized) {
        return;
    }
    Module.__imguiMobileKeyboardInitialized = true;

    Module.__imguiWantMobileKeyboard = false;

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

    const focusFromGesture = () => {
        if (!Module.__imguiWantMobileKeyboard) {
            if (document.activeElement === input) {
                input.blur();
            }
            return;
        }

        if (document.activeElement !== input) {
            try {
                input.focus({preventScroll: true});
            } catch (err) {
                input.focus();
            }
        }
    };

    const events = ['pointerup', 'touchend'];
    for (const eventName of events) {
        Module.canvas.addEventListener(eventName, focusFromGesture, {passive: true});
    }
});

EM_JS(void, setMobileKeyboardWanted, (int wanted), {
    if (typeof document === 'undefined' || !Module.__imguiMobileKeyboardInitialized) {
        return;
    }

    Module.__imguiWantMobileKeyboard = !!wanted;

    const input = document.getElementById('imgui-mobile-keyboard-proxy');
    if (!input) {
        return;
    }

    if (!Module.__imguiWantMobileKeyboard) {
        if (document.activeElement === input) {
            input.blur();
        }

        if (Module.canvas && document.activeElement !== Module.canvas) {
            try {
                Module.canvas.focus({preventScroll: true});
            } catch (err) {
                Module.canvas.focus();
            }
        }
    }
});
#endif

namespace {
WebApp* gApp = nullptr;
#if defined(__EMSCRIPTEN__)
bool gMobileKeyboardEnabled = false;
#endif

void frame() {
    gApp->runFrame();

#if defined(__EMSCRIPTEN__)
    if (gMobileKeyboardEnabled) {
        const ImGuiIO& io = ImGui::GetIO();
        setMobileKeyboardWanted(io.WantTextInput ? 1 : 0);
    }
#endif
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
    if (gMobileKeyboardEnabled) {
        initMobileKeyboardProxy();
    }
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
