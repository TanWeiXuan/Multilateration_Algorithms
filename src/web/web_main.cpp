#include <raylib.h>
#include <rlImGui.h>

#include "web_app.h"

#if defined(__EMSCRIPTEN__)
#include <algorithm>
#include <cmath>

#include <emscripten/emscripten.h>
#include <emscripten/html5.h>
#include <imgui.h>

EM_JS(int, isMobileTouchDevice, (), {
    return (typeof window !== 'undefined') &&
           (('ontouchstart' in window) ||
            (window.matchMedia && window.matchMedia('(pointer: coarse)').matches));
});

EM_JS(double, getDevicePixelRatio, (), {
    if (typeof window === 'undefined' || !window.devicePixelRatio) {
        return 1.0;
    }
    return window.devicePixelRatio;
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
    // Browsers already forward text input into the Emscripten event pipeline.
    // Manually forwarding `input`/`keydown` events causes duplicate characters,
    // so keep manual forwarding disabled consistently on every browser.
    Module.__imguiMobileManualForwarding = false;

    let input = document.getElementById('imgui-mobile-keyboard-proxy');
    if (!input) {
        input = document.createElement('input');
        input.id = 'imgui-mobile-keyboard-proxy';
        input.type = 'text';
        input.autocapitalize = 'none';
        input.autocomplete = 'off';
        input.autocorrect = 'off';
        input.spellcheck = false;
        input.setAttribute('inputmode', 'text');
        input.style.position = 'fixed';
        input.style.left = '-1000px';
        input.style.top = '-1000px';
        input.style.width = '1px';
        input.style.height = '1px';
        input.style.opacity = '0';
        input.style.pointerEvents = 'none';
        document.body.appendChild(input);
    }

    const dispatchImGuiChar = (text) => {
        if (typeof text !== 'string' || text.length === 0) {
            return;
        }
        Module.ccall('imgui_mobile_add_input_characters_utf8', null, ['string'], [text]);
    };

    input.addEventListener('input', (event) => {
        if (!Module.__imguiWantMobileKeyboard || !Module.__imguiMobileManualForwarding) {
            input.value = '';
            return;
        }

        if (event && event.inputType === 'deleteContentBackward') {
            Module.ccall('imgui_mobile_send_backspace', null, [], []);
        } else if (event && typeof event.data === 'string' && event.data.length > 0) {
            dispatchImGuiChar(event.data);
        }

        input.value = '';
    });

    input.addEventListener('keydown', (event) => {
        if (!Module.__imguiWantMobileKeyboard || !Module.__imguiMobileManualForwarding) {
            return;
        }

        if (event.key === 'Enter') {
            Module.ccall('imgui_mobile_send_enter', null, [], []);
            event.preventDefault();
        } else if (event.key === 'Escape') {
            Module.ccall('imgui_mobile_send_escape', null, [], []);
            event.preventDefault();
        }
    });

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

    const events = ['pointerup'];
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

void syncCanvasToViewport() {
    double cssWidth = 0.0;
    double cssHeight = 0.0;
    if (emscripten_get_element_css_size("#canvas", &cssWidth, &cssHeight) != EMSCRIPTEN_RESULT_SUCCESS || cssWidth <= 0.0 ||
        cssHeight <= 0.0) {
        return;
    }

    const double dpr = std::max(1.0, getDevicePixelRatio());
    const int pixelWidth = std::max(1, static_cast<int>(std::lround(cssWidth * dpr)));
    const int pixelHeight = std::max(1, static_cast<int>(std::lround(cssHeight * dpr)));

    emscripten_set_canvas_element_size("#canvas", pixelWidth, pixelHeight);
    SetWindowSize(pixelWidth, pixelHeight);
}

EM_BOOL onCanvasResize(int, const EmscriptenUiEvent*, void*) {
    syncCanvasToViewport();
    return EM_TRUE;
}

EM_BOOL onCanvasOrientationChange(int, const EmscriptenOrientationChangeEvent*, void*) {
    syncCanvasToViewport();
    return EM_TRUE;
}

void installViewportResizeHandlers() {
    syncCanvasToViewport();
    emscripten_set_resize_callback(EMSCRIPTEN_EVENT_TARGET_WINDOW, nullptr, EM_FALSE, onCanvasResize);
    emscripten_set_orientationchange_callback(EMSCRIPTEN_EVENT_TARGET_SCREEN, nullptr, EM_FALSE,
                                              onCanvasOrientationChange);
}

extern "C" {
EMSCRIPTEN_KEEPALIVE void imgui_mobile_add_input_characters_utf8(const char* text) {
    if (!text || !text[0]) {
        return;
    }
    ImGui::GetIO().AddInputCharactersUTF8(text);
}

EMSCRIPTEN_KEEPALIVE void imgui_mobile_send_backspace() {
    ImGuiIO& io = ImGui::GetIO();
    io.AddKeyEvent(ImGuiKey_Backspace, true);
    io.AddKeyEvent(ImGuiKey_Backspace, false);
}

EMSCRIPTEN_KEEPALIVE void imgui_mobile_send_enter() {
    ImGuiIO& io = ImGui::GetIO();
    io.AddKeyEvent(ImGuiKey_Enter, true);
    io.AddKeyEvent(ImGuiKey_Enter, false);
}

EMSCRIPTEN_KEEPALIVE void imgui_mobile_send_escape() {
    ImGuiIO& io = ImGui::GetIO();
    io.AddKeyEvent(ImGuiKey_Escape, true);
    io.AddKeyEvent(ImGuiKey_Escape, false);
}
}
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
    installViewportResizeHandlers();
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
