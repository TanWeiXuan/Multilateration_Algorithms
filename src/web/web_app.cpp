#include "web_app.h"

#include <algorithm>
#include <string>

#include <imgui.h>
#include <raylib.h>
#include <rlImGui.h>

using TrueRangeMultilateration::AlgorithmId;

WebApp::WebApp() {
    params_.truePosition = Eigen::Vector3d(0.0, 0.0, 5.0);
    params_.anchorPositions = {
        Eigen::Vector3d(-5.0, -5.0, 10.0), Eigen::Vector3d(-5.0, 5.0, 10.0),
        Eigen::Vector3d(5.0, 5.0, 10.0),   Eigen::Vector3d(5.0, -5.0, 10.0),
        Eigen::Vector3d(-5.0, -5.0, 0.0),  Eigen::Vector3d(-5.0, 5.0, 0.0),
        Eigen::Vector3d(5.0, 5.0, 0.0),    Eigen::Vector3d(5.0, -5.0, 0.0),
    };

    anchors_.reserve(params_.anchorPositions.size());
    for (size_t i = 0; i < params_.anchorPositions.size(); ++i) {
        anchors_.push_back({static_cast<int>(i), params_.anchorPositions[i]});
    }

    params_.rangeNoiseStdDev = 0.25;
    params_.numRuns = 2000;
}

void WebApp::runFrame() {
    viewport_.canvas = {15.0F, 15.0F, static_cast<float>(GetScreenWidth()) - 330.0F,
                        static_cast<float>(GetScreenHeight()) - 30.0F};

    if (fitRequested_) {
        fitView();
        fitRequested_ = false;
    }

    handleViewportInput();
    if (runner_.status() == TrueRangeMultilateration::SimulationRunner::Status::Running) {
        runner_.step(128);
    }

    BeginDrawing();
    ClearBackground(RAYWHITE);
    drawScene();

    rlImGuiBegin();
    drawPanel();
    rlImGuiEnd();

    EndDrawing();
}

void WebApp::handleViewportInput() {
    const Vector2 mouse = GetMousePosition();
    const bool overCanvas = CheckCollisionPointRec(mouse, viewport_.canvas);
    const ImGuiIO& io = ImGui::GetIO();
    if (!overCanvas || io.WantCaptureMouse) {
        return;
    }

    if (IsMouseButtonDown(MOUSE_BUTTON_MIDDLE) || IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
        viewport_.panByScreenDelta(GetMouseDelta());
    }

    const float wheel = GetMouseWheelMove();
    if (wheel != 0.0F) {
        viewport_.zoomAtScreenPoint(1.0F + wheel * 0.12F, mouse);
    }
}

void WebApp::drawScene() const {
    DrawRectangleRounded(viewport_.canvas, 0.02F, 1, Fade(LIGHTGRAY, 0.3F));

    for (const auto& anchor : anchors_) {
        const Vector2 w{static_cast<float>(anchor.position.x()), static_cast<float>(anchor.position.y())};
        const Vector2 s = viewport_.worldToScreen(w);
        DrawCircleV(s, 5.0F, BLUE);
        DrawText(TextFormat("A%d", anchor.id), static_cast<int>(s.x - 8.0F), static_cast<int>(s.y + 8.0F), 14,
                 DARKBLUE);
        DrawText(TextFormat("z=%.2f m", anchor.position.z()), static_cast<int>(s.x - 20.0F),
                 static_cast<int>(s.y + 22.0F), 12, DARKGRAY);
    }

    const Vector2 gtW{static_cast<float>(params_.truePosition.x()), static_cast<float>(params_.truePosition.y())};
    const Vector2 gtS = viewport_.worldToScreen(gtW);
    DrawCircleV(gtS, 6.0F, GREEN);
    DrawText("GT", static_cast<int>(gtS.x - 8.0F), static_cast<int>(gtS.y + 8.0F), 14, DARKGREEN);
    DrawText(TextFormat("z=%.2f m", params_.truePosition.z()), static_cast<int>(gtS.x - 20.0F),
             static_cast<int>(gtS.y + 22.0F), 12, DARKGRAY);

    if (showEstimates_) {
        const auto& samples = runner_.estimatedPositions();
        const size_t stride = std::max<size_t>(1, samples.size() / std::max(1, maxRenderedEstimates_));
        for (size_t i = 0; i < samples.size(); i += stride) {
            const Vector2 p = viewport_.worldToScreen(
                {static_cast<float>(samples[i].x()), static_cast<float>(samples[i].y())});
            DrawLineEx({p.x - 3.0F, p.y - 3.0F}, {p.x + 3.0F, p.y + 3.0F}, 1.0F, Fade(RED, 0.3F));
            DrawLineEx({p.x - 3.0F, p.y + 3.0F}, {p.x + 3.0F, p.y - 3.0F}, 1.0F, Fade(RED, 0.3F));
        }
    }
}

void WebApp::drawPanel() {
    ImGui::SetNextWindowPos({static_cast<float>(GetScreenWidth() - 305), static_cast<float>(GetScreenHeight() - 500)},
                            ImGuiCond_Always);
    ImGui::SetNextWindowSize({290.0F, 480.0F}, ImGuiCond_Always);
    ImGui::Begin("Multilateration Controls", nullptr, ImGuiWindowFlags_NoCollapse);

    if (ImGui::CollapsingHeader("Anchor and GT Configuration", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (ImGui::Button("Add Anchor")) {
            anchors_.push_back({static_cast<int>(anchors_.size()), Eigen::Vector3d::Zero()});
        }
        ImGui::SameLine();
        if (ImGui::Button("Remove Last") && !anchors_.empty()) {
            anchors_.pop_back();
        }

        for (size_t i = 0; i < anchors_.size(); ++i) {
            float values[3] = {static_cast<float>(anchors_[i].position.x()), static_cast<float>(anchors_[i].position.y()),
                               static_cast<float>(anchors_[i].position.z())};
            if (ImGui::InputFloat3(("A" + std::to_string(i)).c_str(), values)) {
                anchors_[i].position = Eigen::Vector3d(values[0], values[1], values[2]);
            }
        }

        float gt[3] = {static_cast<float>(params_.truePosition.x()), static_cast<float>(params_.truePosition.y()),
                       static_cast<float>(params_.truePosition.z())};
        if (ImGui::InputFloat3("GT xyz", gt)) {
            params_.truePosition = Eigen::Vector3d(gt[0], gt[1], gt[2]);
        }
    }

    if (ImGui::CollapsingHeader("Test Parameter Configuration", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::InputDouble("rangeNoiseStdDev", &params_.rangeNoiseStdDev, 0.01, 0.1, "%.3f");
        float rangeOutlierRatio = static_cast<float>(params_.rangeOutlierRatio);
        if (ImGui::SliderFloat("rangeOutlierRatio", &rangeOutlierRatio, 0.0F, 1.0F)) {
            params_.rangeOutlierRatio = static_cast<double>(rangeOutlierRatio);
        }
        params_.rangeOutlierRatio = std::clamp(params_.rangeOutlierRatio, 0.0, 1.0);
        ImGui::InputDouble("rangeOutlierMagnitude", &params_.rangeOutlierMagnitude);
        ImGui::InputDouble("anchorPosNoiseStdDev", &params_.anchorPosNoiseStdDev);
        int numRuns = static_cast<int>(params_.numRuns);
        ImGui::InputInt("numRuns", &numRuns);
        params_.numRuns = static_cast<size_t>(std::max(1, numRuns));

        bool fixedSeed = params_.randomSeed.has_value();
        if (ImGui::Checkbox("Use Fixed Seed", &fixedSeed)) {
            if (!fixedSeed) {
                params_.randomSeed.reset();
            } else {
                params_.randomSeed = 42;
            }
        }
        if (fixedSeed) {
            long long seed = static_cast<long long>(params_.randomSeed.value_or(42));
            if (ImGui::InputScalar("Fixed Seed", ImGuiDataType_S64, &seed)) {
                params_.randomSeed = static_cast<uint64_t>(std::max(0LL, seed));
            }
        }
    }

    if (ImGui::CollapsingHeader("Algorithm Selection", ImGuiTreeNodeFlags_DefaultOpen)) {
        int selected = static_cast<int>(params_.algorithm);
        const char* names[] = {
            "Ordinary Least Squares (Wikipedia)",
            "Ordinary Least Squares (Wikipedia + BDCSVD)",
            "Nonlinear Least Squares (Eigen LM)",
            "Robust Nonlinear Least Squares (Eigen LM + IRLS/Cauchy)",
            "LLS-I (Yue Wang)",
            "LLS-II-2 (Yue Wang)",
            "Two-Step Weighted LLS-I (Yue Wang / Chan-Ho)",
        };
        if (ImGui::Combo("Algorithm", &selected, names, IM_ARRAYSIZE(names))) {
            params_.algorithm = static_cast<AlgorithmId>(selected);
        }
    }

    if (ImGui::CollapsingHeader("Simulation Start / Progress", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (ImGui::Button("Start Simulation")) {
            params_.anchorPositions.clear();
            params_.anchorPositions.reserve(anchors_.size());
            for (const auto& a : anchors_) params_.anchorPositions.push_back(a.position);
            runner_.begin(params_);
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel")) {
            runner_.cancel();
        }
        ImGui::ProgressBar(static_cast<float>(runner_.progress()), {-1.0F, 0.0F});
        ImGui::Text("%zu / %zu", runner_.currentRun(), runner_.totalRuns());
    }

    if (ImGui::CollapsingHeader("Test Results", ImGuiTreeNodeFlags_DefaultOpen)) {
        const auto& r = runner_.results();
        ImGui::Text("Mean Absolute Error:\n [%.3f %.3f %.3f]", r.meanAbsError.x(), r.meanAbsError.y(), r.meanAbsError.z());
        ImGui::Text("Max Error:\n [%.3f %.3f %.3f]", r.maxError.x(), r.maxError.y(), r.maxError.z());
        ImGui::Text("Covariance Diagonal:\n [%.4f %.4f %.4f]", r.errorCovariance(0, 0), r.errorCovariance(1, 1),
                    r.errorCovariance(2, 2));
        ImGui::Text("Elapsed: %.1f ms", runner_.elapsedMs());
    }

    if (ImGui::CollapsingHeader("Visualization / View Controls", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (ImGui::Button("Fit View")) {
            fitRequested_ = true;
        }
        ImGui::SameLine();
        if (ImGui::Button("Reset View")) {
            viewport_.center = {0.0F, 0.0F};
            viewport_.zoom = 30.0F;
        }
        ImGui::Checkbox("Show Estimates", &showEstimates_);
        ImGui::SliderInt("Max Rendered Estimates", &maxRenderedEstimates_, 100, 5000);
    }

    ImGui::End();
}

void WebApp::fitView() {
    std::vector<Vector2> points;
    points.reserve(anchors_.size() + 1);
    for (const auto& a : anchors_) {
        points.push_back({static_cast<float>(a.position.x()), static_cast<float>(a.position.y())});
    }
    points.push_back({static_cast<float>(params_.truePosition.x()), static_cast<float>(params_.truePosition.y())});
    viewport_.fitPoints(points);
}
