#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "imgui.h"
#include "raylib.h"
#include "raymath.h"
#include "rlImGui.h"
#include "app_state.hpp"
#include "scene_query.hpp"
#include "render_overlay.hpp"
#include "panels.hpp"
#include "ui_common.hpp"
#include "wire/core/core_state.hpp"

namespace {

constexpr float kAxisLength = 2.0f;

constexpr const char* kViewerSettingsFile = "viewer_state.ini";

bool parse_bool(std::string_view value, bool fallback) {
  if (value == "1" || value == "true" || value == "True") {
    return true;
  }
  if (value == "0" || value == "false" || value == "False") {
    return false;
  }
  return fallback;
}

ViewerPersistentSettings LoadViewerPersistentSettings() {
  ViewerPersistentSettings settings{};
  std::ifstream ifs(kViewerSettingsFile);
  if (!ifs.is_open()) {
    return settings;
  }

  std::string line;
  while (std::getline(ifs, line)) {
    if (line.empty()) {
      continue;
    }
    const std::size_t eq = line.find('=');
    if (eq == std::string::npos || eq == 0 || eq + 1 >= line.size()) {
      continue;
    }
    const std::string key = line.substr(0, eq);
    const std::string value = line.substr(eq + 1);
    try {
      if (key == "window_width") {
        settings.window_width = std::max(640, std::stoi(value));
      } else if (key == "window_height") {
        settings.window_height = std::max(480, std::stoi(value));
      } else if (key == "ui_unified_workspace") {
        settings.ui_unified_workspace = parse_bool(value, settings.ui_unified_workspace);
      } else if (key == "ui_show_workspace") {
        settings.ui_show_workspace = parse_bool(value, settings.ui_show_workspace);
      } else if (key == "ui_workspace_width") {
        settings.ui_workspace_width = std::clamp(std::stof(value), 300.0f, 900.0f);
      } else if (key == "camera_fov_deg") {
        settings.camera_fov_deg = std::clamp(std::stof(value), 20.0f, 110.0f);
      } else if (key == "camera_walk_speed") {
        settings.camera_walk_speed = std::clamp(std::stof(value), 0.5f, 80.0f);
      }
    } catch (...) {
      // Ignore malformed line and keep defaults.
    }
  }
  return settings;
}

void SaveViewerPersistentSettings(const ViewerPersistentSettings& settings) {
  std::ofstream ofs(kViewerSettingsFile, std::ios::trunc);
  if (!ofs.is_open()) {
    return;
  }
  ofs << "window_width=" << settings.window_width << "\n";
  ofs << "window_height=" << settings.window_height << "\n";
  ofs << "ui_unified_workspace=" << (settings.ui_unified_workspace ? 1 : 0) << "\n";
  ofs << "ui_show_workspace=" << (settings.ui_show_workspace ? 1 : 0) << "\n";
  ofs << "ui_workspace_width=" << settings.ui_workspace_width << "\n";
  ofs << "camera_fov_deg=" << settings.camera_fov_deg << "\n";
  ofs << "camera_walk_speed=" << settings.camera_walk_speed << "\n";
}

wire::core::Vec3d Lerp(const wire::core::Vec3d& a, const wire::core::Vec3d& b, double t) {
  return wire::core::Vec3d{
      a.x + (b.x - a.x) * t,
      a.y + (b.y - a.y) * t,
      a.z + (b.z - a.z) * t,
  };
}

wire::core::Vec3d RotateXDeg(const wire::core::Vec3d& v, double deg) {
  constexpr double kPi = 3.14159265358979323846;
  const double rad = deg * (kPi / 180.0);
  const double c = std::cos(rad);
  const double s = std::sin(rad);
  return {v.x, v.y * c - v.z * s, v.y * s + v.z * c};
}

wire::core::Vec3d RotateYDeg(const wire::core::Vec3d& v, double deg) {
  constexpr double kPi = 3.14159265358979323846;
  const double rad = deg * (kPi / 180.0);
  const double c = std::cos(rad);
  const double s = std::sin(rad);
  return {v.x * c + v.z * s, v.y, -v.x * s + v.z * c};
}

wire::core::Vec3d RotateZDeg(const wire::core::Vec3d& v, double deg) {
  constexpr double kPi = 3.14159265358979323846;
  const double rad = deg * (kPi / 180.0);
  const double c = std::cos(rad);
  const double s = std::sin(rad);
  return {v.x * c - v.y * s, v.x * s + v.y * c, v.z};
}

wire::core::Vec3d RotateEulerXYZDeg(const wire::core::Vec3d& v, const wire::core::Vec3d& euler_deg) {
  const wire::core::Vec3d rx = RotateXDeg(v, euler_deg.x);
  const wire::core::Vec3d ry = RotateYDeg(rx, euler_deg.y);
  return RotateZDeg(ry, euler_deg.z);
}

wire::core::Vec3d PoleTopPoint(const wire::core::Pole& pole) {
  const wire::core::Vec3d local_up{0.0, 0.0, pole.height_m};
  return pole.world_transform.position + RotateEulerXYZDeg(local_up, pole.world_transform.rotation_euler_deg);
}

Color VisualPartColor(wire::core::VisualPartKind kind) {
  switch (kind) {
  case wire::core::VisualPartKind::kSupportArm:
    return BROWN;
  case wire::core::VisualPartKind::kInsulator:
    return SKYBLUE;
  case wire::core::VisualPartKind::kFitting:
    return LIGHTGRAY;
  default:
    return GRAY;
  }
}

void OrbitCameraTurntable(Camera3D* camera, Vector2 mouse_delta, float orbit_speed) {
  const Vector3 offset = Vector3Subtract(camera->position, camera->target);
  const float radius = Vector3Length(offset);
  if (radius <= 1e-5f) {
    return;
  }

  float yaw = std::atan2(offset.z, offset.x);
  float pitch = std::asin(offset.y / radius);

  yaw -= mouse_delta.x * orbit_speed;
  pitch -= mouse_delta.y * orbit_speed;

  const float pitch_limit = 1.55334f; // about 89 degrees
  pitch = std::clamp(pitch, -pitch_limit, pitch_limit);

  const float cos_pitch = std::cos(pitch);
  Vector3 new_offset{
      radius * cos_pitch * std::cos(yaw),
      radius * std::sin(pitch),
      radius * cos_pitch * std::sin(yaw),
  };

  camera->position = Vector3Add(camera->target, new_offset);
}

Vector3 CameraForward(const Camera3D& camera) {
  return Vector3Normalize(Vector3Subtract(camera.target, camera.position));
}

Vector3 CameraRight(const Camera3D& camera) {
  const Vector3 fwd = CameraForward(camera);
  return Vector3Normalize(Vector3CrossProduct(fwd, camera.up));
}

void UpdateWalkCamera(Camera3D* camera, ViewerUiState& ui_state) {
  const float dt = std::max(GetFrameTime(), 1.0f / 240.0f);
  const bool speed_boost = IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT);
  const float speed = ui_state.camera_walk_speed * (speed_boost ? 3.0f : 1.0f);

  Vector3 forward = CameraForward(*camera);
  Vector3 right = Vector3Normalize(Vector3CrossProduct(forward, camera->up));

  const Vector2 mouse_delta = GetMouseDelta();
  const float yaw = -mouse_delta.x * ui_state.camera_mouse_sensitivity;
  const float pitch = -mouse_delta.y * ui_state.camera_mouse_sensitivity;
  if (std::fabs(yaw) > 0.0f) {
    forward = Vector3Normalize(Vector3RotateByAxisAngle(forward, camera->up, yaw));
  }
  if (std::fabs(pitch) > 0.0f) {
    right = Vector3Normalize(Vector3CrossProduct(forward, camera->up));
    Vector3 pitched = Vector3Normalize(Vector3RotateByAxisAngle(forward, right, pitch));
    const float up_dot = std::fabs(Vector3DotProduct(pitched, camera->up));
    if (up_dot < 0.995f) {
      forward = pitched;
    }
  }
  right = Vector3Normalize(Vector3CrossProduct(forward, camera->up));

  Vector3 move{0.0f, 0.0f, 0.0f};
  if (IsKeyDown(KEY_W)) {
    move = Vector3Add(move, forward);
  }
  if (IsKeyDown(KEY_S)) {
    move = Vector3Subtract(move, forward);
  }
  if (IsKeyDown(KEY_A)) {
    move = Vector3Subtract(move, right);
  }
  if (IsKeyDown(KEY_D)) {
    move = Vector3Add(move, right);
  }
  if (IsKeyDown(KEY_E)) {
    move = Vector3Add(move, camera->up);
  }
  if (IsKeyDown(KEY_Q)) {
    move = Vector3Subtract(move, camera->up);
  }
  if (Vector3Length(move) > 1e-5f) {
    move = Vector3Scale(Vector3Normalize(move), speed * dt);
    camera->position = Vector3Add(camera->position, move);
  }
  camera->target = Vector3Add(camera->position, forward);

  const float wheel = GetMouseWheelMove();
  if (std::fabs(wheel) > 0.0f) {
    ui_state.camera_walk_speed = std::clamp(ui_state.camera_walk_speed + wheel * 0.75f, 0.5f, 80.0f);
  }
}

void PanCamera(Camera3D* camera, Vector2 mouse_delta, float pan_speed) {
  const Vector3 forward = Vector3Normalize(Vector3Subtract(camera->target, camera->position));
  const Vector3 right = Vector3Normalize(Vector3CrossProduct(forward, camera->up));
  const Vector3 up = Vector3Normalize(camera->up);
  const float distance = Vector3Distance(camera->position, camera->target);
  const float scale = pan_speed * std::max(0.1f, distance);

  const Vector3 pan_x = Vector3Scale(right, -mouse_delta.x * scale);
  const Vector3 pan_y = Vector3Scale(up, mouse_delta.y * scale);
  const Vector3 pan = Vector3Add(pan_x, pan_y);

  camera->position = Vector3Add(camera->position, pan);
  camera->target = Vector3Add(camera->target, pan);
}

void DollyCamera(Camera3D* camera, float amount, float dolly_speed) {
  const Vector3 view = Vector3Subtract(camera->position, camera->target);
  float distance = Vector3Length(view);
  if (distance <= 1e-5f) {
    distance = 1e-5f;
  }
  distance *= (1.0f + amount * dolly_speed);
  distance = std::clamp(distance, 0.2f, 10000.0f);

  const Vector3 dir = Vector3Normalize(view);
  camera->position = Vector3Add(camera->target, Vector3Scale(dir, distance));
}

void UpdateCameraFov(Camera3D* camera, ViewerUiState& ui_state, bool ui_captures_keyboard) {
  ui_state.camera_fov_deg = std::clamp(ui_state.camera_fov_deg, 20.0f, 110.0f);
  if (!ui_captures_keyboard) {
    if (IsKeyPressed(KEY_LEFT_BRACKET)) {
      ui_state.camera_fov_deg = std::clamp(ui_state.camera_fov_deg - 2.0f, 20.0f, 110.0f);
    } else if (IsKeyPressed(KEY_RIGHT_BRACKET)) {
      ui_state.camera_fov_deg = std::clamp(ui_state.camera_fov_deg + 2.0f, 20.0f, 110.0f);
    }
  }
  camera->fovy = ui_state.camera_fov_deg;
}

void UpdateCameraForViewport(Camera3D* camera, ViewerUiState& ui_state) {
  ui_state.camera_consumed_escape = false;
  ImGuiIO& io = ImGui::GetIO();
  const bool ui_captures_mouse = io.WantCaptureMouse;
  const bool ui_captures_keyboard = io.WantCaptureKeyboard;
  const Vector2 mouse_delta = GetMouseDelta();
  const bool shift = IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT);
  const bool ctrl = IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL);
  const bool alt = IsKeyDown(KEY_LEFT_ALT) || IsKeyDown(KEY_RIGHT_ALT);

  UpdateCameraFov(camera, ui_state, ui_captures_keyboard);

  if (!ui_captures_keyboard && shift && IsKeyPressed(KEY_F)) {
    ui_state.camera_walk_mode = !ui_state.camera_walk_mode;
    ui_state.camera_drag_mode = CameraDragMode::kNone;
    if (ui_state.camera_walk_mode) {
      DisableCursor();
      PushLog(ui_state, "Camera: Walk mode ON (WASD + mouse, Q/E up/down, Esc exit)");
    } else {
      EnableCursor();
      PushLog(ui_state, "Camera: Walk mode OFF");
    }
  }
  if (ui_state.camera_walk_mode) {
    if (IsKeyPressed(KEY_ESCAPE)) {
      ui_state.camera_walk_mode = false;
      ui_state.camera_consumed_escape = true;
      EnableCursor();
      PushLog(ui_state, "Camera: Walk mode OFF");
      return;
    }
    UpdateWalkCamera(camera, ui_state);
    return;
  }

  if (!ui_captures_mouse && alt && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
    wire::core::Vec3d picked{};
    if (TryPickGroundPoint(*camera, ui_state.draw_plane_z, &picked)) {
      camera->target = ToRaylib(picked);
      PushLog(ui_state, "Camera: orbit pivot set from click");
    }
  }

  if (!ui_captures_mouse && IsMouseButtonPressed(MOUSE_BUTTON_MIDDLE)) {
    if (shift) {
      ui_state.camera_drag_mode = CameraDragMode::kPan;
    } else if (ctrl) {
      ui_state.camera_drag_mode = CameraDragMode::kDolly;
    } else {
      ui_state.camera_drag_mode = CameraDragMode::kOrbit;
    }
    DisableCursor();
  }

  if (ui_state.camera_drag_mode != CameraDragMode::kNone) {
    if (ui_state.camera_drag_mode == CameraDragMode::kOrbit) {
      OrbitCameraTurntable(camera, mouse_delta, 0.006f);
    } else if (ui_state.camera_drag_mode == CameraDragMode::kPan) {
      PanCamera(camera, mouse_delta, 0.002f);
    } else if (ui_state.camera_drag_mode == CameraDragMode::kDolly) {
      DollyCamera(camera, mouse_delta.y, 0.01f);
    }

    if (IsMouseButtonReleased(MOUSE_BUTTON_MIDDLE)) {
      ui_state.camera_drag_mode = CameraDragMode::kNone;
      EnableCursor();
    }
  }

  if (!ui_captures_mouse && !ui_captures_keyboard && ui_state.camera_drag_mode == CameraDragMode::kNone) {
    const float wheel = GetMouseWheelMove();
    if (std::fabs(wheel) > 0.0f) {
      DollyCamera(camera, -wheel, 0.12f);
    }
  }
}

} // namespace

int main() {
  const ViewerPersistentSettings persisted = LoadViewerPersistentSettings();
  SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_MSAA_4X_HINT | FLAG_VSYNC_HINT);
  InitWindow(persisted.window_width, persisted.window_height, "wire viewer");
  SetExitKey(KEY_NULL);
  SetTargetFPS(60);

  Camera3D camera{};
  camera.position = ToRaylib({10.0, -10.0, 8.0});
  camera.target = ToRaylib({6.0, 0.0, 4.0});
  camera.up = {0.0f, 1.0f, 0.0f};
  camera.fovy = persisted.camera_fov_deg;
  camera.projection = CAMERA_PERSPECTIVE;

  CoreState state = wire::core::make_demo_state();
  ViewerUiState ui_state;
  ui_state.ui_unified_workspace = persisted.ui_unified_workspace;
  ui_state.ui_show_workspace = persisted.ui_show_workspace;
  ui_state.ui_workspace_width = persisted.ui_workspace_width;
  ui_state.camera_fov_deg = persisted.camera_fov_deg;
  ui_state.camera_walk_speed = persisted.camera_walk_speed;
  PushLog(ui_state, "[info] viewer started");
  PushLog(ui_state, "[info] demo state loaded");
  PushLog(ui_state, "[mode] Placement/Connection/Branch/Detail/DrawPath");
  PushLog(ui_state, "[flow] Main path: Pole->Pole connection");
  PushLog(ui_state, "[hint] Blender style controls enabled");
  PushLog(ui_state, "[hint] MMB orbit, Shift+MMB pan, Ctrl+MMB dolly");
  PushLog(ui_state, "[hint] Alt+LMB: set orbit pivot, [ / ]: FOV, Shift+F: Walk");
  PushLog(ui_state, "[hint] Mouse wheel zoom");

  rlImGuiSetup(true);
  ImGui::StyleColorsDark();
  {
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding = 3.0f;
    style.FrameRounding = 2.0f;
    style.GrabRounding = 2.0f;
    style.WindowBorderSize = 1.0f;
    style.FrameBorderSize = 0.0f;
  }
  while (!WindowShouldClose()) {
    BeginDrawing();
    ClearBackground(Color{26, 32, 39, 255});

    rlImGuiBegin();
    UpdateCameraForViewport(&camera, ui_state);
    UpdateDrawPathInput(state, camera, ui_state);
    UpdateBranchPickInput(state, camera, ui_state);
    if (ui_state.auto_recalc) {
      wire::core::CommitOptions options{};
      options.run_recalc = true;
      options.run_validate = false;
      (void)state.Commit(options);
    }

    BeginMode3D(camera);
    DrawGrid(40, 1.0f);
    DrawAxes();
    DrawCore(state, ui_state);
    DrawPathPreview(ui_state);
    EndMode3D();

    DrawStatsPanel(state, ui_state);
    rlImGuiEnd();

    DrawFPS(10, 10);
    EndDrawing();
  }

  rlImGuiShutdown();
  {
    ViewerPersistentSettings out{};
    out.window_width = GetScreenWidth();
    out.window_height = GetScreenHeight();
    out.ui_unified_workspace = ui_state.ui_unified_workspace;
    out.ui_show_workspace = ui_state.ui_show_workspace;
    out.ui_workspace_width = ui_state.ui_workspace_width;
    out.camera_fov_deg = ui_state.camera_fov_deg;
    out.camera_walk_speed = ui_state.camera_walk_speed;
    SaveViewerPersistentSettings(out);
  }
  CloseWindow();
  return 0;
}




