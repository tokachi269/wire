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
#include "wire/core/coord_utils.hpp"
#include "wire/core/core_state.hpp"

namespace {

constexpr float kAxisLength = 2.0f;
constexpr float kSelectionClickRadiusPx = 14.0f;
constexpr float kSelectionDragThresholdPx = 6.0f;
constexpr int kGroundGridHalfExtent = 20;
constexpr float kGroundGridSpacing = 1.0f;
constexpr Color kSkyTopColor = Color{142, 182, 220, 255};
constexpr Color kSkyBottomColor = Color{200, 214, 228, 255};

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

wire::core::Vec3d PoleTopPoint(const wire::core::Pole& pole) {
  const wire::core::PoleFrame frame =
      wire::core::BuildPoleFrame(pole.world_transform, pole.world_transform.rotation_euler_deg.z);
  return wire::core::LocalPointToWorld(frame, wire::core::ScaleVec(wire::core::WorldUp(), pole.height_m));
}

void DrawGroundGrid() {
  const Color minor = Color{114, 136, 154, 78};
  const Color major = Color{88, 114, 136, 128};
  for (int i = -kGroundGridHalfExtent; i <= kGroundGridHalfExtent; ++i) {
    const double coord = static_cast<double>(i) * kGroundGridSpacing;
    const Color color = (i == 0) ? major : minor;
    DrawLine3D(ToRaylib({coord, -kGroundGridHalfExtent * kGroundGridSpacing, 0.0}),
               ToRaylib({coord, +kGroundGridHalfExtent * kGroundGridSpacing, 0.0}), color);
    DrawLine3D(ToRaylib({-kGroundGridHalfExtent * kGroundGridSpacing, coord, 0.0}),
               ToRaylib({+kGroundGridHalfExtent * kGroundGridSpacing, coord, 0.0}), color);
  }
}

void DrawSceneBackdrop() {
  const int width = GetScreenWidth();
  const int height = GetScreenHeight();
  DrawRectangleGradientV(0, 0, width, height, kSkyTopColor, kSkyBottomColor);
}

bool IsSelectionViewportMode(const ViewerUiState& ui_state) {
  return ui_state.mode != EditMode::kBranch && ui_state.mode != EditMode::kDrawPath;
}

Rectangle SelectionRectangle(const DragSelectionState& drag_selection) {
  const float x = std::min(drag_selection.start_screen.x, drag_selection.end_screen.x);
  const float y = std::min(drag_selection.start_screen.y, drag_selection.end_screen.y);
  const float width = std::fabs(drag_selection.end_screen.x - drag_selection.start_screen.x);
  const float height = std::fabs(drag_selection.end_screen.y - drag_selection.start_screen.y);
  return Rectangle{x, y, width, height};
}

bool HasMeaningfulDragSelection(const DragSelectionState& drag_selection) {
  return std::fabs(drag_selection.end_screen.x - drag_selection.start_screen.x) >= kSelectionDragThresholdPx ||
         std::fabs(drag_selection.end_screen.y - drag_selection.start_screen.y) >= kSelectionDragThresholdPx;
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

bool IsHostPointInFrontOfCamera(const Camera3D& camera, const Vector3& host_point) {
  const Vector3 to_point{
      host_point.x - camera.position.x,
      host_point.y - camera.position.y,
      host_point.z - camera.position.z,
  };
  const Vector3 forward{
      camera.target.x - camera.position.x,
      camera.target.y - camera.position.y,
      camera.target.z - camera.position.z,
  };
  const float dot = to_point.x * forward.x + to_point.y * forward.y + to_point.z * forward.z;
  return dot > 0.0f;
}

bool TryProjectWorldPoint(const Camera3D& camera, const wire::core::Vec3d& world, Vector2* out_screen) {
  if (out_screen == nullptr) {
    return false;
  }
  const Vector3 host = ToRaylib(world);
  if (!IsHostPointInFrontOfCamera(camera, host)) {
    return false;
  }
  *out_screen = GetWorldToScreen(host, camera);
  return true;
}

float DistancePointToSegmentSquared(const Vector2& point, const Vector2& a, const Vector2& b) {
  const float ab_x = b.x - a.x;
  const float ab_y = b.y - a.y;
  const float ap_x = point.x - a.x;
  const float ap_y = point.y - a.y;
  const float ab_len_sq = ab_x * ab_x + ab_y * ab_y;
  if (ab_len_sq <= 1e-6f) {
    return ap_x * ap_x + ap_y * ap_y;
  }
  const float t = std::clamp((ap_x * ab_x + ap_y * ab_y) / ab_len_sq, 0.0f, 1.0f);
  const float closest_x = a.x + ab_x * t;
  const float closest_y = a.y + ab_y * t;
  const float dx = point.x - closest_x;
  const float dy = point.y - closest_y;
  return dx * dx + dy * dy;
}

float Cross2d(const Vector2& a, const Vector2& b, const Vector2& c) {
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

bool OnSegment2d(const Vector2& a, const Vector2& b, const Vector2& p) {
  const float min_x = std::min(a.x, b.x) - 1e-3f;
  const float max_x = std::max(a.x, b.x) + 1e-3f;
  const float min_y = std::min(a.y, b.y) - 1e-3f;
  const float max_y = std::max(a.y, b.y) + 1e-3f;
  return p.x >= min_x && p.x <= max_x && p.y >= min_y && p.y <= max_y;
}

bool SegmentsIntersect2d(const Vector2& a, const Vector2& b, const Vector2& c, const Vector2& d) {
  const float ab_c = Cross2d(a, b, c);
  const float ab_d = Cross2d(a, b, d);
  const float cd_a = Cross2d(c, d, a);
  const float cd_b = Cross2d(c, d, b);
  if (((ab_c > 0.0f && ab_d < 0.0f) || (ab_c < 0.0f && ab_d > 0.0f)) &&
      ((cd_a > 0.0f && cd_b < 0.0f) || (cd_a < 0.0f && cd_b > 0.0f))) {
    return true;
  }
  if (std::fabs(ab_c) <= 1e-3f && OnSegment2d(a, b, c)) return true;
  if (std::fabs(ab_d) <= 1e-3f && OnSegment2d(a, b, d)) return true;
  if (std::fabs(cd_a) <= 1e-3f && OnSegment2d(c, d, a)) return true;
  if (std::fabs(cd_b) <= 1e-3f && OnSegment2d(c, d, b)) return true;
  return false;
}

bool RectangleContainsPoint(const Rectangle& rect, const Vector2& point) {
  return CheckCollisionPointRec(point, rect);
}

bool RectangleIntersectsSegment(const Rectangle& rect, const Vector2& a, const Vector2& b) {
  if (RectangleContainsPoint(rect, a) || RectangleContainsPoint(rect, b)) {
    return true;
  }
  const Vector2 tl{rect.x, rect.y};
  const Vector2 tr{rect.x + rect.width, rect.y};
  const Vector2 br{rect.x + rect.width, rect.y + rect.height};
  const Vector2 bl{rect.x, rect.y + rect.height};
  return SegmentsIntersect2d(a, b, tl, tr) || SegmentsIntersect2d(a, b, tr, br) ||
         SegmentsIntersect2d(a, b, br, bl) || SegmentsIntersect2d(a, b, bl, tl);
}

std::vector<Vector2> ProjectSpanPolyline(const CoreState& state, const wire::core::EditState& edit, const Camera3D& camera,
                                         const wire::core::Span& span) {
  std::vector<Vector2> projected{};
  if (const wire::core::CurveCacheEntry* curve = state.find_curve_cache(span.id);
      curve != nullptr && curve->points.size() >= 2) {
    projected.reserve(curve->points.size());
    for (const wire::core::Vec3d& point : curve->points) {
      Vector2 screen{};
      if (TryProjectWorldPoint(camera, point, &screen)) {
        projected.push_back(screen);
      }
    }
    if (projected.size() >= 2) {
      return projected;
    }
  }
  const wire::core::Port* port_a = edit.ports.find(span.port_a_id);
  const wire::core::Port* port_b = edit.ports.find(span.port_b_id);
  if (port_a == nullptr || port_b == nullptr) {
    return projected;
  }
  Vector2 a{};
  Vector2 b{};
  if (!TryProjectWorldPoint(camera, port_a->world_position, &a) || !TryProjectWorldPoint(camera, port_b->world_position, &b)) {
    return {};
  }
  return {a, b};
}

float DistanceToProjectedPolylineSquared(const std::vector<Vector2>& polyline, const Vector2& point) {
  if (polyline.size() < 2) {
    return std::numeric_limits<float>::max();
  }
  float best = std::numeric_limits<float>::max();
  for (std::size_t i = 0; i + 1 < polyline.size(); ++i) {
    best = std::min(best, DistancePointToSegmentSquared(point, polyline[i], polyline[i + 1]));
  }
  return best;
}

bool PolylineIntersectsRectangle(const std::vector<Vector2>& polyline, const Rectangle& rect) {
  if (polyline.size() < 2) {
    return false;
  }
  for (std::size_t i = 0; i + 1 < polyline.size(); ++i) {
    if (RectangleIntersectsSegment(rect, polyline[i], polyline[i + 1])) {
      return true;
    }
  }
  return false;
}

SelectionItem PickViewportSelection(const CoreState& state, const Camera3D& camera, const ViewerUiState& ui_state,
                                    const Vector2& mouse_screen) {
  const auto view = state.view();
  const auto& edit = view.edit_state();
  const wire::core::BackboneResult backbone = state.BuildBackboneResult();
  SelectionItem best{};
  float best_distance_sq = kSelectionClickRadiusPx * kSelectionClickRadiusPx;

  if (ui_state.selection_include_poles) {
    for (const wire::core::Pole& pole : edit.poles.items()) {
      Vector2 base{};
      Vector2 top{};
      if (!TryProjectWorldPoint(camera, pole.world_transform.position, &base) ||
          !TryProjectWorldPoint(camera, PoleTopPoint(pole), &top)) {
        continue;
      }
      const float d2 = DistancePointToSegmentSquared(mouse_screen, base, top);
      if (d2 < best_distance_sq) {
        best = {SelectedType::kPole, pole.id};
        best_distance_sq = d2;
      }
    }
  }

  if (ui_state.selection_include_midair_nodes) {
    for (const wire::core::SupportNode& node : backbone.nodes) {
      if (node.support_kind != wire::core::SupportKind::kMidair) {
        continue;
      }
      Vector2 screen{};
      if (!TryProjectWorldPoint(camera, node.position, &screen)) {
        continue;
      }
      const float dx = mouse_screen.x - screen.x;
      const float dy = mouse_screen.y - screen.y;
      const float d2 = dx * dx + dy * dy;
      if (d2 < best_distance_sq) {
        best = {SelectedType::kSupportNode, node.node_id};
        best_distance_sq = d2;
      }
    }
  }

  if (ui_state.selection_include_spans) {
    for (const wire::core::Span& span : edit.spans.items()) {
      const std::vector<Vector2> polyline = ProjectSpanPolyline(state, edit, camera, span);
      const float d2 = DistanceToProjectedPolylineSquared(polyline, mouse_screen);
      if (d2 < best_distance_sq) {
        best = {SelectedType::kSpan, span.id};
        best_distance_sq = d2;
      }
    }
  }

  return best;
}

std::vector<SelectionItem> CollectViewportSelection(const CoreState& state, const Camera3D& camera,
                                                    const ViewerUiState& ui_state, const Rectangle& rect) {
  const auto view = state.view();
  const auto& edit = view.edit_state();
  const wire::core::BackboneResult backbone = state.BuildBackboneResult();
  std::vector<SelectionItem> items{};

  if (ui_state.selection_include_poles) {
    for (const wire::core::Pole& pole : edit.poles.items()) {
      Vector2 base{};
      Vector2 top{};
      if (!TryProjectWorldPoint(camera, pole.world_transform.position, &base) ||
          !TryProjectWorldPoint(camera, PoleTopPoint(pole), &top)) {
        continue;
      }
      if (RectangleIntersectsSegment(rect, base, top)) {
        items.push_back({SelectedType::kPole, pole.id});
      }
    }
  }

  if (ui_state.selection_include_midair_nodes) {
    for (const wire::core::SupportNode& node : backbone.nodes) {
      if (node.support_kind != wire::core::SupportKind::kMidair) {
        continue;
      }
      Vector2 screen{};
      if (TryProjectWorldPoint(camera, node.position, &screen) && RectangleContainsPoint(rect, screen)) {
        items.push_back({SelectedType::kSupportNode, node.node_id});
      }
    }
  }

  if (ui_state.selection_include_spans) {
    for (const wire::core::Span& span : edit.spans.items()) {
      const std::vector<Vector2> polyline = ProjectSpanPolyline(state, edit, camera, span);
      if (PolylineIntersectsRectangle(polyline, rect)) {
        items.push_back({SelectedType::kSpan, span.id});
      }
    }
  }

  return items;
}

void DrawDragSelectionOverlay(const ViewerUiState& ui_state) {
  if (!ui_state.drag_selection.active) {
    return;
  }
  const Rectangle rect = SelectionRectangle(ui_state.drag_selection);
  DrawRectangleRec(rect, Color{255, 215, 90, 36});
  DrawRectangleLinesEx(rect, 1.5f, Color{255, 225, 120, 220});
}

void UpdateViewportSelectionInput(const CoreState& state, const Camera3D& camera, ViewerUiState& ui_state) {
  ImGuiIO& io = ImGui::GetIO();
  if (!IsSelectionViewportMode(ui_state) || io.WantCaptureMouse || ui_state.camera_walk_mode ||
      ui_state.camera_drag_mode != CameraDragMode::kNone) {
    ui_state.drag_selection.active = false;
    return;
  }

  const bool shift = IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT);
  const bool alt = IsKeyDown(KEY_LEFT_ALT) || IsKeyDown(KEY_RIGHT_ALT);
  if (alt) {
    return;
  }

  const Vector2 mouse_screen = GetMousePosition();
  if (!ui_state.drag_selection.active && shift && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
    ui_state.drag_selection.active = true;
    ui_state.drag_selection.start_screen = mouse_screen;
    ui_state.drag_selection.end_screen = mouse_screen;
    return;
  }

  if (ui_state.drag_selection.active) {
    ui_state.drag_selection.end_screen = mouse_screen;
    if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
      const bool used_box = HasMeaningfulDragSelection(ui_state.drag_selection);
      const Rectangle rect = SelectionRectangle(ui_state.drag_selection);
      ui_state.drag_selection.active = false;
      if (used_box) {
        ReplaceSelection(ui_state, CollectViewportSelection(state, camera, ui_state, rect));
        PushLog(ui_state, "Box select count=" + std::to_string(static_cast<unsigned long long>(ui_state.selection_items.size())));
      } else {
        const SelectionItem picked = PickViewportSelection(state, camera, ui_state, mouse_screen);
        if (IsValidSelectionItem(picked)) {
          SetPrimarySelection(ui_state, picked.type, picked.id);
        } else {
          ClearSelection(ui_state);
        }
      }
    }
    return;
  }

  if (!shift && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
    const SelectionItem picked = PickViewportSelection(state, camera, ui_state, mouse_screen);
    if (IsValidSelectionItem(picked)) {
      SetPrimarySelection(ui_state, picked.type, picked.id);
    } else {
      ClearSelection(ui_state);
    }
  }
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

  CoreState state{};
  ViewerUiState ui_state;
  ui_state.ui_unified_workspace = persisted.ui_unified_workspace;
  ui_state.ui_show_workspace = persisted.ui_show_workspace;
  ui_state.ui_workspace_width = persisted.ui_workspace_width;
  ui_state.camera_fov_deg = persisted.camera_fov_deg;
  ui_state.camera_walk_speed = persisted.camera_walk_speed;
  PushLog(ui_state, "[info] viewer started");
  PushLog(ui_state, "[info] empty state loaded");
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
    ClearBackground(kSkyBottomColor);
    DrawSceneBackdrop();

    rlImGuiBegin();
    UpdateCameraForViewport(&camera, ui_state);
    UpdateViewportSelectionInput(state, camera, ui_state);
    UpdateDrawPathInput(state, camera, ui_state);
    UpdateBranchPickInput(state, camera, ui_state);
    if (ui_state.auto_recalc) {
      wire::core::CommitOptions options{};
      options.run_recalc = true;
      options.run_validate = false;
      (void)state.Commit(options);
    }
    UpdatePreferredVisibleSpans(state, camera, ui_state);

    BeginMode3D(camera);
    DrawGroundGrid();
    DrawAxes();
    DrawCore(state, ui_state);
    DrawPathPreview(ui_state);
    EndMode3D();

    DrawStatsPanel(state, ui_state);
    rlImGuiEnd();
    DrawDragSelectionOverlay(ui_state);

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




