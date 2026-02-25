#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

#include "imgui.h"
#include "raylib.h"
#include "raymath.h"
#include "rlImGui.h"
#include "wire/core/core_state.hpp"

namespace {

using wire::core::CoreState;
using wire::core::ObjectId;

constexpr float kAxisLength = 2.0f;

enum class SelectedType {
  kNone = 0,
  kPole = 1,
  kPort = 2,
  kSpan = 3,
  kAnchor = 4,
  kBundle = 5,
  kAttachment = 6,
};

enum class CameraDragMode {
  kNone = 0,
  kOrbit = 1,
  kPan = 2,
  kDolly = 3,
};

enum class EditMode {
  kPlacement = 0,
  kConnection = 1,
  kBranch = 2,
  kDetail = 3,
  kDrawPath = 4,
};

struct ViewerUiState {
  EditMode mode = EditMode::kPlacement;
  SelectedType selected_type = SelectedType::kNone;
  ObjectId selected_id = wire::core::kInvalidObjectId;

  double pole_x = 0.0;
  double pole_y = 0.0;
  double pole_z = 0.0;
  double pole_height = 9.0;
  int pole_kind = static_cast<int>(wire::core::PoleKind::kWood);

  ObjectId port_owner_pole_id = wire::core::kInvalidObjectId;
  double port_x = 0.0;
  double port_y = 0.0;
  double port_z = 7.0;
  int port_kind = static_cast<int>(wire::core::PortKind::kPower);
  int port_layer = static_cast<int>(wire::core::PortLayer::kLowVoltage);

  ObjectId span_port_a_id = wire::core::kInvalidObjectId;
  ObjectId span_port_b_id = wire::core::kInvalidObjectId;
  ObjectId span_bundle_id = wire::core::kInvalidObjectId;
  int span_kind = static_cast<int>(wire::core::SpanKind::kDistribution);
  int span_layer = static_cast<int>(wire::core::SpanLayer::kLowVoltage);
  double split_t = 0.5;

  int placement_pole_type_index = 0;
  ObjectId connect_pole_a_id = wire::core::kInvalidObjectId;
  ObjectId connect_pole_b_id = wire::core::kInvalidObjectId;
  int connect_category = static_cast<int>(wire::core::ConnectionCategory::kLowVoltage);
  int connect_context = static_cast<int>(wire::core::ConnectionContext::kTrunkContinue);
  ObjectId branch_source_span_id = wire::core::kInvalidObjectId;
  double branch_t = 0.5;
  double branch_target_x = 0.0;
  double branch_target_y = 0.0;
  double branch_target_z = 3.0;
  ObjectId drop_source_pole_id = wire::core::kInvalidObjectId;
  double drop_target_x = 0.0;
  double drop_target_y = 4.0;
  double drop_target_z = 3.0;
  int detail_pole_type_index = 0;
  bool show_debug_labels = false;
  bool show_whole_aabb = false;
  bool show_segment_aabb = false;
  bool geometry_settings_loaded = false;
  int geometry_samples = 8;
  bool geometry_sag_enabled = false;
  double geometry_sag_factor = 0.03;
  std::uint64_t road_id = 1;
  double road_start_x = -20.0;
  double road_start_y = 0.0;
  double road_start_z = 0.0;
  bool road_use_mid = false;
  double road_mid_x = 0.0;
  double road_mid_y = 0.0;
  double road_mid_z = 0.0;
  double road_end_x = 20.0;
  double road_end_y = 0.0;
  double road_end_z = 0.0;
  double road_interval = 8.0;
  int road_category_index = static_cast<int>(wire::core::ConnectionCategory::kLowVoltage);
  int road_pole_type_index = 0;
  int last_generated_poles = 0;
  int last_generated_spans = 0;
  std::uint64_t last_generation_session = 0;
  std::vector<wire::core::Vec3d> draw_path_points{};
  bool draw_hover_valid = false;
  wire::core::Vec3d draw_hover_point{};
  double draw_plane_z = 0.0;
  bool draw_show_preview = true;
  bool draw_keep_path_after_generate = true;
  int draw_parallel_spans = 0;  // 0 = auto by category
  int draw_direction_mode = static_cast<int>(wire::core::PathDirectionMode::kAuto);
  bool layout_settings_loaded = false;
  bool layout_angle_correction_enabled = true;
  double layout_corner_threshold_deg = 12.0;
  double layout_min_side_scale = 1.0;
  double layout_max_side_scale = 1.8;
  int selected_slot_debug_index = 0;

  std::string last_error;
  std::vector<std::string> logs;
  CameraDragMode camera_drag_mode = CameraDragMode::kNone;
  bool auto_recalc = true;
  double edit_x = 0.0;
  double edit_y = 0.0;
  double edit_z = 0.0;
};

void PushLog(ViewerUiState& ui_state, const std::string& line);

constexpr std::array<wire::core::ConnectionCategory, 5> kAllCategories = {
    wire::core::ConnectionCategory::kHighVoltage,
    wire::core::ConnectionCategory::kLowVoltage,
    wire::core::ConnectionCategory::kCommunication,
    wire::core::ConnectionCategory::kOptical,
    wire::core::ConnectionCategory::kDrop,
};

constexpr std::array<wire::core::ConnectionContext, 4> kAllConnectionContexts = {
    wire::core::ConnectionContext::kTrunkContinue,
    wire::core::ConnectionContext::kCornerPass,
    wire::core::ConnectionContext::kBranchAdd,
    wire::core::ConnectionContext::kDropAdd,
};

const char* CategoryLabel(wire::core::ConnectionCategory category) {
  switch (category) {
    case wire::core::ConnectionCategory::kHighVoltage:
      return "HighVoltage";
    case wire::core::ConnectionCategory::kLowVoltage:
      return "LowVoltage";
    case wire::core::ConnectionCategory::kCommunication:
      return "Communication";
    case wire::core::ConnectionCategory::kOptical:
      return "Optical";
    case wire::core::ConnectionCategory::kDrop:
      return "Drop";
    default:
      return "Unknown";
  }
}

const char* ContextLabel(wire::core::ConnectionContext context) {
  switch (context) {
    case wire::core::ConnectionContext::kTrunkContinue:
      return "Trunk";
    case wire::core::ConnectionContext::kCornerPass:
      return "Corner";
    case wire::core::ConnectionContext::kBranchAdd:
      return "Branch";
    case wire::core::ConnectionContext::kDropAdd:
      return "Drop";
    default:
      return "Unknown";
  }
}

const char* PoleContextLabel(wire::core::PoleContextKind context) {
  switch (context) {
    case wire::core::PoleContextKind::kStraight:
      return "Straight";
    case wire::core::PoleContextKind::kCorner:
      return "Corner";
    case wire::core::PoleContextKind::kBranch:
      return "Branch";
    case wire::core::PoleContextKind::kTerminal:
      return "Terminal";
    default:
      return "Unknown";
  }
}

const char* SlotSideLabel(wire::core::SlotSide side) {
  switch (side) {
    case wire::core::SlotSide::kLeft:
      return "L";
    case wire::core::SlotSide::kCenter:
      return "C";
    case wire::core::SlotSide::kRight:
      return "R";
    default:
      return "?";
  }
}

const char* SlotRoleLabel(wire::core::SlotRole role) {
  switch (role) {
    case wire::core::SlotRole::kNeutral:
      return "Neutral";
    case wire::core::SlotRole::kTrunkPreferred:
      return "Trunk";
    case wire::core::SlotRole::kBranchPreferred:
      return "Branch";
    case wire::core::SlotRole::kDropPreferred:
      return "Drop";
    default:
      return "Unknown";
  }
}

const char* PathDirectionModeLabel(wire::core::PathDirectionMode mode) {
  switch (mode) {
    case wire::core::PathDirectionMode::kAuto:
      return "Auto";
    case wire::core::PathDirectionMode::kForward:
      return "Forward";
    case wire::core::PathDirectionMode::kReverse:
      return "Reverse";
    default:
      return "Unknown";
  }
}

const char* PathDirectionChosenLabel(wire::core::PathDirectionChosen chosen) {
  switch (chosen) {
    case wire::core::PathDirectionChosen::kForward:
      return "Forward";
    case wire::core::PathDirectionChosen::kReverse:
      return "Reverse";
    default:
      return "Unknown";
  }
}

const char* ModeLabel(EditMode mode) {
  switch (mode) {
    case EditMode::kPlacement:
      return "Placement";
    case EditMode::kConnection:
      return "Connection";
    case EditMode::kBranch:
      return "Branch";
    case EditMode::kDetail:
      return "Detail";
    case EditMode::kDrawPath:
      return "DrawPath";
    default:
      return "Unknown";
  }
}

int FallbackParallelSpanCount(wire::core::ConnectionCategory category) {
  switch (category) {
    case wire::core::ConnectionCategory::kHighVoltage:
      return 3;
    case wire::core::ConnectionCategory::kLowVoltage:
      return 4;
    case wire::core::ConnectionCategory::kCommunication:
      return 4;
    case wire::core::ConnectionCategory::kOptical:
      return 2;
    case wire::core::ConnectionCategory::kDrop:
      return 1;
    default:
      return 1;
  }
}

int AutoParallelSpanCountFromPoleType(
    const CoreState& state,
    wire::core::PoleTypeId pole_type_id,
    wire::core::ConnectionCategory category) {
  const auto it = state.pole_types().find(pole_type_id);
  if (it == state.pole_types().end()) {
    return FallbackParallelSpanCount(category);
  }
  int count = 0;
  for (const auto& slot : it->second.port_slots) {
    if (slot.enabled && slot.category == category) {
      ++count;
    }
  }
  return (count > 0) ? count : FallbackParallelSpanCount(category);
}

std::vector<wire::core::PoleTypeId> SortedPoleTypeIds(const CoreState& state) {
  std::vector<wire::core::PoleTypeId> ids;
  ids.reserve(state.pole_types().size());
  for (const auto& [id, _] : state.pole_types()) {
    ids.push_back(id);
  }
  std::sort(ids.begin(), ids.end());
  return ids;
}

std::size_t ClampedTypeIndex(int current, std::size_t count) {
  if (count == 0) {
    return 0;
  }
  const int max_index = static_cast<int>(count - 1);
  return static_cast<std::size_t>(std::clamp(current, 0, max_index));
}

Vector3 ToRaylib(const wire::core::Vec3d& ue_xyz) {
  return Vector3{
      static_cast<float>(ue_xyz.x),
      static_cast<float>(ue_xyz.z),
      static_cast<float>(ue_xyz.y),
  };
}

BoundingBox ToRaylibBounds(const wire::core::AABBd& box_ue) {
  const Vector3 a = ToRaylib(box_ue.min);
  const Vector3 b = ToRaylib(box_ue.max);
  BoundingBox out{};
  out.min = {
      std::min(a.x, b.x),
      std::min(a.y, b.y),
      std::min(a.z, b.z),
  };
  out.max = {
      std::max(a.x, b.x),
      std::max(a.y, b.y),
      std::max(a.z, b.z),
  };
  return out;
}

double PolylineLength(const std::vector<wire::core::Vec3d>& points) {
  if (points.size() < 2) {
    return 0.0;
  }
  double length = 0.0;
  for (std::size_t i = 0; i + 1 < points.size(); ++i) {
    const wire::core::Vec3d d = points[i + 1] - points[i];
    length += std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
  }
  return length;
}

wire::core::Vec3d FromRaylib(const Vector3& raylib_xyz) {
  return wire::core::Vec3d{
      static_cast<double>(raylib_xyz.x),
      static_cast<double>(raylib_xyz.z),
      static_cast<double>(raylib_xyz.y),
  };
}

bool TryPickGroundPoint(const Camera3D& camera, double ue_plane_z, wire::core::Vec3d* out_ue_point) {
  if (out_ue_point == nullptr) {
    return false;
  }
  const Vector2 mouse = GetMousePosition();
  const Ray ray = GetMouseRay(mouse, camera);
  if (std::abs(ray.direction.y) <= 1e-6f) {
    return false;
  }
  const float plane_y = static_cast<float>(ue_plane_z);  // raylib y == UE z
  const float t = (plane_y - ray.position.y) / ray.direction.y;
  if (t < 0.0f) {
    return false;
  }
  const Vector3 hit{
      ray.position.x + ray.direction.x * t,
      ray.position.y + ray.direction.y * t,
      ray.position.z + ray.direction.z * t,
  };
  *out_ue_point = FromRaylib(hit);
  return true;
}

void ExecuteGenerateFromDrawPath(CoreState& state, ViewerUiState& ui_state, bool from_enter_key) {
  if (ui_state.draw_path_points.size() < 2) {
    ui_state.last_error = "path needs at least 2 points";
    return;
  }

  const auto type_ids = SortedPoleTypeIds(state);
  if (type_ids.empty()) {
    ui_state.last_error = "no pole type available";
    return;
  }

  const std::size_t road_type_index = ClampedTypeIndex(ui_state.road_pole_type_index, type_ids.size());
  ui_state.road_pole_type_index = static_cast<int>(road_type_index);
  const int road_category_index = std::clamp(ui_state.road_category_index, 0, static_cast<int>(kAllCategories.size() - 1));
  ui_state.road_category_index = road_category_index;

  wire::core::RoadSegment road{};
  road.id = ui_state.road_id++;
  road.polyline = ui_state.draw_path_points;

  const wire::core::ConnectionCategory category = kAllCategories[ui_state.road_category_index];
  int parallel_spans = ui_state.draw_parallel_spans;
  if (parallel_spans <= 0) {
    parallel_spans =
        AutoParallelSpanCountFromPoleType(state, type_ids[road_type_index], category);
  }
  parallel_spans = std::clamp(parallel_spans, 1, 8);
  const int mode_index = std::clamp(ui_state.draw_direction_mode, 0, 2);
  ui_state.draw_direction_mode = mode_index;

  wire::core::CoreState::GenerateGroupedLineOptions options{};
  options.road = road;
  options.interval = 0.0;  // DrawPath uses clicked points directly.
  options.pole_type_id = type_ids[ui_state.road_pole_type_index];
  options.direction_mode = static_cast<wire::core::PathDirectionMode>(mode_index);
  options.group_spec.category = category;
  options.group_spec.conductor_count = parallel_spans;
  options.group_spec.group_kind =
      (parallel_spans <= 1)
          ? wire::core::ConductorGroupKind::kSingle
          : ((category == wire::core::ConnectionCategory::kHighVoltage && parallel_spans == 3)
                 ? wire::core::ConductorGroupKind::kThreePhase
                 : wire::core::ConductorGroupKind::kParallel);
  options.group_spec.lane_spacing_m =
      (category == wire::core::ConnectionCategory::kHighVoltage) ? 0.45 : 0.2;
  options.group_spec.maintain_lane_order = true;
  options.group_spec.allow_lane_mirror = true;

  const auto result = state.GenerateGroupedLine(options);
  if (!result.ok) {
    ui_state.last_error = result.error;
    PushLog(ui_state, from_enter_key ? "Generate path (Enter) failed" : "Generate path failed");
    return;
  }
  ui_state.last_error.clear();
  ui_state.last_generated_poles = static_cast<int>(result.value.pole_ids.size());
  ui_state.last_generated_spans = static_cast<int>(result.value.span_ids.size());
  ui_state.last_generation_session = result.value.generation_session_id;
  if (!result.value.pole_ids.empty()) {
    ui_state.selected_type = SelectedType::kPole;
    ui_state.selected_id = result.value.pole_ids.back();
  }
  if (!ui_state.draw_keep_path_after_generate) {
    ui_state.draw_path_points.clear();
  }
  PushLog(
      ui_state,
      "Generated path poles=" + std::to_string(ui_state.last_generated_poles) +
          " spans=" + std::to_string(ui_state.last_generated_spans) +
          " lanes=" + std::to_string(parallel_spans) +
          " dir=" + PathDirectionChosenLabel(result.value.direction_debug.chosen));
}

void UpdateDrawPathInput(CoreState& state, const Camera3D& camera, ViewerUiState& ui_state) {
  if (ui_state.mode != EditMode::kDrawPath) {
    ui_state.draw_hover_valid = false;
    return;
  }

  ImGuiIO& io = ImGui::GetIO();
  wire::core::Vec3d hover{};
  ui_state.draw_hover_valid = TryPickGroundPoint(camera, ui_state.draw_plane_z, &hover);
  if (ui_state.draw_hover_valid) {
    ui_state.draw_hover_point = hover;
  }

  if (!io.WantCaptureMouse && ui_state.camera_drag_mode == CameraDragMode::kNone) {
    if (ui_state.draw_hover_valid && IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
      ui_state.draw_path_points.push_back(ui_state.draw_hover_point);
    }
    if (IsMouseButtonPressed(MOUSE_RIGHT_BUTTON) && !ui_state.draw_path_points.empty()) {
      ui_state.draw_path_points.pop_back();
    }
  }

  if (!io.WantCaptureKeyboard) {
    if (IsKeyPressed(KEY_BACKSPACE) && !ui_state.draw_path_points.empty()) {
      ui_state.draw_path_points.pop_back();
    }
    if (IsKeyPressed(KEY_ESCAPE)) {
      ui_state.draw_path_points.clear();
    }
    if (IsKeyPressed(KEY_ENTER) || IsKeyPressed(KEY_KP_ENTER)) {
      ExecuteGenerateFromDrawPath(state, ui_state, true);
    }
  }
}

void DrawPathPreview(const ViewerUiState& ui_state) {
  if (!ui_state.draw_show_preview || ui_state.mode != EditMode::kDrawPath) {
    return;
  }
  if (ui_state.draw_path_points.empty() && !ui_state.draw_hover_valid) {
    return;
  }

  for (std::size_t i = 0; i < ui_state.draw_path_points.size(); ++i) {
    DrawSphere(ToRaylib(ui_state.draw_path_points[i]), 0.12f, Color{255, 200, 0, 255});
    if (i + 1 < ui_state.draw_path_points.size()) {
      DrawLine3D(ToRaylib(ui_state.draw_path_points[i]), ToRaylib(ui_state.draw_path_points[i + 1]), Color{255, 220, 80, 255});
    }
  }
  if (ui_state.draw_hover_valid) {
    DrawSphere(ToRaylib(ui_state.draw_hover_point), 0.08f, Color{120, 255, 180, 200});
    if (!ui_state.draw_path_points.empty()) {
      DrawLine3D(
          ToRaylib(ui_state.draw_path_points.back()),
          ToRaylib(ui_state.draw_hover_point),
          Color{120, 255, 180, 180});
    }
  }
}

wire::core::Vec3d Lerp(const wire::core::Vec3d& a, const wire::core::Vec3d& b, double t) {
  return wire::core::Vec3d{
      a.x + (b.x - a.x) * t,
      a.y + (b.y - a.y) * t,
      a.z + (b.z - a.z) * t,
  };
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

  const float pitch_limit = 1.55334f;  // about 89 degrees
  pitch = std::clamp(pitch, -pitch_limit, pitch_limit);

  const float cos_pitch = std::cos(pitch);
  Vector3 new_offset{
      radius * cos_pitch * std::cos(yaw),
      radius * std::sin(pitch),
      radius * cos_pitch * std::sin(yaw),
  };

  camera->position = Vector3Add(camera->target, new_offset);
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

void UpdateCameraForViewport(Camera3D* camera, ViewerUiState& ui_state) {
  ImGuiIO& io = ImGui::GetIO();
  const bool ui_captures_mouse = io.WantCaptureMouse;
  const bool ui_captures_keyboard = io.WantCaptureKeyboard;
  const Vector2 mouse_delta = GetMouseDelta();
  const bool shift = IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT);
  const bool ctrl = IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL);

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

void PushLog(ViewerUiState& ui_state, const std::string& line) {
  ui_state.logs.push_back(line);
  if (ui_state.logs.size() > 12) {
    ui_state.logs.erase(ui_state.logs.begin());
  }
}

std::string DirtyBitsToText(wire::core::DirtyBits bits) {
  std::string text;
  if (wire::core::any(bits, wire::core::DirtyBits::kTopology)) text += "Topology|";
  if (wire::core::any(bits, wire::core::DirtyBits::kGeometry)) text += "Geometry|";
  if (wire::core::any(bits, wire::core::DirtyBits::kBounds)) text += "Bounds|";
  if (wire::core::any(bits, wire::core::DirtyBits::kRender)) text += "Render|";
  if (wire::core::any(bits, wire::core::DirtyBits::kRaycast)) text += "Raycast|";
  if (text.empty()) {
    return "None";
  }
  text.pop_back();
  return text;
}

Color DirtyColorForSpan(const wire::core::SpanRuntimeState* runtime_state) {
  if (runtime_state == nullptr) {
    return GRAY;
  }
  if (wire::core::any(runtime_state->dirty_bits, wire::core::DirtyBits::kTopology)) return PURPLE;
  if (wire::core::any(runtime_state->dirty_bits, wire::core::DirtyBits::kGeometry)) return YELLOW;
  if (wire::core::any(runtime_state->dirty_bits, wire::core::DirtyBits::kBounds)) return BLUE;
  if (wire::core::any(runtime_state->dirty_bits, wire::core::DirtyBits::kRender)) return RED;
  if (wire::core::any(runtime_state->dirty_bits, wire::core::DirtyBits::kRaycast)) return GREEN;
  return SKYBLUE;
}

void DrawAxes() {
  DrawLine3D(ToRaylib({0.0, 0.0, 0.0}), ToRaylib({kAxisLength, 0.0, 0.0}), RED);
  DrawLine3D(ToRaylib({0.0, 0.0, 0.0}), ToRaylib({0.0, kAxisLength, 0.0}), GREEN);
  DrawLine3D(ToRaylib({0.0, 0.0, 0.0}), ToRaylib({0.0, 0.0, kAxisLength}), BLUE);
}

void DrawCore(const CoreState& state, const ViewerUiState& ui_state) {
  const auto& edit = state.edit_state();

  for (const wire::core::Pole& pole : edit.poles.items()) {
    const wire::core::Vec3d pole_center_ue{
        pole.world_transform.position.x,
        pole.world_transform.position.y,
        pole.world_transform.position.z + (pole.height_m * 0.5),
    };

    Color color = DARKGRAY;
    if (ui_state.selected_type == SelectedType::kPole && ui_state.selected_id == pole.id) {
      color = GOLD;
    }
    DrawCylinderWires(ToRaylib(pole_center_ue), 0.12f, 0.12f, static_cast<float>(pole.height_m), 10, color);
  }

  for (const wire::core::Port& port : edit.ports.items()) {
    Color color = ORANGE;
    if (ui_state.selected_type == SelectedType::kPort && ui_state.selected_id == port.id) {
      color = GOLD;
    }
    DrawSphere(ToRaylib(port.world_position), 0.09f, color);
  }

  for (const wire::core::Anchor& anchor : edit.anchors.items()) {
    Color color = PURPLE;
    if (ui_state.selected_type == SelectedType::kAnchor && ui_state.selected_id == anchor.id) {
      color = GOLD;
    }
    DrawSphere(ToRaylib(anchor.world_position), 0.08f, color);
  }

  for (const wire::core::Span& span : edit.spans.items()) {
    const wire::core::Port* start_port = edit.ports.find(span.port_a_id);
    const wire::core::Port* end_port = edit.ports.find(span.port_b_id);
    if (start_port == nullptr || end_port == nullptr) {
      continue;
    }
    const wire::core::SpanRuntimeState* runtime_state = state.find_span_runtime_state(span.id);
    Color color = DirtyColorForSpan(runtime_state);
    if (ui_state.selected_type == SelectedType::kSpan && ui_state.selected_id == span.id) {
      color = GOLD;
    }

    const wire::core::CurveCacheEntry* curve = state.find_curve_cache(span.id);
    if (curve != nullptr && curve->points.size() >= 2) {
      for (std::size_t i = 0; i + 1 < curve->points.size(); ++i) {
        DrawLine3D(ToRaylib(curve->points[i]), ToRaylib(curve->points[i + 1]), color);
      }
    } else {
      DrawLine3D(ToRaylib(start_port->world_position), ToRaylib(end_port->world_position), color);
    }

    const wire::core::BoundsCacheEntry* bounds = state.find_bounds_cache(span.id);
    if (bounds != nullptr) {
      if (ui_state.show_whole_aabb) {
        DrawBoundingBox(ToRaylibBounds(bounds->whole), DARKGREEN);
      }
      if (ui_state.show_segment_aabb) {
        for (const wire::core::AABBd& segment : bounds->segments) {
          DrawBoundingBox(ToRaylibBounds(segment), Color{70, 130, 180, 180});
        }
      }
    }
  }

  for (const wire::core::Attachment& attachment : edit.attachments.items()) {
    const wire::core::Span* span = edit.spans.find(attachment.span_id);
    if (span == nullptr) {
      continue;
    }
    const wire::core::Port* port_a = edit.ports.find(span->port_a_id);
    const wire::core::Port* port_b = edit.ports.find(span->port_b_id);
    if (port_a == nullptr || port_b == nullptr) {
      continue;
    }
    wire::core::Vec3d pos = Lerp(port_a->world_position, port_b->world_position, attachment.t);
    pos.z += attachment.offset_m;

    Color color = MAGENTA;
    if (ui_state.selected_type == SelectedType::kAttachment && ui_state.selected_id == attachment.id) {
      color = GOLD;
    }
    DrawCubeV(ToRaylib(pos), Vector3{0.14f, 0.14f, 0.14f}, color);
  }
}

void DrawObjectList(
    ViewerUiState& ui_state,
    const char* header,
    SelectedType type,
    const std::vector<ObjectId>& ids,
    std::function<std::string(ObjectId)> make_label) {
  if (!ImGui::CollapsingHeader(header, ImGuiTreeNodeFlags_DefaultOpen)) {
    return;
  }

  for (ObjectId id : ids) {
    const std::string label = make_label(id);
    const bool is_selected = (ui_state.selected_type == type && ui_state.selected_id == id);
    if (ImGui::Selectable(label.c_str(), is_selected)) {
      ui_state.selected_type = type;
      ui_state.selected_id = id;
    }
  }
}

void DrawSelectedInfo(const CoreState& state, const ViewerUiState& ui_state) {
  ImGui::TextUnformatted("Selected");
  ImGui::Separator();
  if (ui_state.selected_type == SelectedType::kNone || ui_state.selected_id == wire::core::kInvalidObjectId) {
    ImGui::TextUnformatted("None");
    return;
  }

  const auto& edit = state.edit_state();
  switch (ui_state.selected_type) {
    case SelectedType::kPole: {
      const auto* pole = edit.poles.find(ui_state.selected_id);
      if (pole == nullptr) {
        ImGui::TextUnformatted("Selected pole is missing");
        return;
      }
      ImGui::Text("Type: Pole");
      ImGui::Text("ID: %s", pole->display_id.c_str());
      ImGui::Text("Name: %s", pole->name.c_str());
      ImGui::Text("PoleTypeId: %u", static_cast<unsigned int>(pole->pole_type_id));
      ImGui::Text("Generated: %s", pole->generation.generated ? "true" : "false");
      ImGui::Text("Gen Source: %d", static_cast<int>(pole->generation.source));
      ImGui::Text("Gen Session: %llu", static_cast<unsigned long long>(pole->generation.generation_session_id));
      ImGui::Text("Pos: %.2f %.2f %.2f", pole->world_transform.position.x, pole->world_transform.position.y, pole->world_transform.position.z);
      ImGui::Text("Height: %.2f", pole->height_m);
      ImGui::Text("PoleContext: %s", PoleContextLabel(pole->context.kind));
      ImGui::Text("cornerAngle: %.2f", pole->context.corner_angle_deg);
      ImGui::Text("cornerTurnSign: %.0f", pole->context.corner_turn_sign);
      ImGui::Text("sideScale: %.3f", pole->context.side_scale);
      ImGui::Text("angleCorrectionApplied: %s", pole->context.angle_correction_applied ? "true" : "false");
      ImGui::Text("flip180: %s", pole->orientation_control.flip_180 ? "true" : "false");
      ImGui::Text("manualYawOverride: %s", pole->orientation_control.manual_yaw_override ? "true" : "false");
      ImGui::Text("placementOverride: %s", pole->placement_override_flag ? "true" : "false");
      ImGui::Text("orientationOverride: %s", pole->orientation_override_flag ? "true" : "false");
      return;
    }
    case SelectedType::kPort: {
      const auto* port = edit.ports.find(ui_state.selected_id);
      if (port == nullptr) {
        ImGui::TextUnformatted("Selected port is missing");
        return;
      }
      ImGui::Text("Type: Port");
      ImGui::Text("ID: %s", port->display_id.c_str());
      ImGui::Text("Owner Pole: %llu", static_cast<unsigned long long>(port->owner_pole_id));
      ImGui::Text("Pos: %.2f %.2f %.2f", port->world_position.x, port->world_position.y, port->world_position.z);
      ImGui::Text("Category: %s", CategoryLabel(port->category));
      ImGui::Text("SlotId: %d", port->source_slot_id);
      ImGui::Text("Layer: %d Side: %s Role: %s", port->template_layer, SlotSideLabel(port->template_side), SlotRoleLabel(port->template_role));
      ImGui::Text("GeneratedByRule: %s", port->generated_by_rule ? "true" : "false");
      ImGui::Text("PlacementContext: %s", ContextLabel(port->placement_context));
      ImGui::Text("AngleCorrected: %s sideScale=%.3f", port->angle_correction_applied ? "true" : "false", port->side_scale_applied);
      ImGui::Text("placementOverride: %s", port->placement_override_flag ? "true" : "false");
      ImGui::Text("orientationOverride: %s", port->orientation_override_flag ? "true" : "false");
      return;
    }
    case SelectedType::kSpan: {
      const auto* span = edit.spans.find(ui_state.selected_id);
      if (span == nullptr) {
        ImGui::TextUnformatted("Selected span is missing");
        return;
      }
      ImGui::Text("Type: Span");
      ImGui::Text("ID: %s", span->display_id.c_str());
      ImGui::Text("portA: %llu", static_cast<unsigned long long>(span->port_a_id));
      ImGui::Text("portB: %llu", static_cast<unsigned long long>(span->port_b_id));
      ImGui::Text("bundle: %llu", static_cast<unsigned long long>(span->bundle_id));
      ImGui::Text("Generated: %s", span->generation.generated ? "true" : "false");
      ImGui::Text("Gen Session: %llu", static_cast<unsigned long long>(span->generation.generation_session_id));
      ImGui::Text("GeneratedByRule: %s", span->generated_by_rule ? "true" : "false");
      ImGui::Text("PlacementContext: %s", ContextLabel(span->placement_context));
      ImGui::Text("placementOverride: %s", span->placement_override_flag ? "true" : "false");
      ImGui::Text("orientationOverride: %s", span->orientation_override_flag ? "true" : "false");
      const auto* curve = state.find_curve_cache(span->id);
      if (curve != nullptr) {
        ImGui::Text("curveSamples: %d", static_cast<int>(curve->points.size()));
        ImGui::Text("curveLength: %.2f", PolylineLength(curve->points));
      } else {
        ImGui::TextUnformatted("curveSamples: (none)");
      }
      const auto* bounds = state.find_bounds_cache(span->id);
      if (bounds != nullptr) {
        const double sx = bounds->whole.max.x - bounds->whole.min.x;
        const double sy = bounds->whole.max.y - bounds->whole.min.y;
        const double sz = bounds->whole.max.z - bounds->whole.min.z;
        ImGui::Text("AABB size: %.2f %.2f %.2f", sx, sy, sz);
        ImGui::Text("segmentAABBs: %d", static_cast<int>(bounds->segments.size()));
      }
      const auto* runtime_state = state.find_span_runtime_state(span->id);
      if (runtime_state != nullptr) {
        ImGui::Separator();
        ImGui::Text("dataVersion: %llu", static_cast<unsigned long long>(runtime_state->data_version));
        ImGui::Text("geometryVersion: %llu", static_cast<unsigned long long>(runtime_state->geometry_version));
        ImGui::Text("boundsVersion: %llu", static_cast<unsigned long long>(runtime_state->bounds_version));
        ImGui::Text("renderVersion: %llu", static_cast<unsigned long long>(runtime_state->render_version));
        ImGui::Text("dirtyBits: %s", DirtyBitsToText(runtime_state->dirty_bits).c_str());
      }
      return;
    }
    case SelectedType::kAnchor: {
      const auto* anchor = edit.anchors.find(ui_state.selected_id);
      if (anchor == nullptr) {
        ImGui::TextUnformatted("Selected anchor is missing");
        return;
      }
      ImGui::Text("Type: Anchor");
      ImGui::Text("ID: %s", anchor->display_id.c_str());
      ImGui::Text("Owner Pole: %llu", static_cast<unsigned long long>(anchor->owner_pole_id));
      ImGui::Text("Pos: %.2f %.2f %.2f", anchor->world_position.x, anchor->world_position.y, anchor->world_position.z);
      return;
    }
    case SelectedType::kBundle: {
      const auto* bundle = edit.bundles.find(ui_state.selected_id);
      if (bundle == nullptr) {
        ImGui::TextUnformatted("Selected bundle is missing");
        return;
      }
      ImGui::Text("Type: Bundle");
      ImGui::Text("ID: %s", bundle->display_id.c_str());
      ImGui::Text("Conductor count: %d", bundle->conductor_count);
      ImGui::Text("Spacing: %.2f", bundle->phase_spacing_m);
      return;
    }
    case SelectedType::kAttachment: {
      const auto* attachment = edit.attachments.find(ui_state.selected_id);
      if (attachment == nullptr) {
        ImGui::TextUnformatted("Selected attachment is missing");
        return;
      }
      ImGui::Text("Type: Attachment");
      ImGui::Text("ID: %s", attachment->display_id.c_str());
      ImGui::Text("Span: %llu", static_cast<unsigned long long>(attachment->span_id));
      ImGui::Text("t: %.3f", attachment->t);
      return;
    }
    default:
      break;
  }
}

void HandleResultError(ViewerUiState& ui_state, const std::string& error, const std::string& fallback_log) {
  if (!error.empty()) {
    ui_state.last_error = error;
  } else {
    ui_state.last_error = fallback_log;
  }
  PushLog(ui_state, fallback_log);
}

void DrawEditSelectedPanel(CoreState& state, ViewerUiState& ui_state) {
  ImGui::Separator();
  ImGui::TextUnformatted("Edit Selected");
  ImGui::InputDouble("Edit X", &ui_state.edit_x);
  ImGui::InputDouble("Edit Y", &ui_state.edit_y);
  ImGui::InputDouble("Edit Z", &ui_state.edit_z);

  if (ui_state.selected_type == SelectedType::kPole) {
    if (ImGui::Button("Load Pole Pos")) {
      const auto* pole = state.edit_state().poles.find(ui_state.selected_id);
      if (pole != nullptr) {
        ui_state.edit_x = pole->world_transform.position.x;
        ui_state.edit_y = pole->world_transform.position.y;
        ui_state.edit_z = pole->world_transform.position.z;
      }
    }
    ImGui::SameLine();
    if (ImGui::Button("Move Pole")) {
      const auto* pole = state.edit_state().poles.find(ui_state.selected_id);
      if (pole != nullptr) {
        wire::core::Transformd moved = pole->world_transform;
        moved.position = {ui_state.edit_x, ui_state.edit_y, ui_state.edit_z};
        const auto result = state.MovePole(ui_state.selected_id, moved);
        if (!result.ok) {
          HandleResultError(ui_state, result.error, "Move Pole failed");
        } else {
          ui_state.last_error.clear();
          PushLog(ui_state, "Moved Pole id=" + std::to_string(ui_state.selected_id));
        }
      }
    }
  } else if (ui_state.selected_type == SelectedType::kPort) {
    if (ImGui::Button("Load Port Pos")) {
      const auto* port = state.edit_state().ports.find(ui_state.selected_id);
      if (port != nullptr) {
        ui_state.edit_x = port->world_position.x;
        ui_state.edit_y = port->world_position.y;
        ui_state.edit_z = port->world_position.z;
      }
    }
    ImGui::SameLine();
    if (ImGui::Button("Move Port")) {
      const auto result = state.MovePort(ui_state.selected_id, {ui_state.edit_x, ui_state.edit_y, ui_state.edit_z});
      if (!result.ok) {
        HandleResultError(ui_state, result.error, "Move Port failed");
      } else {
        ui_state.last_error.clear();
        PushLog(ui_state, "Moved Port id=" + std::to_string(ui_state.selected_id));
      }
    }
  } else if (ui_state.selected_type == SelectedType::kAnchor) {
    if (ImGui::Button("Load Anchor Pos")) {
      const auto* anchor = state.edit_state().anchors.find(ui_state.selected_id);
      if (anchor != nullptr) {
        ui_state.edit_x = anchor->world_position.x;
        ui_state.edit_y = anchor->world_position.y;
        ui_state.edit_z = anchor->world_position.z;
      }
    }
    ImGui::SameLine();
    if (ImGui::Button("Move Anchor")) {
      const auto result = state.MoveAnchor(ui_state.selected_id, {ui_state.edit_x, ui_state.edit_y, ui_state.edit_z});
      if (!result.ok) {
        HandleResultError(ui_state, result.error, "Move Anchor failed");
      } else {
        ui_state.last_error.clear();
        PushLog(ui_state, "Moved Anchor id=" + std::to_string(ui_state.selected_id));
      }
    }
  } else if (ui_state.selected_type == SelectedType::kSpan) {
    ImGui::InputDouble("Split t", &ui_state.split_t);
    if (ImGui::Button("Split Span")) {
      const auto result = state.SplitSpan(ui_state.selected_id, ui_state.split_t);
      if (!result.ok) {
        HandleResultError(ui_state, result.error, "Split Span failed");
      } else {
        ui_state.last_error.clear();
        PushLog(ui_state, "Split Span id=" + std::to_string(ui_state.selected_id));
        ui_state.selected_type = SelectedType::kPort;
        ui_state.selected_id = result.value.new_port_id;
      }
    }
    ImGui::SameLine();
    if (ImGui::Button("Delete Span")) {
      const auto result = state.DeleteSpan(ui_state.selected_id);
      if (!result.ok) {
        HandleResultError(ui_state, result.error, "Delete Span failed");
      } else {
        ui_state.last_error.clear();
        PushLog(ui_state, "Deleted Span id=" + std::to_string(ui_state.selected_id));
        ui_state.selected_type = SelectedType::kNone;
        ui_state.selected_id = wire::core::kInvalidObjectId;
      }
    }
  } else {
    ImGui::TextUnformatted("Select Pole/Port/Anchor/Span to edit");
  }
}

void DrawPlacementModePanel(CoreState& state, ViewerUiState& ui_state) {
  ImGui::TextUnformatted("Placement");
  const auto type_ids = SortedPoleTypeIds(state);
  if (type_ids.empty()) {
    ImGui::TextUnformatted("No PoleType available");
    return;
  }
  const std::size_t type_index = ClampedTypeIndex(ui_state.placement_pole_type_index, type_ids.size());
  ui_state.placement_pole_type_index = static_cast<int>(type_index);
  const wire::core::PoleTypeId selected_type_id = type_ids[type_index];
  const auto type_it = state.pole_types().find(selected_type_id);
  const std::string selected_type_name = (type_it != state.pole_types().end()) ? type_it->second.name : std::to_string(selected_type_id);

  if (ImGui::BeginCombo("PoleType", selected_type_name.c_str())) {
    for (std::size_t i = 0; i < type_ids.size(); ++i) {
      const auto it = state.pole_types().find(type_ids[i]);
      const std::string label = (it != state.pole_types().end()) ? it->second.name : std::to_string(type_ids[i]);
      const bool selected = (i == type_index);
      if (ImGui::Selectable(label.c_str(), selected)) {
        ui_state.placement_pole_type_index = static_cast<int>(i);
      }
      if (selected) {
        ImGui::SetItemDefaultFocus();
      }
    }
    ImGui::EndCombo();
  }

  ImGui::InputDouble("Pole X", &ui_state.pole_x);
  ImGui::InputDouble("Pole Y", &ui_state.pole_y);
  ImGui::InputDouble("Pole Z", &ui_state.pole_z);
  ImGui::InputDouble("Pole Height", &ui_state.pole_height);
  if (ImGui::Button("Place Pole")) {
    wire::core::Transformd tf{};
    tf.position = {ui_state.pole_x, ui_state.pole_y, ui_state.pole_z};
    const auto add_pole_result = state.AddPole(tf, ui_state.pole_height, "Pole", static_cast<wire::core::PoleKind>(ui_state.pole_kind));
    if (!add_pole_result.ok) {
      ui_state.last_error = add_pole_result.error;
      PushLog(ui_state, "Place Pole failed");
      return;
    }
    const auto apply_result = state.ApplyPoleType(add_pole_result.value, type_ids[ui_state.placement_pole_type_index]);
    if (!apply_result.ok) {
      ui_state.last_error = apply_result.error;
      PushLog(ui_state, "ApplyPoleType failed");
      return;
    }
    ui_state.last_error.clear();
    ui_state.selected_type = SelectedType::kPole;
    ui_state.selected_id = add_pole_result.value;
    PushLog(ui_state, "Placed Pole id=" + std::to_string(add_pole_result.value));
  }

  ImGui::Separator();
  ImGui::TextUnformatted("Road Auto Generate");
  ImGui::InputScalar("Road Id", ImGuiDataType_U64, &ui_state.road_id);
  ImGui::InputDouble("Road Start X", &ui_state.road_start_x);
  ImGui::InputDouble("Road Start Y", &ui_state.road_start_y);
  ImGui::InputDouble("Road Start Z", &ui_state.road_start_z);
  ImGui::Checkbox("Use Mid Point", &ui_state.road_use_mid);
  if (ui_state.road_use_mid) {
    ImGui::InputDouble("Road Mid X", &ui_state.road_mid_x);
    ImGui::InputDouble("Road Mid Y", &ui_state.road_mid_y);
    ImGui::InputDouble("Road Mid Z", &ui_state.road_mid_z);
  }
  ImGui::InputDouble("Road End X", &ui_state.road_end_x);
  ImGui::InputDouble("Road End Y", &ui_state.road_end_y);
  ImGui::InputDouble("Road End Z", &ui_state.road_end_z);
  ImGui::InputDouble("Pole Interval", &ui_state.road_interval);

  const std::size_t road_type_index = ClampedTypeIndex(ui_state.road_pole_type_index, type_ids.size());
  ui_state.road_pole_type_index = static_cast<int>(road_type_index);
  const wire::core::PoleTypeId road_type_id = type_ids[road_type_index];
  const auto road_type_it = state.pole_types().find(road_type_id);
  const std::string road_type_name = (road_type_it != state.pole_types().end()) ? road_type_it->second.name : std::to_string(road_type_id);
  if (ImGui::BeginCombo("Road PoleType", road_type_name.c_str())) {
    for (std::size_t i = 0; i < type_ids.size(); ++i) {
      const auto it = state.pole_types().find(type_ids[i]);
      const std::string label = (it != state.pole_types().end()) ? it->second.name : std::to_string(type_ids[i]);
      const bool selected = (i == road_type_index);
      if (ImGui::Selectable(label.c_str(), selected)) {
        ui_state.road_pole_type_index = static_cast<int>(i);
      }
      if (selected) {
        ImGui::SetItemDefaultFocus();
      }
    }
    ImGui::EndCombo();
  }

  const int road_category_index = std::clamp(ui_state.road_category_index, 0, static_cast<int>(kAllCategories.size() - 1));
  ui_state.road_category_index = road_category_index;
  if (ImGui::BeginCombo("Road Category", CategoryLabel(kAllCategories[road_category_index]))) {
    for (int i = 0; i < static_cast<int>(kAllCategories.size()); ++i) {
      const bool selected = (i == road_category_index);
      if (ImGui::Selectable(CategoryLabel(kAllCategories[i]), selected)) {
        ui_state.road_category_index = i;
      }
      if (selected) {
        ImGui::SetItemDefaultFocus();
      }
    }
    ImGui::EndCombo();
  }

  if (ImGui::Button("Generate Road Line")) {
    wire::core::RoadSegment road{};
    road.id = ui_state.road_id;
    road.polyline.push_back({ui_state.road_start_x, ui_state.road_start_y, ui_state.road_start_z});
    if (ui_state.road_use_mid) {
      road.polyline.push_back({ui_state.road_mid_x, ui_state.road_mid_y, ui_state.road_mid_z});
    }
    road.polyline.push_back({ui_state.road_end_x, ui_state.road_end_y, ui_state.road_end_z});

    const auto result = state.GenerateSimpleLine(
        road,
        ui_state.road_interval,
        type_ids[ui_state.road_pole_type_index],
        kAllCategories[ui_state.road_category_index]);
    if (!result.ok) {
      ui_state.last_error = result.error;
      PushLog(ui_state, "GenerateSimpleLine failed");
    } else {
      ui_state.last_error.clear();
      ui_state.last_generated_poles = static_cast<int>(result.value.pole_ids.size());
      ui_state.last_generated_spans = static_cast<int>(result.value.span_ids.size());
      ui_state.last_generation_session = result.value.generation_session_id;
      if (!result.value.pole_ids.empty()) {
        ui_state.selected_type = SelectedType::kPole;
        ui_state.selected_id = result.value.pole_ids.back();
      }
      PushLog(
          ui_state,
          "Generated road poles=" + std::to_string(ui_state.last_generated_poles) +
              " spans=" + std::to_string(ui_state.last_generated_spans));
    }
  }
  ImGui::Text("Last generated poles: %d", ui_state.last_generated_poles);
  ImGui::Text("Last generated spans: %d", ui_state.last_generated_spans);
  ImGui::Text("Last generation session: %llu", static_cast<unsigned long long>(ui_state.last_generation_session));
}

void DrawConnectionModePanel(CoreState& state, ViewerUiState& ui_state) {
  ImGui::TextUnformatted("Connection (Pole -> Pole)");
  ImGui::InputScalar("Source PoleId", ImGuiDataType_U64, &ui_state.connect_pole_a_id);
  ImGui::InputScalar("Target PoleId", ImGuiDataType_U64, &ui_state.connect_pole_b_id);

  const int category_index = std::clamp(ui_state.connect_category, 0, static_cast<int>(kAllCategories.size() - 1));
  ui_state.connect_category = category_index;
  const int context_index = std::clamp(ui_state.connect_context, 0, static_cast<int>(kAllConnectionContexts.size() - 1));
  ui_state.connect_context = context_index;
  if (ImGui::BeginCombo("Category", CategoryLabel(kAllCategories[category_index]))) {
    for (int i = 0; i < static_cast<int>(kAllCategories.size()); ++i) {
      const bool selected = (i == category_index);
      if (ImGui::Selectable(CategoryLabel(kAllCategories[i]), selected)) {
        ui_state.connect_category = i;
      }
      if (selected) {
        ImGui::SetItemDefaultFocus();
      }
    }
    ImGui::EndCombo();
  }
  if (ImGui::BeginCombo("Context", ContextLabel(kAllConnectionContexts[context_index]))) {
    for (int i = 0; i < static_cast<int>(kAllConnectionContexts.size()); ++i) {
      const bool selected = (i == context_index);
      if (ImGui::Selectable(ContextLabel(kAllConnectionContexts[i]), selected)) {
        ui_state.connect_context = i;
      }
      if (selected) {
        ImGui::SetItemDefaultFocus();
      }
    }
    ImGui::EndCombo();
  }

  if (ImGui::Button("Connect Pole -> Pole")) {
    const int selected_context_index =
        std::clamp(ui_state.connect_context, 0, static_cast<int>(kAllConnectionContexts.size() - 1));
    wire::core::CoreState::AddConnectionByPoleOptions options{};
    options.connection_context = kAllConnectionContexts[selected_context_index];
    const auto result = state.AddConnectionByPole(
        ui_state.connect_pole_a_id,
        ui_state.connect_pole_b_id,
        kAllCategories[ui_state.connect_category],
        options);
    if (!result.ok) {
      ui_state.last_error = result.error;
      PushLog(ui_state, "Connect Pole->Pole failed");
    } else {
      ui_state.last_error.clear();
      ui_state.selected_type = SelectedType::kSpan;
      ui_state.selected_id = result.value.span_id;
      PushLog(
          ui_state,
          "Connected span=" + std::to_string(result.value.span_id) +
              " portA=" + std::to_string(result.value.port_a_id) +
              " slotA=" + std::to_string(result.value.slot_a_id) +
              " portB=" + std::to_string(result.value.port_b_id) +
              " slotB=" + std::to_string(result.value.slot_b_id));
    }
  }
}

void DrawPathModePanel(CoreState& state, ViewerUiState& ui_state) {
  ImGui::TextUnformatted("Draw Path");
  ImGui::TextUnformatted("LMB: add point on draw plane");
  ImGui::TextUnformatted("RMB/Backspace: undo last");
  ImGui::TextUnformatted("Esc: clear path, Enter: generate");
  ImGui::InputDouble("Draw Plane Z", &ui_state.draw_plane_z, 0.1, 1.0, "%.2f");
  ImGui::Checkbox("Show Preview", &ui_state.draw_show_preview);
  ImGui::Checkbox("Keep Path After Generate", &ui_state.draw_keep_path_after_generate);
  ImGui::InputInt("Parallel Spans (0=auto by PoleType slots)", &ui_state.draw_parallel_spans);
  ui_state.draw_parallel_spans = std::clamp(ui_state.draw_parallel_spans, 0, 8);
  ImGui::TextUnformatted("Pole placement: one pole per clicked path point");
  ImGui::Text("Path points: %d", static_cast<int>(ui_state.draw_path_points.size()));
  if (ui_state.draw_hover_valid) {
    ImGui::Text(
        "Hover: %.2f %.2f %.2f",
        ui_state.draw_hover_point.x,
        ui_state.draw_hover_point.y,
        ui_state.draw_hover_point.z);
  } else {
    ImGui::TextUnformatted("Hover: (no ground hit)");
  }

  const auto type_ids = SortedPoleTypeIds(state);
  if (!type_ids.empty()) {
    const std::size_t road_type_index = ClampedTypeIndex(ui_state.road_pole_type_index, type_ids.size());
    ui_state.road_pole_type_index = static_cast<int>(road_type_index);
    const wire::core::PoleTypeId road_type_id = type_ids[road_type_index];
    const auto road_type_it = state.pole_types().find(road_type_id);
    const std::string road_type_name =
        (road_type_it != state.pole_types().end()) ? road_type_it->second.name : std::to_string(road_type_id);
    if (ImGui::BeginCombo("Path PoleType", road_type_name.c_str())) {
      for (std::size_t i = 0; i < type_ids.size(); ++i) {
        const auto it = state.pole_types().find(type_ids[i]);
        const std::string label = (it != state.pole_types().end()) ? it->second.name : std::to_string(type_ids[i]);
        const bool selected = (i == road_type_index);
        if (ImGui::Selectable(label.c_str(), selected)) {
          ui_state.road_pole_type_index = static_cast<int>(i);
        }
        if (selected) {
          ImGui::SetItemDefaultFocus();
        }
      }
      ImGui::EndCombo();
    }
  }

  const int category_index = std::clamp(ui_state.road_category_index, 0, static_cast<int>(kAllCategories.size() - 1));
  ui_state.road_category_index = category_index;
  if (ImGui::BeginCombo("Path Category", CategoryLabel(kAllCategories[category_index]))) {
    for (int i = 0; i < static_cast<int>(kAllCategories.size()); ++i) {
      const bool selected = (i == category_index);
      if (ImGui::Selectable(CategoryLabel(kAllCategories[i]), selected)) {
        ui_state.road_category_index = i;
      }
      if (selected) {
        ImGui::SetItemDefaultFocus();
      }
    }
    ImGui::EndCombo();
  }
  if (ImGui::BeginCombo(
          "Direction Mode",
          PathDirectionModeLabel(static_cast<wire::core::PathDirectionMode>(ui_state.draw_direction_mode)))) {
    for (int i = 0; i < 3; ++i) {
      const bool selected = (i == ui_state.draw_direction_mode);
      const auto mode = static_cast<wire::core::PathDirectionMode>(i);
      if (ImGui::Selectable(PathDirectionModeLabel(mode), selected)) {
        ui_state.draw_direction_mode = i;
      }
      if (selected) {
        ImGui::SetItemDefaultFocus();
      }
    }
    ImGui::EndCombo();
  }
  const wire::core::PoleTypeId resolved_type_id =
      type_ids.empty()
          ? wire::core::kInvalidPoleTypeId
          : type_ids[ClampedTypeIndex(ui_state.road_pole_type_index, type_ids.size())];
  const int auto_parallel =
      (resolved_type_id == wire::core::kInvalidPoleTypeId)
          ? FallbackParallelSpanCount(kAllCategories[category_index])
          : AutoParallelSpanCountFromPoleType(state, resolved_type_id, kAllCategories[category_index]);
  const int resolved_parallel =
      (ui_state.draw_parallel_spans > 0) ? ui_state.draw_parallel_spans : auto_parallel;
  ImGui::Text("Resolved Parallel Spans: %d", resolved_parallel);
  if (ImGui::Button("Flip Direction (Manual)")) {
    if (ui_state.draw_direction_mode == static_cast<int>(wire::core::PathDirectionMode::kReverse)) {
      ui_state.draw_direction_mode = static_cast<int>(wire::core::PathDirectionMode::kForward);
    } else {
      ui_state.draw_direction_mode = static_cast<int>(wire::core::PathDirectionMode::kReverse);
    }
  }
  const auto& dir_debug = state.last_path_direction_debug();
  ImGui::Text(
      "Direction chosen: %s (mode=%s)",
      PathDirectionChosenLabel(dir_debug.chosen),
      PathDirectionModeLabel(dir_debug.requested_mode));
  ImGui::Text(
      "Cost F/R: %d / %d",
      dir_debug.forward_cost.total,
      dir_debug.reverse_cost.total);
  ImGui::Text(
      "Cost detail cross=%d side=%d layer=%d corner=%d branch=%d",
      dir_debug.forward_cost.estimated_cross_penalty,
      dir_debug.forward_cost.side_flip_penalty,
      dir_debug.forward_cost.layer_jump_penalty,
      dir_debug.forward_cost.corner_compression_penalty,
      dir_debug.forward_cost.branch_conflict_penalty);

  if (ImGui::Button("Generate From Path")) {
    ExecuteGenerateFromDrawPath(state, ui_state, false);
  }
  ImGui::SameLine();
  if (ImGui::Button("Undo Last Point")) {
    if (!ui_state.draw_path_points.empty()) {
      ui_state.draw_path_points.pop_back();
    }
  }
  ImGui::SameLine();
  if (ImGui::Button("Clear Path")) {
    ui_state.draw_path_points.clear();
  }

  const auto& lane_assignments = state.last_lane_assignments();
  ImGui::Separator();
  ImGui::Text("Lane assignments: %d", static_cast<int>(lane_assignments.size()));
  for (const auto& a : lane_assignments) {
    ImGui::Text(
        "seg=%d A=%llu B=%llu lanes=%d mirrored=%s",
        static_cast<int>(a.segment_index),
        static_cast<unsigned long long>(a.pole_a_id),
        static_cast<unsigned long long>(a.pole_b_id),
        static_cast<int>(a.port_ids_a.size()),
        a.mirrored ? "true" : "false");
  }
}

void DrawBranchModePanel(CoreState& state, ViewerUiState& ui_state) {
  ImGui::TextUnformatted("Branch / Drop");
  ImGui::InputScalar("Branch Source SpanId", ImGuiDataType_U64, &ui_state.branch_source_span_id);
  ImGui::InputDouble("Branch t", &ui_state.branch_t);
  ImGui::InputDouble("Branch Target X", &ui_state.branch_target_x);
  ImGui::InputDouble("Branch Target Y", &ui_state.branch_target_y);
  ImGui::InputDouble("Branch Target Z", &ui_state.branch_target_z);
  if (ImGui::Button("Add Drop From Span")) {
    const auto result = state.AddDropFromSpan(
        ui_state.branch_source_span_id,
        ui_state.branch_t,
        {ui_state.branch_target_x, ui_state.branch_target_y, ui_state.branch_target_z},
        wire::core::ConnectionCategory::kDrop);
    if (!result.ok) {
      ui_state.last_error = result.error;
      PushLog(ui_state, "AddDropFromSpan failed");
    } else {
      ui_state.last_error.clear();
      ui_state.selected_type = SelectedType::kSpan;
      ui_state.selected_id = result.value.span_id;
      PushLog(
          ui_state,
          "DropFromSpan span=" + std::to_string(result.value.span_id) +
              " splitPort=" + std::to_string(result.value.split_port_id));
    }
  }

  ImGui::Separator();
  ImGui::TextUnformatted("Drop From Pole");
  ImGui::InputScalar("Drop Source PoleId", ImGuiDataType_U64, &ui_state.drop_source_pole_id);
  ImGui::InputDouble("Drop Target X", &ui_state.drop_target_x);
  ImGui::InputDouble("Drop Target Y", &ui_state.drop_target_y);
  ImGui::InputDouble("Drop Target Z", &ui_state.drop_target_z);
  if (ImGui::Button("Add Drop From Pole")) {
    const auto result = state.AddDropFromPole(
        ui_state.drop_source_pole_id,
        {ui_state.drop_target_x, ui_state.drop_target_y, ui_state.drop_target_z},
        wire::core::ConnectionCategory::kDrop);
    if (!result.ok) {
      ui_state.last_error = result.error;
      PushLog(ui_state, "AddDropFromPole failed");
    } else {
      ui_state.last_error.clear();
      ui_state.selected_type = SelectedType::kSpan;
      ui_state.selected_id = result.value.span_id;
      PushLog(
          ui_state,
          "DropFromPole span=" + std::to_string(result.value.span_id) +
              " sourcePort=" + std::to_string(result.value.source_port_id));
    }
  }
}

void DrawDetailModePanel(CoreState& state, ViewerUiState& ui_state) {
  ImGui::TextUnformatted("Detail");
  ImGui::Checkbox("Debug Labels", &ui_state.show_debug_labels);

  if (ui_state.selected_type != SelectedType::kPole || ui_state.selected_id == wire::core::kInvalidObjectId) {
    ImGui::TextUnformatted("Select a Pole in list to edit details");
    return;
  }

  const auto detail = state.GetPoleDetail(ui_state.selected_id);
  if (detail.pole == nullptr) {
    ImGui::TextUnformatted("Selected pole is missing");
    return;
  }

  ImGui::Text("Pole: %s", detail.pole->display_id.c_str());
  ImGui::Text("PoleTypeId: %u", static_cast<unsigned int>(detail.pole->pole_type_id));
  bool flip_180 = detail.pole->orientation_control.flip_180;
  if (ImGui::Checkbox("Flip 180", &flip_180)) {
    const auto flip_result = state.SetPoleFlip180(ui_state.selected_id, flip_180);
    if (!flip_result.ok) {
      ui_state.last_error = flip_result.error;
      PushLog(ui_state, "SetPoleFlip180 failed");
    } else {
      ui_state.last_error.clear();
      PushLog(ui_state, "SetPoleFlip180 updated");
    }
  }

  const auto type_ids = SortedPoleTypeIds(state);
  if (!type_ids.empty()) {
    const std::size_t idx = ClampedTypeIndex(ui_state.detail_pole_type_index, type_ids.size());
    ui_state.detail_pole_type_index = static_cast<int>(idx);
    const auto type_it = state.pole_types().find(type_ids[idx]);
    const std::string selected_name = (type_it != state.pole_types().end()) ? type_it->second.name : std::to_string(type_ids[idx]);
    if (ImGui::BeginCombo("Set PoleType", selected_name.c_str())) {
      for (std::size_t i = 0; i < type_ids.size(); ++i) {
        const auto it = state.pole_types().find(type_ids[i]);
        const std::string label = (it != state.pole_types().end()) ? it->second.name : std::to_string(type_ids[i]);
        const bool selected = (i == idx);
        if (ImGui::Selectable(label.c_str(), selected)) {
          ui_state.detail_pole_type_index = static_cast<int>(i);
        }
      }
      ImGui::EndCombo();
    }

    if (ImGui::Button("Apply PoleType To Selected")) {
      const auto apply_result = state.ApplyPoleType(ui_state.selected_id, type_ids[ui_state.detail_pole_type_index]);
      if (!apply_result.ok) {
        ui_state.last_error = apply_result.error;
        PushLog(ui_state, "ApplyPoleType failed");
      } else {
        ui_state.last_error.clear();
        PushLog(ui_state, "Applied PoleType to " + std::to_string(ui_state.selected_id));
      }
    }
  }

  ImGui::Separator();
  ImGui::Text("Ports: %d", static_cast<int>(detail.owned_ports.size()));
  std::unordered_map<int, const wire::core::Port*> by_slot;
  for (const auto* port : detail.owned_ports) {
    by_slot[port->source_slot_id] = port;
  }

  if (detail.pole_type != nullptr) {
    ImGui::TextUnformatted("Slot usage");
    for (const auto& slot : detail.pole_type->port_slots) {
      const auto it = by_slot.find(slot.slot_id);
      if (it == by_slot.end()) {
        ImGui::Text(
            "slot=%d cat=%s layer=%d side=%s role=%s used=0 [empty]",
            slot.slot_id,
            CategoryLabel(slot.category),
            slot.layer,
            SlotSideLabel(slot.side),
            SlotRoleLabel(slot.role));
        continue;
      }
      const auto* port = it->second;
      const auto usage_it = state.connection_index().spans_by_port.find(port->id);
      const int usage = (usage_it == state.connection_index().spans_by_port.end()) ? 0 : static_cast<int>(usage_it->second.size());
      ImGui::Text(
          "slot=%d cat=%s layer=%d side=%s role=%s used=%d -> %s",
          slot.slot_id,
          CategoryLabel(slot.category),
          slot.layer,
          SlotSideLabel(slot.side),
          SlotRoleLabel(slot.role),
          usage,
          port->display_id.c_str());
    }
  } else {
    for (const auto* port : detail.owned_ports) {
      const auto it = state.connection_index().spans_by_port.find(port->id);
      const int usage = (it == state.connection_index().spans_by_port.end()) ? 0 : static_cast<int>(it->second.size());
      ImGui::Text(
          "%s cat=%s slot=%d used=%d",
          port->display_id.c_str(),
          CategoryLabel(port->category),
          port->source_slot_id,
          usage);
    }
  }

  ImGui::Separator();
  ImGui::Text("Anchors: %d", static_cast<int>(detail.owned_anchors.size()));
  for (const auto* anchor : detail.owned_anchors) {
    ImGui::Text("%s slot=%d", anchor->display_id.c_str(), anchor->source_slot_id);
  }
}

void DrawDebugDirectPanel(CoreState& state, ViewerUiState& ui_state) {
  if (!ImGui::CollapsingHeader("Debug: Direct Port/Span Editing")) {
    return;
  }

  ImGui::InputScalar("Port Owner PoleId", ImGuiDataType_U64, &ui_state.port_owner_pole_id);
  ImGui::InputDouble("Port X", &ui_state.port_x);
  ImGui::InputDouble("Port Y", &ui_state.port_y);
  ImGui::InputDouble("Port Z", &ui_state.port_z);
  if (ImGui::Button("Debug Add Port")) {
    const auto result = state.AddPort(
        ui_state.port_owner_pole_id,
        {ui_state.port_x, ui_state.port_y, ui_state.port_z},
        static_cast<wire::core::PortKind>(ui_state.port_kind),
        static_cast<wire::core::PortLayer>(ui_state.port_layer));
    if (!result.ok) {
      ui_state.last_error = result.error;
    } else {
      ui_state.last_error.clear();
      ui_state.selected_type = SelectedType::kPort;
      ui_state.selected_id = result.value;
      PushLog(ui_state, "Debug Add Port id=" + std::to_string(result.value));
    }
  }

  ImGui::InputScalar("Span PortAId", ImGuiDataType_U64, &ui_state.span_port_a_id);
  ImGui::InputScalar("Span PortBId", ImGuiDataType_U64, &ui_state.span_port_b_id);
  ImGui::InputScalar("Span BundleId (0=none)", ImGuiDataType_U64, &ui_state.span_bundle_id);
  if (ImGui::Button("Debug Add Span")) {
    const auto result = state.AddSpan(
        ui_state.span_port_a_id,
        ui_state.span_port_b_id,
        static_cast<wire::core::SpanKind>(ui_state.span_kind),
        static_cast<wire::core::SpanLayer>(ui_state.span_layer),
        ui_state.span_bundle_id);
    if (!result.ok) {
      ui_state.last_error = result.error;
      PushLog(ui_state, "Debug Add Span failed");
    } else {
      ui_state.last_error.clear();
      ui_state.selected_type = SelectedType::kSpan;
      ui_state.selected_id = result.value;
      PushLog(ui_state, "Debug Add Span id=" + std::to_string(result.value));
    }
  }
}

void DrawStatsPanel(CoreState& state, ViewerUiState& ui_state) {
  const float screen_width = static_cast<float>(GetScreenWidth());
  const float screen_height = static_cast<float>(GetScreenHeight());
  const float min_panel_width = 280.0f;
  const float max_panel_width = std::max(min_panel_width, screen_width - 220.0f);
  static float panel_width = 360.0f;
  panel_width = std::clamp(panel_width, min_panel_width, max_panel_width);

  ImGui::SetNextWindowPos(ImVec2(std::max(0.0f, screen_width - panel_width), 0.0f), ImGuiCond_Always);
  ImGui::SetNextWindowSize(ImVec2(panel_width, screen_height), ImGuiCond_Always);
  ImGui::SetNextWindowSizeConstraints(
      ImVec2(min_panel_width, 320.0f),
      ImVec2(max_panel_width, screen_height));

  const ImGuiWindowFlags flags =
      ImGuiWindowFlags_NoMove |
      ImGuiWindowFlags_NoCollapse;

  ImGui::Begin("Wire Viewer", nullptr, flags);
  panel_width = ImGui::GetWindowSize().x;
  ImGui::PushTextWrapPos(0.0f);
  if (ImGui::CollapsingHeader("Guide", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::TextUnformatted("Camera");
    ImGui::BulletText("MMB orbit");
    ImGui::BulletText("Shift+MMB pan");
    ImGui::BulletText("Ctrl+MMB dolly");
    ImGui::BulletText("Wheel zoom");
    ImGui::Separator();
    ImGui::TextUnformatted("Modes");
    ImGui::BulletText("Placement: add pole / generate line by form");
    ImGui::BulletText("Connection: connect selected pole ids");
    ImGui::BulletText("Branch: split span and add drop");
    ImGui::BulletText("Detail: inspect and tweak selected pole");
    ImGui::BulletText("DrawPath: click points and press Enter/Generate");
    ImGui::Separator();
    ImGui::TextUnformatted("DrawPath shortcuts");
    ImGui::BulletText("LMB add point, RMB/Backspace undo");
    ImGui::BulletText("Esc clear, Enter generate");
    ImGui::BulletText("Each clicked point becomes one pole");
    ImGui::BulletText("Draw Plane Z controls input height");
    ImGui::BulletText("Direction Mode: Auto/Forward/Reverse");
    ImGui::BulletText("Parallel Spans: 0=auto, 1..8 fixed");
  }
  ImGui::Separator();

  ImGui::Text("Poles: %d", static_cast<int>(state.edit_state().poles.size()));
  ImGui::Text("Ports: %d", static_cast<int>(state.edit_state().ports.size()));
  ImGui::Text("Spans: %d", static_cast<int>(state.edit_state().spans.size()));
  ImGui::Text("Anchors: %d", static_cast<int>(state.edit_state().anchors.size()));
  ImGui::Text("Bundles: %d", static_cast<int>(state.edit_state().bundles.size()));
  ImGui::Text("Attachments: %d", static_cast<int>(state.edit_state().attachments.size()));
  ImGui::Text("Next ID: %llu", static_cast<unsigned long long>(state.next_id()));
  ImGui::Separator();
  ImGui::Text("DirtyQueue Topology: %d", static_cast<int>(state.dirty_queue().topology_dirty_span_ids.size()));
  ImGui::Text("DirtyQueue Geometry: %d", static_cast<int>(state.dirty_queue().geometry_dirty_span_ids.size()));
  ImGui::Text("DirtyQueue Bounds: %d", static_cast<int>(state.dirty_queue().bounds_dirty_span_ids.size()));
  ImGui::Text("DirtyQueue Render: %d", static_cast<int>(state.dirty_queue().render_dirty_span_ids.size()));
  ImGui::Text("DirtyQueue Raycast: %d", static_cast<int>(state.dirty_queue().raycast_dirty_span_ids.size()));
  const auto& recalc = state.last_recalc_stats();
  ImGui::Text("Recalc last frame: %d", static_cast<int>(recalc.total_processed()));
  ImGui::Text("  Geometry processed: %d", static_cast<int>(recalc.geometry_processed));
  ImGui::Text("  Bounds processed: %d", static_cast<int>(recalc.bounds_processed));
  ImGui::Text("  Render processed: %d", static_cast<int>(recalc.render_processed));
  ImGui::Checkbox("Auto Recalc", &ui_state.auto_recalc);
  ImGui::SameLine();
  if (ImGui::Button("Run Recalc")) {
    const auto stats = state.ProcessDirtyQueues();
    PushLog(ui_state, "Recalc processed=" + std::to_string(stats.total_processed()));
  }

  if (!ui_state.geometry_settings_loaded) {
    const auto& gs = state.geometry_settings();
    ui_state.geometry_samples = gs.curve_samples;
    ui_state.geometry_sag_enabled = gs.sag_enabled;
    ui_state.geometry_sag_factor = gs.sag_factor;
    ui_state.geometry_settings_loaded = true;
  }
  if (!ui_state.layout_settings_loaded) {
    const auto& ls = state.layout_settings();
    ui_state.layout_angle_correction_enabled = ls.angle_correction_enabled;
    ui_state.layout_corner_threshold_deg = ls.corner_threshold_deg;
    ui_state.layout_min_side_scale = ls.min_side_scale;
    ui_state.layout_max_side_scale = ls.max_side_scale;
    ui_state.layout_settings_loaded = true;
  }

  ImGui::Separator();
  ImGui::TextUnformatted("Geometry Settings");
  ImGui::SliderInt("Curve Samples", &ui_state.geometry_samples, 2, 64);
  ImGui::Checkbox("Sag Enabled", &ui_state.geometry_sag_enabled);
  ImGui::InputDouble("Sag Factor", &ui_state.geometry_sag_factor, 0.005, 0.01, "%.4f");
  if (ImGui::Button("Apply Geometry")) {
    wire::core::GeometrySettings settings{};
    settings.curve_samples = ui_state.geometry_samples;
    settings.sag_enabled = ui_state.geometry_sag_enabled;
    settings.sag_factor = ui_state.geometry_sag_factor;
    const auto result = state.UpdateGeometrySettings(settings, true);
    if (!result.ok) {
      ui_state.last_error = result.error;
      PushLog(ui_state, "UpdateGeometrySettings failed");
    } else {
      ui_state.last_error.clear();
      PushLog(
          ui_state,
          "Geometry settings updated changed=" +
              std::string(result.value ? "true" : "false") +
              " dirtySpans=" + std::to_string(result.change_set.dirty_span_ids.size()));
    }
  }

  ImGui::Separator();
  ImGui::TextUnformatted("Layout Settings");
  ImGui::Checkbox("Angle Correction Enabled", &ui_state.layout_angle_correction_enabled);
  ImGui::InputDouble("Corner Threshold Deg", &ui_state.layout_corner_threshold_deg, 1.0, 5.0, "%.2f");
  ImGui::InputDouble("Min Side Scale", &ui_state.layout_min_side_scale, 0.05, 0.1, "%.3f");
  ImGui::InputDouble("Max Side Scale", &ui_state.layout_max_side_scale, 0.05, 0.1, "%.3f");
  if (ImGui::Button("Apply Layout")) {
    wire::core::LayoutSettings settings{};
    settings.angle_correction_enabled = ui_state.layout_angle_correction_enabled;
    settings.corner_threshold_deg = ui_state.layout_corner_threshold_deg;
    settings.min_side_scale = ui_state.layout_min_side_scale;
    settings.max_side_scale = ui_state.layout_max_side_scale;
    const auto result = state.UpdateLayoutSettings(settings);
    if (!result.ok) {
      ui_state.last_error = result.error;
      PushLog(ui_state, "UpdateLayoutSettings failed");
    } else {
      ui_state.last_error.clear();
      PushLog(ui_state, "Layout settings updated");
    }
  }
  if (ImGui::Button("Clear Slot Debug Log")) {
    state.clear_slot_selection_debug_records();
    ui_state.selected_slot_debug_index = 0;
  }

  ImGui::Separator();
  ImGui::TextUnformatted("Slot Selection Debug");
  const auto& debug_records = state.slot_selection_debug_records();
  ImGui::Text("Events: %d", static_cast<int>(debug_records.size()));
  if (!debug_records.empty()) {
    ui_state.selected_slot_debug_index =
        std::clamp(ui_state.selected_slot_debug_index, 0, static_cast<int>(debug_records.size() - 1));
    ImGui::SliderInt("Event Index", &ui_state.selected_slot_debug_index, 0, static_cast<int>(debug_records.size() - 1));
    const auto& event = debug_records[static_cast<std::size_t>(ui_state.selected_slot_debug_index)];
    ImGui::Text("Pole=%llu Peer=%llu Ctx=%s Cat=%s", static_cast<unsigned long long>(event.pole_id), static_cast<unsigned long long>(event.peer_pole_id), ContextLabel(event.connection_context), CategoryLabel(event.category));
    ImGui::Text(
        "PoleContext=%s corner=%.2f turn=%.0f sideScale=%.3f",
        PoleContextLabel(event.pole_context),
        event.corner_angle_deg,
        event.corner_turn_sign,
        event.side_scale);
    ImGui::Text("Selected slot=%d result=%s", event.selected_slot_id, event.result.c_str());
    ImGui::BeginChild("SlotScoreLog", ImVec2(0.0f, 160.0f), true, ImGuiWindowFlags_HorizontalScrollbar);
    for (const auto& c : event.candidates) {
      ImGui::Text(
          "slot=%d total=%d eligible=%d tie=%d",
          c.slot_id,
          c.total_score,
          c.eligible ? 1 : 0,
          c.tie_breaker);
      ImGui::Text(
          "cat=%d ctx=%d layer=%d side=%d role=%d pri=%d usage=%d cong=%d",
          c.category_score,
          c.context_score,
          c.layer_score,
          c.side_score,
          c.role_score,
          c.priority_score,
          c.usage_score,
          c.congestion_score);
      ImGui::TextWrapped("reason=%s", c.reason.c_str());
      ImGui::Separator();
    }
    ImGui::EndChild();
  }

  ImGui::Separator();
  ImGui::TextUnformatted("Debug Draw");
  ImGui::Checkbox("Show Span AABB", &ui_state.show_whole_aabb);
  ImGui::Checkbox("Show Segment AABB", &ui_state.show_segment_aabb);

  const wire::core::ValidationResult validation = state.Validate();
  ImGui::Text("Validation: %s", validation.ok() ? "OK" : "ERROR");

  ImGui::Separator();
  ImGui::Text("Mode: %s", ModeLabel(ui_state.mode));
  int mode_index = static_cast<int>(ui_state.mode);
  const std::array<const char*, 5> mode_names = {"Placement", "Connection", "Branch", "Detail", "DrawPath"};
  if (ImGui::BeginCombo("Edit Mode", mode_names[mode_index])) {
    for (int i = 0; i < static_cast<int>(mode_names.size()); ++i) {
      const bool selected = (mode_index == i);
      if (ImGui::Selectable(mode_names[i], selected)) {
        mode_index = i;
      }
      if (selected) {
        ImGui::SetItemDefaultFocus();
      }
    }
    ImGui::EndCombo();
  }
  ui_state.mode = static_cast<EditMode>(std::clamp(mode_index, 0, static_cast<int>(mode_names.size() - 1)));

  ImGui::Separator();
  if (ui_state.mode == EditMode::kPlacement) {
    DrawPlacementModePanel(state, ui_state);
  } else if (ui_state.mode == EditMode::kConnection) {
    DrawConnectionModePanel(state, ui_state);
  } else if (ui_state.mode == EditMode::kBranch) {
    DrawBranchModePanel(state, ui_state);
  } else if (ui_state.mode == EditMode::kDetail) {
    DrawDetailModePanel(state, ui_state);
  } else if (ui_state.mode == EditMode::kDrawPath) {
    DrawPathModePanel(state, ui_state);
  }
  DrawDebugDirectPanel(state, ui_state);

  ImGui::Separator();
  DrawObjectList(
      ui_state,
      "Poles",
      SelectedType::kPole,
      [&]() {
        std::vector<ObjectId> ids;
        ids.reserve(state.edit_state().poles.size());
        for (const auto& pole : state.edit_state().poles.items()) {
          ids.push_back(pole.id);
        }
        return ids;
      }(),
      [&](ObjectId id) {
        const auto* pole = state.edit_state().poles.find(id);
        if (pole == nullptr) {
          return std::to_string(id);
        }
        return pole->display_id + " " + pole->name;
      });

  DrawObjectList(
      ui_state,
      "Ports",
      SelectedType::kPort,
      [&]() {
        std::vector<ObjectId> ids;
        ids.reserve(state.edit_state().ports.size());
        for (const auto& port : state.edit_state().ports.items()) {
          ids.push_back(port.id);
        }
        return ids;
      }(),
      [&](ObjectId id) {
        const auto* port = state.edit_state().ports.find(id);
        if (port == nullptr) {
          return std::to_string(id);
        }
        return port->display_id;
      });

  DrawObjectList(
      ui_state,
      "Spans",
      SelectedType::kSpan,
      [&]() {
        std::vector<ObjectId> ids;
        ids.reserve(state.edit_state().spans.size());
        for (const auto& span : state.edit_state().spans.items()) {
          ids.push_back(span.id);
        }
        return ids;
      }(),
      [&](ObjectId id) {
        const auto* span = state.edit_state().spans.find(id);
        if (span == nullptr) {
          return std::to_string(id);
        }
        return span->display_id;
      });

  DrawSelectedInfo(state, ui_state);
  DrawEditSelectedPanel(state, ui_state);

  if (!ui_state.last_error.empty()) {
    ImGui::Separator();
    ImGui::TextWrapped("Error: %s", ui_state.last_error.c_str());
  }

  ImGui::Separator();
  ImGui::TextUnformatted("Logs");
  ImGui::BeginChild("LogArea", ImVec2(0.0f, 140.0f), true, ImGuiWindowFlags_HorizontalScrollbar);
  for (const std::string& line : ui_state.logs) {
    ImGui::TextWrapped("%s", line.c_str());
  }
  ImGui::EndChild();
  ImGui::PopTextWrapPos();
  ImGui::End();
}

}  // namespace

int main() {
  SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_MSAA_4X_HINT | FLAG_VSYNC_HINT);
  InitWindow(1280, 720, "wire viewer");
  SetExitKey(KEY_NULL);
  SetTargetFPS(60);

  Camera3D camera{};
  camera.position = ToRaylib({10.0, -10.0, 8.0});
  camera.target = ToRaylib({6.0, 0.0, 4.0});
  camera.up = {0.0f, 1.0f, 0.0f};
  camera.fovy = 45.0f;
  camera.projection = CAMERA_PERSPECTIVE;

  CoreState state = wire::core::make_demo_state();
  ViewerUiState ui_state;
  PushLog(ui_state, "[info] viewer started");
  PushLog(ui_state, "[info] demo state loaded");
  PushLog(ui_state, "[mode] Placement/Connection/Branch/Detail/DrawPath");
  PushLog(ui_state, "[flow] Main path: Pole->Pole connection");
  PushLog(ui_state, "[hint] Blender style controls enabled");
  PushLog(ui_state, "[hint] MMB orbit, Shift+MMB pan, Ctrl+MMB dolly");
  PushLog(ui_state, "[hint] Mouse wheel zoom");

  rlImGuiSetup(true);
  while (!WindowShouldClose()) {
    BeginDrawing();
    ClearBackground(Color{26, 32, 39, 255});

    rlImGuiBegin();
    UpdateCameraForViewport(&camera, ui_state);
    UpdateDrawPathInput(state, camera, ui_state);
    if (ui_state.auto_recalc) {
      (void)state.ProcessDirtyQueues();
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
  CloseWindow();
  return 0;
}
