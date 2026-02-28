#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <functional>
#include <sstream>
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
  bool show_selected_wire_group_highlight = true;
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
  std::uint32_t draw_category_mask = (1u << static_cast<int>(wire::core::ConnectionCategory::kLowVoltage));
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
  bool draw_clicked_points_only = false;
  double draw_interval_m = 8.0;
  int draw_parallel_spans = 0; // 0 = category standard lanes
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
  bool ui_unified_workspace = true;
  bool ui_show_workspace = true;
  float ui_workspace_width = 0.0f;
};

struct ViewerPersistentSettings {
  int window_width = 1280;
  int window_height = 720;
  bool ui_unified_workspace = true;
  bool ui_show_workspace = true;
  float ui_workspace_width = 420.0f;
};

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
}

void PushLog(ViewerUiState& ui_state, const std::string& line);

constexpr std::array<wire::core::ConnectionCategory, 5> kAllCategories = {
    wire::core::ConnectionCategory::kHighVoltage,   wire::core::ConnectionCategory::kLowVoltage,
    wire::core::ConnectionCategory::kCommunication, wire::core::ConnectionCategory::kOptical,
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

const char* PolePlacementModeLabel(wire::core::PlacementMode mode) {
  switch (mode) {
  case wire::core::PlacementMode::kAuto:
    return "Auto";
  case wire::core::PlacementMode::kManual:
    return "Manual";
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

const char* PortPositionModeLabel(wire::core::PortPositionMode mode) {
  switch (mode) {
  case wire::core::PortPositionMode::kAuto:
    return "Auto";
  case wire::core::PortPositionMode::kManual:
    return "Manual";
  default:
    return "Unknown";
  }
}

const char* PortPlacementSourceLabel(wire::core::PortPlacementSourceKind source) {
  switch (source) {
  case wire::core::PortPlacementSourceKind::kTemplateSlot:
    return "TemplateSlot";
  case wire::core::PortPlacementSourceKind::kGenerated:
    return "Generated";
  case wire::core::PortPlacementSourceKind::kManualEdit:
    return "ManualEdit";
  case wire::core::PortPlacementSourceKind::kAerialBranch:
    return "AerialBranch";
  case wire::core::PortPlacementSourceKind::kUnknown:
  default:
    return "Unknown";
  }
}

const char* WireGroupKindLabel(wire::core::WireGroupKind kind) {
  switch (kind) {
  case wire::core::WireGroupKind::kPowerHighVoltage:
    return "PowerHighVoltage";
  case wire::core::WireGroupKind::kPowerLowVoltage:
    return "PowerLowVoltage";
  case wire::core::WireGroupKind::kComm:
    return "Comm";
  case wire::core::WireGroupKind::kOptical:
    return "Optical";
  case wire::core::WireGroupKind::kUnknown:
  default:
    return "Unknown";
  }
}

const char* WireLaneRoleLabel(wire::core::WireLaneRole role) {
  switch (role) {
  case wire::core::WireLaneRole::kPhaseA:
    return "PhaseA";
  case wire::core::WireLaneRole::kPhaseB:
    return "PhaseB";
  case wire::core::WireLaneRole::kPhaseC:
    return "PhaseC";
  case wire::core::WireLaneRole::kNeutral:
    return "Neutral";
  case wire::core::WireLaneRole::kCommLine:
    return "CommLine";
  case wire::core::WireLaneRole::kOpticalFiber:
    return "OpticalFiber";
  case wire::core::WireLaneRole::kAux:
    return "Aux";
  case wire::core::WireLaneRole::kUnknown:
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
    return 2;
  case wire::core::ConnectionCategory::kCommunication:
    return 1;
  case wire::core::ConnectionCategory::kOptical:
    return 1;
  case wire::core::ConnectionCategory::kDrop:
    return 1;
  default:
    return 1;
  }
}

int AutoParallelSpanCountFromPoleType(const CoreState& state, wire::core::PoleTypeId pole_type_id,
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

bool IsDrawCategorySelected(const ViewerUiState& ui_state, int category_index) {
  if (category_index < 0 || category_index >= static_cast<int>(kAllCategories.size())) {
    return false;
  }
  return (ui_state.draw_category_mask & (1u << category_index)) != 0u;
}

void SetDrawCategorySelected(ViewerUiState& ui_state, int category_index, bool selected) {
  if (category_index < 0 || category_index >= static_cast<int>(kAllCategories.size())) {
    return;
  }
  const std::uint32_t bit = (1u << category_index);
  if (selected) {
    ui_state.draw_category_mask |= bit;
  } else {
    ui_state.draw_category_mask &= ~bit;
  }
}

std::vector<wire::core::ConnectionCategory> SelectedDrawCategories(const ViewerUiState& ui_state) {
  std::vector<wire::core::ConnectionCategory> categories;
  for (int i = 0; i < static_cast<int>(kAllCategories.size()); ++i) {
    if (IsDrawCategorySelected(ui_state, i)) {
      categories.push_back(kAllCategories[i]);
    }
  }
  return categories;
}

std::string DrawCategoryPreview(const ViewerUiState& ui_state) {
  const auto categories = SelectedDrawCategories(ui_state);
  if (categories.empty()) {
    return "(none)";
  }
  if (categories.size() == 1) {
    return CategoryLabel(categories.front());
  }
  std::ostringstream oss;
  for (std::size_t i = 0; i < categories.size(); ++i) {
    if (i > 0) {
      oss << ", ";
    }
    oss << CategoryLabel(categories[i]);
  }
  return oss.str();
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
  const float plane_y = static_cast<float>(ue_plane_z); // raylib y == UE z
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

  wire::core::RoadSegment road{};
  road.id = ui_state.road_id++;
  road.polyline = ui_state.draw_path_points;

  const auto selected_categories = SelectedDrawCategories(ui_state);
  if (selected_categories.empty()) {
    ui_state.last_error = "select at least one path category";
    return;
  }
  const int mode_index = std::clamp(ui_state.draw_direction_mode, 0, 2);
  ui_state.draw_direction_mode = mode_index;

  int total_generated_poles = 0;
  int total_generated_spans = 0;
  std::size_t success_categories = 0;
  std::ostringstream failure_details;
  bool has_failure = false;

  for (const auto category : selected_categories) {
    int parallel_spans = ui_state.draw_parallel_spans;
    if (parallel_spans <= 0) {
      parallel_spans = FallbackParallelSpanCount(category);
    }
    parallel_spans = std::clamp(parallel_spans, 0, 8);

    wire::core::CoreState::GenerateWireGroupFromPathInput input{};
    input.polyline = road.polyline;
    if (ui_state.draw_clicked_points_only) {
      // Disable intermediate poles by using an interval longer than the whole path.
      input.interval_m = std::max(0.001, PolylineLength(road.polyline) + 1.0);
    } else {
      input.interval_m = ui_state.draw_interval_m;
    }
    input.pole_type_id = type_ids[ui_state.road_pole_type_index];
    input.category = category;
    input.direction_mode = static_cast<wire::core::PathDirectionMode>(mode_index);
    input.requested_lane_count = parallel_spans;

    const auto result = state.GenerateWireGroupFromPath(input);
    if (!result.ok) {
      has_failure = true;
      failure_details << CategoryLabel(category) << ": " << result.error << "; ";
      continue;
    }
    ++success_categories;
    total_generated_poles += static_cast<int>(result.value.generated_pole_ids.size());
    total_generated_spans += static_cast<int>(result.value.generated_span_ids.size());
    if (!result.value.generated_pole_ids.empty()) {
      ui_state.selected_type = SelectedType::kPole;
      ui_state.selected_id = result.value.generated_pole_ids.back();
    } else if (!result.value.generated_span_ids.empty()) {
      ui_state.selected_type = SelectedType::kSpan;
      ui_state.selected_id = result.value.generated_span_ids.back();
    }
  }

  if (success_categories == 0) {
    ui_state.last_error = failure_details.str();
    PushLog(ui_state, from_enter_key ? "Generate path (Enter) failed" : "Generate path failed");
    return;
  }

  ui_state.last_error = has_failure ? failure_details.str() : "";
  ui_state.last_generated_poles = total_generated_poles;
  ui_state.last_generated_spans = total_generated_spans;
  ui_state.last_generation_session = 0;

  if (!ui_state.draw_keep_path_after_generate) {
    ui_state.draw_path_points.clear();
  }
  PushLog(ui_state, "Generated path categories=" + std::to_string(success_categories) +
                        " poles=" + std::to_string(ui_state.last_generated_poles) +
                        " spans=" + std::to_string(ui_state.last_generated_spans));
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
      DrawLine3D(ToRaylib(ui_state.draw_path_points[i]), ToRaylib(ui_state.draw_path_points[i + 1]),
                 Color{255, 220, 80, 255});
    }
  }
  if (ui_state.draw_hover_valid) {
    DrawSphere(ToRaylib(ui_state.draw_hover_point), 0.08f, Color{120, 255, 180, 200});
    if (!ui_state.draw_path_points.empty()) {
      DrawLine3D(ToRaylib(ui_state.draw_path_points.back()), ToRaylib(ui_state.draw_hover_point),
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
  if (wire::core::any(bits, wire::core::DirtyBits::kTopology))
    text += "Topology|";
  if (wire::core::any(bits, wire::core::DirtyBits::kGeometry))
    text += "Geometry|";
  if (wire::core::any(bits, wire::core::DirtyBits::kBounds))
    text += "Bounds|";
  if (wire::core::any(bits, wire::core::DirtyBits::kRender))
    text += "Render|";
  if (wire::core::any(bits, wire::core::DirtyBits::kRaycast))
    text += "Raycast|";
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
  if (wire::core::any(runtime_state->dirty_bits, wire::core::DirtyBits::kTopology))
    return PURPLE;
  if (wire::core::any(runtime_state->dirty_bits, wire::core::DirtyBits::kGeometry))
    return YELLOW;
  if (wire::core::any(runtime_state->dirty_bits, wire::core::DirtyBits::kBounds))
    return BLUE;
  if (wire::core::any(runtime_state->dirty_bits, wire::core::DirtyBits::kRender))
    return RED;
  if (wire::core::any(runtime_state->dirty_bits, wire::core::DirtyBits::kRaycast))
    return GREEN;
  return SKYBLUE;
}

void DrawAxes() {
  DrawLine3D(ToRaylib({0.0, 0.0, 0.0}), ToRaylib({kAxisLength, 0.0, 0.0}), RED);
  DrawLine3D(ToRaylib({0.0, 0.0, 0.0}), ToRaylib({0.0, kAxisLength, 0.0}), GREEN);
  DrawLine3D(ToRaylib({0.0, 0.0, 0.0}), ToRaylib({0.0, 0.0, kAxisLength}), BLUE);
}

void DrawCore(const CoreState& state, const ViewerUiState& ui_state) {
  const auto& edit = state.edit_state();
  ObjectId selected_wire_group_id = wire::core::kInvalidObjectId;
  if (ui_state.selected_type == SelectedType::kSpan && ui_state.selected_id != wire::core::kInvalidObjectId) {
    const wire::core::Span* selected_span = edit.spans.find(ui_state.selected_id);
    if (selected_span != nullptr) {
      selected_wire_group_id = selected_span->wire_group_id;
    }
  }

  for (const wire::core::Pole& pole : edit.poles.items()) {
    // raylib DrawCylinderWires uses base-center as position.
    const wire::core::Vec3d pole_base_ue{
        pole.world_transform.position.x,
        pole.world_transform.position.y,
        pole.world_transform.position.z,
    };

    Color color = DARKGRAY;
    if (ui_state.selected_type == SelectedType::kPole && ui_state.selected_id == pole.id) {
      color = GOLD;
    }
    DrawCylinderWires(ToRaylib(pole_base_ue), 0.12f, 0.12f, static_cast<float>(pole.height_m), 10, color);
  }

  for (const wire::core::Port& port : edit.ports.items()) {
    Color color = (port.position_mode == wire::core::PortPositionMode::kManual) ? MAGENTA : ORANGE;
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
    } else if (ui_state.show_selected_wire_group_highlight && selected_wire_group_id != wire::core::kInvalidObjectId &&
               span.wire_group_id == selected_wire_group_id) {
      color = ORANGE;
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

void DrawObjectList(ViewerUiState& ui_state, const char* header, SelectedType type, const std::vector<ObjectId>& ids,
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
    ImGui::Text("Pos: %.2f %.2f %.2f", pole->world_transform.position.x, pole->world_transform.position.y,
                pole->world_transform.position.z);
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
    ImGui::Text("Layer: %d Side: %s Role: %s", port->template_layer, SlotSideLabel(port->template_side),
                SlotRoleLabel(port->template_role));
    ImGui::Text("PositionMode: %s", PortPositionModeLabel(port->position_mode));
    ImGui::Text("PlacementSource: %s", PortPlacementSourceLabel(port->placement_source));
    ImGui::Text("UserEditedPos: %s", port->user_edited_position ? "true" : "false");
    ImGui::Text("GeneratedByRule: %s", port->generated_by_rule ? "true" : "false");
    ImGui::Text("PlacementContext: %s", ContextLabel(port->placement_context));
    ImGui::Text("AngleCorrected: %s sideScale=%.3f", port->angle_correction_applied ? "true" : "false",
                port->side_scale_applied);
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
    ImGui::Text("wireGroup: %llu", static_cast<unsigned long long>(span->wire_group_id));
    ImGui::Text("wireLane: %llu", static_cast<unsigned long long>(span->wire_lane_id));
    if (span->wire_group_id != wire::core::kInvalidObjectId) {
      const auto* group = state.GetWireGroup(span->wire_group_id);
      if (group != nullptr) {
        ImGui::Text("wireGroup kind: %s", WireGroupKindLabel(group->kind));
      } else {
        ImGui::TextUnformatted("wireGroup: missing");
      }
    } else {
      ImGui::TextUnformatted("wireGroup: unassigned");
    }
    if (span->wire_lane_id != wire::core::kInvalidObjectId) {
      const auto* lane = state.GetWireLane(span->wire_lane_id);
      if (lane != nullptr) {
        ImGui::Text("wireLane index=%d role=%s", lane->lane_index, WireLaneRoleLabel(lane->role));
      } else {
        ImGui::TextUnformatted("wireLane: missing");
      }
    } else {
      ImGui::TextUnformatted("wireLane: unassigned");
    }
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
        PushLog(ui_state, "Moved Port (Manual) id=" + std::to_string(ui_state.selected_id));
      }
    }
    if (ImGui::Button("Reset Port To Auto")) {
      const auto result = state.ResetPortPositionToAuto(ui_state.selected_id);
      if (!result.ok) {
        HandleResultError(ui_state, result.error, "Reset Port To Auto failed");
      } else {
        ui_state.last_error.clear();
        PushLog(ui_state, "Reset Port To Auto id=" + std::to_string(ui_state.selected_id));
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
  const std::string selected_type_name =
      (type_it != state.pole_types().end()) ? type_it->second.name : std::to_string(selected_type_id);

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
    const auto add_pole_result =
        state.AddPole(tf, ui_state.pole_height, "Pole", static_cast<wire::core::PoleKind>(ui_state.pole_kind));
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
  const std::string road_type_name =
      (road_type_it != state.pole_types().end()) ? road_type_it->second.name : std::to_string(road_type_id);
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

  const int road_category_index =
      std::clamp(ui_state.road_category_index, 0, static_cast<int>(kAllCategories.size() - 1));
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

    const auto result = state.GenerateSimpleLine(road, ui_state.road_interval, type_ids[ui_state.road_pole_type_index],
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
      PushLog(ui_state, "Generated road poles=" + std::to_string(ui_state.last_generated_poles) +
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
  const int context_index =
      std::clamp(ui_state.connect_context, 0, static_cast<int>(kAllConnectionContexts.size() - 1));
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
    const auto result = state.AddConnectionByPole(ui_state.connect_pole_a_id, ui_state.connect_pole_b_id,
                                                  kAllCategories[ui_state.connect_category], options);
    if (!result.ok) {
      ui_state.last_error = result.error;
      PushLog(ui_state, "Connect Pole->Pole failed");
    } else {
      ui_state.last_error.clear();
      ui_state.selected_type = SelectedType::kSpan;
      ui_state.selected_id = result.value.span_id;
      PushLog(ui_state, "Connected span=" + std::to_string(result.value.span_id) +
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
  ImGui::InputDouble("Path Interval (m)", &ui_state.draw_interval_m, 0.5, 1.0, "%.2f");
  ui_state.draw_interval_m = std::max(0.0, ui_state.draw_interval_m);
  ImGui::Checkbox("Clicked Points Only (No Intermediate Pole)", &ui_state.draw_clicked_points_only);
  ImGui::Checkbox("Show Preview", &ui_state.draw_show_preview);
  ImGui::Checkbox("Keep Path After Generate", &ui_state.draw_keep_path_after_generate);
  ImGui::InputInt("Lane Override (0=category standard)", &ui_state.draw_parallel_spans);
  ui_state.draw_parallel_spans = std::clamp(ui_state.draw_parallel_spans, 0, 8);
  ImGui::TextUnformatted("Generate by WireGroup/WireLane (not per-wire manual placement)");
  ImGui::Text("Path points: %d", static_cast<int>(ui_state.draw_path_points.size()));
  if (ui_state.draw_hover_valid) {
    ImGui::Text("Hover: %.2f %.2f %.2f", ui_state.draw_hover_point.x, ui_state.draw_hover_point.y,
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

  if (ImGui::BeginCombo("Path Categories", DrawCategoryPreview(ui_state).c_str())) {
    for (int i = 0; i < static_cast<int>(kAllCategories.size()); ++i) {
      bool selected = IsDrawCategorySelected(ui_state, i);
      if (ImGui::Selectable(CategoryLabel(kAllCategories[i]), selected, ImGuiSelectableFlags_DontClosePopups)) {
        selected = !selected;
        SetDrawCategorySelected(ui_state, i, selected);
      }
    }
    ImGui::EndCombo();
  }
  const auto selected_categories = SelectedDrawCategories(ui_state);
  if (selected_categories.empty()) {
    ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.35f, 1.0f), "Select at least one category");
  } else {
    ImGui::Text("Selected Categories: %d", static_cast<int>(selected_categories.size()));
  }
  if (ImGui::BeginCombo("Direction Mode", PathDirectionModeLabel(static_cast<wire::core::PathDirectionMode>(
                                              ui_state.draw_direction_mode)))) {
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
      type_ids.empty() ? wire::core::kInvalidPoleTypeId
                       : type_ids[ClampedTypeIndex(ui_state.road_pole_type_index, type_ids.size())];
  (void)resolved_type_id;
  if (!selected_categories.empty()) {
    std::ostringstream lane_info;
    lane_info << "Resolved Group Lanes: ";
    for (std::size_t i = 0; i < selected_categories.size(); ++i) {
      const auto category = selected_categories[i];
      const int auto_parallel = FallbackParallelSpanCount(category);
      const int resolved_parallel = (ui_state.draw_parallel_spans > 0) ? ui_state.draw_parallel_spans : auto_parallel;
      if (i > 0) {
        lane_info << " / ";
      }
      lane_info << CategoryLabel(category) << "=" << resolved_parallel;
    }
    ImGui::TextUnformatted(lane_info.str().c_str());
  }
  if (!ui_state.draw_clicked_points_only && ui_state.draw_interval_m <= 0.0) {
    ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.35f, 1.0f), "Path Interval must be > 0");
  }
  if (ImGui::Button("Flip Direction (Manual)")) {
    if (ui_state.draw_direction_mode == static_cast<int>(wire::core::PathDirectionMode::kReverse)) {
      ui_state.draw_direction_mode = static_cast<int>(wire::core::PathDirectionMode::kForward);
    } else {
      ui_state.draw_direction_mode = static_cast<int>(wire::core::PathDirectionMode::kReverse);
    }
  }
  const auto& dir_debug = state.last_path_direction_debug();
  ImGui::Text("Direction chosen: %s (mode=%s)", PathDirectionChosenLabel(dir_debug.chosen),
              PathDirectionModeLabel(dir_debug.requested_mode));
  ImGui::Text("Cost F/R: %d / %d", dir_debug.forward_cost.total, dir_debug.reverse_cost.total);
  ImGui::Text("Cost detail cross=%d side=%d layer=%d corner=%d branch=%d",
              dir_debug.forward_cost.estimated_cross_penalty, dir_debug.forward_cost.side_flip_penalty,
              dir_debug.forward_cost.layer_jump_penalty, dir_debug.forward_cost.corner_compression_penalty,
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
    ImGui::Text("seg=%d A=%llu B=%llu lanes=%d mirrored=%s", static_cast<int>(a.segment_index),
                static_cast<unsigned long long>(a.pole_a_id), static_cast<unsigned long long>(a.pole_b_id),
                static_cast<int>(a.port_ids_a.size()), a.mirrored ? "true" : "false");
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
    const auto result =
        state.AddDropFromSpan(ui_state.branch_source_span_id, ui_state.branch_t,
                              {ui_state.branch_target_x, ui_state.branch_target_y, ui_state.branch_target_z},
                              wire::core::ConnectionCategory::kDrop);
    if (!result.ok) {
      ui_state.last_error = result.error;
      PushLog(ui_state, "AddDropFromSpan failed");
    } else {
      ui_state.last_error.clear();
      ui_state.selected_type = SelectedType::kSpan;
      ui_state.selected_id = result.value.span_id;
      PushLog(ui_state, "DropFromSpan span=" + std::to_string(result.value.span_id) +
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
    const auto result = state.AddDropFromPole(ui_state.drop_source_pole_id,
                                              {ui_state.drop_target_x, ui_state.drop_target_y, ui_state.drop_target_z},
                                              wire::core::ConnectionCategory::kDrop);
    if (!result.ok) {
      ui_state.last_error = result.error;
      PushLog(ui_state, "AddDropFromPole failed");
    } else {
      ui_state.last_error.clear();
      ui_state.selected_type = SelectedType::kSpan;
      ui_state.selected_id = result.value.span_id;
      PushLog(ui_state, "DropFromPole span=" + std::to_string(result.value.span_id) +
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
  wire::core::PlacementMode placement_mode = detail.pole->placement_mode;
  if (ImGui::BeginCombo("Placement Mode", PolePlacementModeLabel(placement_mode))) {
    for (int i = 0; i < 2; ++i) {
      const auto candidate = static_cast<wire::core::PlacementMode>(i);
      const bool selected = (candidate == placement_mode);
      if (ImGui::Selectable(PolePlacementModeLabel(candidate), selected)) {
        const auto mode_result = state.SetPolePlacementMode(ui_state.selected_id, candidate);
        if (!mode_result.ok) {
          ui_state.last_error = mode_result.error;
          PushLog(ui_state, "SetPolePlacementMode failed");
        } else {
          ui_state.last_error.clear();
          PushLog(ui_state, std::string("SetPolePlacementMode -> ") + PolePlacementModeLabel(candidate));
          placement_mode = candidate;
        }
      }
      if (selected) {
        ImGui::SetItemDefaultFocus();
      }
    }
    ImGui::EndCombo();
  }
  ImGui::Text("UserEditedPole: %s", detail.pole->user_edited ? "true" : "false");
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
    const std::string selected_name =
        (type_it != state.pole_types().end()) ? type_it->second.name : std::to_string(type_ids[idx]);
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
        ImGui::Text("slot=%d cat=%s layer=%d side=%s role=%s used=0 [empty]", slot.slot_id,
                    CategoryLabel(slot.category), slot.layer, SlotSideLabel(slot.side), SlotRoleLabel(slot.role));
        continue;
      }
      const auto* port = it->second;
      const auto usage_it = state.connection_index().spans_by_port.find(port->id);
      const int usage =
          (usage_it == state.connection_index().spans_by_port.end()) ? 0 : static_cast<int>(usage_it->second.size());
      ImGui::Text("slot=%d cat=%s layer=%d side=%s role=%s used=%d -> %s", slot.slot_id, CategoryLabel(slot.category),
                  slot.layer, SlotSideLabel(slot.side), SlotRoleLabel(slot.role), usage, port->display_id.c_str());
    }
  } else {
    for (const auto* port : detail.owned_ports) {
      const auto it = state.connection_index().spans_by_port.find(port->id);
      const int usage = (it == state.connection_index().spans_by_port.end()) ? 0 : static_cast<int>(it->second.size());
      ImGui::Text("%s cat=%s slot=%d used=%d", port->display_id.c_str(), CategoryLabel(port->category),
                  port->source_slot_id, usage);
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
    const auto result = state.AddPort(ui_state.port_owner_pole_id, {ui_state.port_x, ui_state.port_y, ui_state.port_z},
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
    const auto result = state.AddSpan(ui_state.span_port_a_id, ui_state.span_port_b_id,
                                      static_cast<wire::core::SpanKind>(ui_state.span_kind),
                                      static_cast<wire::core::SpanLayer>(ui_state.span_layer), ui_state.span_bundle_id);
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

void DrawModeButtons(ViewerUiState& ui_state) {
  const std::array<std::pair<EditMode, const char*>, 5> modes = {{
      {EditMode::kPlacement, "Placement"},
      {EditMode::kConnection, "Connection"},
      {EditMode::kBranch, "Branch"},
      {EditMode::kDetail, "Detail"},
      {EditMode::kDrawPath, "DrawPath"},
  }};
  for (std::size_t i = 0; i < modes.size(); ++i) {
    const bool active = (ui_state.mode == modes[i].first);
    if (active) {
      ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.22f, 0.34f, 0.48f, 1.0f));
      ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.26f, 0.40f, 0.58f, 1.0f));
    }
    if (ImGui::Button(modes[i].second)) {
      ui_state.mode = modes[i].first;
    }
    if (active) {
      ImGui::PopStyleColor(2);
    }
    if (i + 1 < modes.size()) {
      ImGui::SameLine();
    }
  }
}

void DrawTopbarWindow(const CoreState& state, ViewerUiState& ui_state) {
  const float w = static_cast<float>(GetScreenWidth());
  const float topbar_h = 74.0f;
  ImGui::SetNextWindowPos(ImVec2(8.0f, 8.0f), ImGuiCond_Always);
  ImGui::SetNextWindowSize(ImVec2(std::max(320.0f, w - 16.0f), topbar_h), ImGuiCond_Always);
  const ImGuiWindowFlags flags =
      ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize;
  if (!ImGui::Begin("Topbar", nullptr, flags)) {
    ImGui::End();
    return;
  }

  DrawModeButtons(ui_state);
  ImGui::SameLine();
  ImGui::SetCursorPosX(std::max(ImGui::GetCursorPosX(), ImGui::GetWindowWidth() - 250.0f));
  ImGui::Checkbox("Unified UI", &ui_state.ui_unified_workspace);
  ImGui::SameLine();
  ImGui::Checkbox("Show Workspace", &ui_state.ui_show_workspace);
  if (ui_state.ui_unified_workspace) {
    ImGui::SameLine();
    ImGui::SetNextItemWidth(140.0f);
    if (ui_state.ui_workspace_width <= 1.0f) {
      ui_state.ui_workspace_width = 420.0f;
    }
    ImGui::SliderFloat("##WorkspaceWidth", &ui_state.ui_workspace_width, 300.0f, 760.0f, "W %.0f");
  }
  ImGui::Separator();
  ImGui::Text("Poles:%d  Ports:%d  Spans:%d  Groups:%d  Lanes:%d",
              static_cast<int>(state.edit_state().poles.size()), static_cast<int>(state.edit_state().ports.size()),
              static_cast<int>(state.edit_state().spans.size()), static_cast<int>(state.edit_state().wire_groups.size()),
              static_cast<int>(state.edit_state().wire_lanes.size()));
  ImGui::SameLine();
  ImGui::Text("|  Mode: %s", ModeLabel(ui_state.mode));
  if (ui_state.selected_type == SelectedType::kPole) {
    if (const auto* pole = state.edit_state().poles.find(ui_state.selected_id); pole != nullptr) {
      ImGui::SameLine();
      ImGui::Text("|  Selected Pole Placement: %s", PolePlacementModeLabel(pole->placement_mode));
    }
  }
  ImGui::End();
}

void DrawToolboxContent(CoreState& state, ViewerUiState& ui_state) {
  ImGui::Text("Active Tool: %s", ModeLabel(ui_state.mode));
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
}

void DrawToolboxWindow(CoreState& state, ViewerUiState& ui_state) {
  ImGui::SetNextWindowPos(ImVec2(8.0f, 90.0f), ImGuiCond_FirstUseEver);
  ImGui::SetNextWindowSize(ImVec2(420.0f, 520.0f), ImGuiCond_FirstUseEver);
  const ImGuiWindowFlags flags = ImGuiWindowFlags_NoCollapse;
  if (!ImGui::Begin("Toolbox", nullptr, flags)) {
    ImGui::End();
    return;
  }
  DrawToolboxContent(state, ui_state);
  ImGui::End();
}

void DrawInspectorContent(CoreState& state, ViewerUiState& ui_state) {
  if (ui_state.selected_type == SelectedType::kPole &&
      ui_state.selected_id != wire::core::kInvalidObjectId) {
    if (const auto* pole = state.edit_state().poles.find(ui_state.selected_id); pole != nullptr) {
      int placement_mode = static_cast<int>(pole->placement_mode);
      if (ImGui::Combo("Placement Mode (Quick)", &placement_mode, "Auto\0Manual\0")) {
        const auto result =
            state.SetPolePlacementMode(ui_state.selected_id, static_cast<wire::core::PlacementMode>(placement_mode));
        if (!result.ok) {
          ui_state.last_error = result.error;
          PushLog(ui_state, "SetPolePlacementMode failed");
        } else {
          ui_state.last_error.clear();
          PushLog(ui_state, "SetPolePlacementMode updated");
        }
      }
    }
  }

  DrawSelectedInfo(state, ui_state);
  DrawEditSelectedPanel(state, ui_state);
}

void DrawInspectorWindow(CoreState& state, ViewerUiState& ui_state) {
  const float w = static_cast<float>(GetScreenWidth());
  ImGui::SetNextWindowPos(ImVec2(std::max(440.0f, w - 430.0f), 90.0f), ImGuiCond_FirstUseEver);
  ImGui::SetNextWindowSize(ImVec2(420.0f, 620.0f), ImGuiCond_FirstUseEver);
  const ImGuiWindowFlags flags = ImGuiWindowFlags_NoCollapse;
  if (!ImGui::Begin("Inspector", nullptr, flags)) {
    ImGui::End();
    return;
  }
  DrawInspectorContent(state, ui_state);
  ImGui::End();
}

void DrawOutlinerContent(CoreState& state, ViewerUiState& ui_state) {
  DrawObjectList(
      ui_state, "Poles", SelectedType::kPole,
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
      ui_state, "Ports", SelectedType::kPort,
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
      ui_state, "Spans", SelectedType::kSpan,
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
}

void DrawOutlinerWindow(CoreState& state, ViewerUiState& ui_state) {
  ImGui::SetNextWindowPos(ImVec2(8.0f, 620.0f), ImGuiCond_FirstUseEver);
  ImGui::SetNextWindowSize(ImVec2(420.0f, 260.0f), ImGuiCond_FirstUseEver);
  const ImGuiWindowFlags flags = ImGuiWindowFlags_NoCollapse;
  if (!ImGui::Begin("Outliner", nullptr, flags)) {
    ImGui::End();
    return;
  }
  DrawOutlinerContent(state, ui_state);
  ImGui::End();
}

void DrawDiagnosticsContent(CoreState& state, ViewerUiState& ui_state) {
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

  const auto& recalc = state.last_recalc_stats();
  ImGui::Text("Dirty T/G/B/R/X: %d / %d / %d / %d / %d",
              static_cast<int>(state.dirty_queue().topology_dirty_span_ids.size()),
              static_cast<int>(state.dirty_queue().geometry_dirty_span_ids.size()),
              static_cast<int>(state.dirty_queue().bounds_dirty_span_ids.size()),
              static_cast<int>(state.dirty_queue().render_dirty_span_ids.size()),
              static_cast<int>(state.dirty_queue().raycast_dirty_span_ids.size()));
  ImGui::Text("Last Recalc total=%d geom=%d bounds=%d render=%d", static_cast<int>(recalc.total_processed()),
              static_cast<int>(recalc.geometry_processed), static_cast<int>(recalc.bounds_processed),
              static_cast<int>(recalc.render_processed));
  ImGui::Checkbox("Auto Recalc", &ui_state.auto_recalc);
  ImGui::SameLine();
  if (ImGui::Button("Run Recalc")) {
    const auto stats = state.ProcessDirtyQueues();
    PushLog(ui_state, "Recalc processed=" + std::to_string(stats.total_processed()));
  }

  if (ImGui::CollapsingHeader("Geometry/Layout", ImGuiTreeNodeFlags_DefaultOpen)) {
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
        PushLog(ui_state, "Geometry settings updated");
      }
    }
    ImGui::Separator();
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
  }

  if (ImGui::CollapsingHeader("Debug View", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::Checkbox("Show Span AABB", &ui_state.show_whole_aabb);
    ImGui::Checkbox("Show Segment AABB", &ui_state.show_segment_aabb);
    ImGui::Checkbox("Highlight Selected WireGroup", &ui_state.show_selected_wire_group_highlight);
    const wire::core::ValidationResult validation = state.Validate();
    ImGui::Text("Validation: %s", validation.ok() ? "OK" : "ERROR");
  }

  if (ImGui::CollapsingHeader("Slot Selection Debug")) {
    if (ImGui::Button("Clear Slot Debug Log")) {
      state.clear_slot_selection_debug_records();
      ui_state.selected_slot_debug_index = 0;
    }
    const auto& debug_records = state.slot_selection_debug_records();
    ImGui::Text("Events: %d", static_cast<int>(debug_records.size()));
    if (!debug_records.empty()) {
      ui_state.selected_slot_debug_index =
          std::clamp(ui_state.selected_slot_debug_index, 0, static_cast<int>(debug_records.size() - 1));
      ImGui::SliderInt("Event Index", &ui_state.selected_slot_debug_index, 0,
                       static_cast<int>(debug_records.size() - 1));
      const auto& event = debug_records[static_cast<std::size_t>(ui_state.selected_slot_debug_index)];
      ImGui::Text("Pole=%llu Peer=%llu Ctx=%s Cat=%s", static_cast<unsigned long long>(event.pole_id),
                  static_cast<unsigned long long>(event.peer_pole_id), ContextLabel(event.connection_context),
                  CategoryLabel(event.category));
      ImGui::Text("Selected slot=%d result=%s", event.selected_slot_id, event.result.c_str());
    }
  }

  DrawDebugDirectPanel(state, ui_state);

  if (!ui_state.last_error.empty()) {
    ImGui::Separator();
    ImGui::TextWrapped("Error: %s", ui_state.last_error.c_str());
  }
  ImGui::Separator();
  ImGui::BeginChild("LogArea", ImVec2(0.0f, 90.0f), true, ImGuiWindowFlags_HorizontalScrollbar);
  for (const std::string& line : ui_state.logs) {
    ImGui::TextWrapped("%s", line.c_str());
  }
  ImGui::EndChild();
}

void DrawDiagnosticsWindow(CoreState& state, ViewerUiState& ui_state) {
  const float w = static_cast<float>(GetScreenWidth());
  const float h = static_cast<float>(GetScreenHeight());
  ImGui::SetNextWindowPos(ImVec2(std::max(440.0f, w - 430.0f), std::max(90.0f, h - 360.0f)), ImGuiCond_FirstUseEver);
  ImGui::SetNextWindowSize(ImVec2(420.0f, 340.0f), ImGuiCond_FirstUseEver);
  const ImGuiWindowFlags flags = ImGuiWindowFlags_NoCollapse;
  if (!ImGui::Begin("Diagnostics", nullptr, flags)) {
    ImGui::End();
    return;
  }
  DrawDiagnosticsContent(state, ui_state);
  ImGui::End();
}

void DrawUnifiedWorkspaceWindow(CoreState& state, ViewerUiState& ui_state) {
  if (!ui_state.ui_show_workspace) {
    return;
  }
  const float screen_w = static_cast<float>(GetScreenWidth());
  const float screen_h = static_cast<float>(GetScreenHeight());
  const float topbar_h = 74.0f;
  const float margin = 8.0f;
  const float min_w = 300.0f;
  const float max_w = std::max(min_w, screen_w - margin * 2.0f);
  if (ui_state.ui_workspace_width <= 1.0f) {
    ui_state.ui_workspace_width = std::clamp(screen_w * 0.36f, min_w, std::min(760.0f, max_w));
  }
  ui_state.ui_workspace_width = std::clamp(ui_state.ui_workspace_width, min_w, std::min(760.0f, max_w));
  const float workspace_w = ui_state.ui_workspace_width;
  const float x = std::max(margin, screen_w - workspace_w - margin);
  const float y = topbar_h + margin + 8.0f;
  const float h = std::max(240.0f, screen_h - y - margin);

  ImGui::SetNextWindowPos(ImVec2(x, y), ImGuiCond_Always);
  ImGui::SetNextWindowSize(ImVec2(workspace_w, h), ImGuiCond_FirstUseEver);
  ImGui::SetNextWindowSizeConstraints(ImVec2(min_w, 240.0f), ImVec2(std::min(760.0f, max_w), h));
  const ImGuiWindowFlags flags = ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoMove;
  if (!ImGui::Begin("Workspace", nullptr, flags)) {
    ImGui::End();
    return;
  }
  ui_state.ui_workspace_width = ImGui::GetWindowSize().x;
  if (ImGui::BeginTabBar("WorkspaceTabs")) {
    if (ImGui::BeginTabItem("Toolbox")) {
      DrawToolboxContent(state, ui_state);
      ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem("Inspector")) {
      DrawInspectorContent(state, ui_state);
      ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem("Outliner")) {
      DrawOutlinerContent(state, ui_state);
      ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem("Diagnostics")) {
      DrawDiagnosticsContent(state, ui_state);
      ImGui::EndTabItem();
    }
    ImGui::EndTabBar();
  }
  ImGui::End();
}

void DrawStatsPanel(CoreState& state, ViewerUiState& ui_state) {
  DrawTopbarWindow(state, ui_state);
  if (ui_state.ui_unified_workspace) {
    DrawUnifiedWorkspaceWindow(state, ui_state);
  } else {
    DrawToolboxWindow(state, ui_state);
    DrawInspectorWindow(state, ui_state);
    DrawOutlinerWindow(state, ui_state);
    DrawDiagnosticsWindow(state, ui_state);
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
  camera.fovy = 45.0f;
  camera.projection = CAMERA_PERSPECTIVE;

  CoreState state = wire::core::make_demo_state();
  ViewerUiState ui_state;
  ui_state.ui_unified_workspace = persisted.ui_unified_workspace;
  ui_state.ui_show_workspace = persisted.ui_show_workspace;
  ui_state.ui_workspace_width = persisted.ui_workspace_width;
  PushLog(ui_state, "[info] viewer started");
  PushLog(ui_state, "[info] demo state loaded");
  PushLog(ui_state, "[mode] Placement/Connection/Branch/Detail/DrawPath");
  PushLog(ui_state, "[flow] Main path: Pole->Pole connection");
  PushLog(ui_state, "[hint] Blender style controls enabled");
  PushLog(ui_state, "[hint] MMB orbit, Shift+MMB pan, Ctrl+MMB dolly");
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
  {
    ViewerPersistentSettings out{};
    out.window_width = GetScreenWidth();
    out.window_height = GetScreenHeight();
    out.ui_unified_workspace = ui_state.ui_unified_workspace;
    out.ui_show_workspace = ui_state.ui_show_workspace;
    out.ui_workspace_width = ui_state.ui_workspace_width;
    SaveViewerPersistentSettings(out);
  }
  CloseWindow();
  return 0;
}
