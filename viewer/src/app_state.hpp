#pragma once

#include <algorithm>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <unordered_map>
#include <vector>

#include "raylib.h"
#include "wire/core/core_state_api_types.hpp"
#include "wire/core/core_view.hpp"

using wire::core::CoreState;
using wire::core::ObjectId;

enum class SelectedType {
  kNone = 0,
  kPole = 1,
  kPort = 2,
  kSpan = 3,
  kAnchor = 4,
  kBundle = 5,
  kAttachment = 6,
  kSupportNode = 7,
  kSpanLayout = 8,
  kDetailCurve = 9,
  kJunction = 10,
};

enum class CameraDragMode {
  kNone = 0,
  kOrbit = 1,
  kPan = 2,
  kDolly = 3,
};

enum class EditMode {
  kDrawPath = 0,
};

struct SelectionItem {
  SelectedType type = SelectedType::kNone;
  ObjectId id = wire::core::kInvalidObjectId;
};

struct DragSelectionState {
  bool active = false;
  Vector2 start_screen{0.0f, 0.0f};
  Vector2 end_screen{0.0f, 0.0f};
};

struct ViewerUiState {
  EditMode mode = EditMode::kDrawPath;
  SelectedType selected_type = SelectedType::kNone;
  ObjectId selected_id = wire::core::kInvalidObjectId;
  std::vector<SelectionItem> selection_items{};
  bool selection_include_poles = true;
  bool selection_include_midair_nodes = true;
  bool selection_include_spans = true;
  DragSelectionState drag_selection{};

  bool show_debug_labels = false;
  bool show_whole_aabb = false;
  bool show_segment_aabb = false;
  bool show_selected_bundle_highlight = true;
  bool geometry_settings_loaded = false;
  int geometry_samples = 8;
  bool geometry_sag_enabled = false;
  double geometry_sag_factor = 0.03;
  double geometry_pole_clearance = 0.05;
  bool visual_settings_loaded = false;
  bool visual_enable_support_structures = true;
  bool visual_enable_insulators = true;
  bool viewer_enable_solid_support_render = true;
  double visual_support_center_threshold = 0.03;
  double visual_support_arm_extra = 0.20;
  double visual_insulator_radius = 0.07;
  double visual_insulator_length = 0.16;
  bool cable_template_loaded = false;
  wire::core::CableTemplateId selected_cable_template_id = static_cast<wire::core::CableTemplateId>(1);
  std::string cable_template_name{"HV_BARE"};
  double cable_outer_diameter = 0.048;
  double cable_bend_stiffness = 2.8;
  double cable_min_bend_radius = 0.7;
  int cable_material_style = static_cast<int>(wire::core::CableMaterialStyleKind::kBareConductor);
  bool cable_requires_insulator = true;
  double cable_insulator_attachment_height = 0.145;
  double cable_sag_factor = 0.045;
  double cable_slack_factor = 0.025;
  double cable_default_grouped_support_fanout_spacing = 0.35;
  int cable_continuity_policy = static_cast<int>(wire::core::CableContinuityPolicyHint::kPreferG1);
  bool cable_curve_offset_straight_supplemental_enabled = true;
  double cable_curve_offset_straight_lateral_offset = 0.0;
  double cable_curve_offset_straight_vertical_offset = 0.0;
  double cable_curve_offset_straight_wobble_amplitude = 0.0;
  double cable_curve_offset_straight_wobble_wavelength = 0.0;
  double cable_curve_offset_straight_wobble_phase_bias = 0.0;
  double cable_curve_offset_straight_endpoint_envelope_ratio = 0.0;
  bool bundle_template_loaded = false;
  wire::core::BundleKind selected_bundle_template_id = wire::core::BundleKind::kHighVoltage;
  wire::core::CableTemplateId bundle_template_cable_template_id = wire::core::kInvalidCableTemplateId;
  int bundle_template_default_layer = static_cast<int>(wire::core::SpanLayer::kLowVoltage);
  bool bundle_template_allow_mirror = true;
  bool bundle_template_allow_midair_node = true;
  bool bundle_template_allow_midair_branch = true;
  int bundle_template_support_style = static_cast<int>(wire::core::BundleSupportStyleHint::kAuto);
  int bundle_template_branch_policy = static_cast<int>(wire::core::BundleBranchPolicyHint::kAuto);
  int bundle_template_continuity_policy = static_cast<int>(wire::core::CableContinuityPolicyHint::kAuto);
  double bundle_template_grouped_support_fanout_spacing = 0.2;
  bool pole_template_loaded = false;
  wire::core::PoleTypeId selected_pole_template_id = wire::core::kInvalidPoleTypeId;
  wire::core::PoleTypeDefinition pole_template_edit{};
  double tilt_all_max_deg = 9.5;
  std::uint64_t road_id = 1;
  std::uint32_t draw_category_mask = (1u << static_cast<int>(wire::core::ConnectionCategory::kLowVoltage));
  std::uint32_t draw_bundle_template_mask =
      (1u << static_cast<int>(wire::core::BundleKind::kLowVoltage)) |
      (1u << static_cast<int>(wire::core::BundleKind::kHighVoltage)) |
      (1u << static_cast<int>(wire::core::BundleKind::kCommunication)) |
      (1u << static_cast<int>(wire::core::BundleKind::kOptical));
  std::unordered_map<int, int> draw_bundle_count_by_template{};
  int road_pole_type_index = 1;
  int last_generated_poles = 0;
  int last_generated_spans = 0;
  std::vector<ObjectId> last_generated_pole_ids{};
  std::vector<ObjectId> last_generated_span_ids{};
  std::optional<wire::core::BackboneSpec> last_draw_path_request{};
  std::vector<wire::core::Vec3d> draw_path_points{};
  std::vector<wire::core::SupportKind> draw_path_point_support_kinds{};
  std::vector<ObjectId> draw_path_point_node_ids{};
  bool draw_pick_enabled = true;
  double draw_snap_radius_world = 0.75;
  bool draw_show_backbone_overlay = true;
  wire::core::PickResult draw_hover_pick{};
  bool draw_hover_has_resolution = false;
  wire::core::ResolveBranchPickResult draw_hover_resolution{};
  std::string draw_hover_status{};
  bool draw_hover_valid = false;
  wire::core::Vec3d draw_hover_point{};
  double draw_plane_z = 0.0;
  bool draw_show_preview = true;
  bool draw_keep_path_after_generate = true;
  bool draw_clicked_points_only = true;
  double draw_interval_m = 8.0;
  int draw_direction_mode = static_cast<int>(wire::core::PathDirectionMode::kAuto);
  bool layout_settings_loaded = false;
  bool layout_angle_correction_enabled = true;
  double layout_corner_threshold_deg = wire::core::kDefaultCornerThresholdDeg;
  double layout_min_side_scale = 1.0;
  double layout_max_side_scale = 1.8;
  std::string last_repro_capture_path{};
  std::vector<ObjectId> preferred_visible_span_ids{};
  int preferred_visible_span_count = 0;

  std::string last_error;
  std::vector<std::string> logs;
  CameraDragMode camera_drag_mode = CameraDragMode::kNone;
  bool camera_walk_mode = false;
  float camera_walk_speed = 6.0f;
  float camera_mouse_sensitivity = 0.003f;
  float camera_fov_deg = 45.0f;
  bool camera_consumed_escape = false;
  bool ui_unified_workspace = true;
  bool ui_show_workspace = true;
  float ui_workspace_width = 0.0f;
  bool pole_height_view_show_ports = true;
  bool pole_height_view_show_supports = true;
  bool pole_height_view_show_bundles = true;
};

inline bool IsValidSelectionItem(const SelectionItem& item) {
  return item.type != SelectedType::kNone && item.id != wire::core::kInvalidObjectId;
}

inline void NormalizeSelection(ViewerUiState& ui_state) {
  ui_state.selection_items.erase(
      std::remove_if(ui_state.selection_items.begin(), ui_state.selection_items.end(),
                     [](const SelectionItem& item) { return !IsValidSelectionItem(item); }),
      ui_state.selection_items.end());
  std::sort(ui_state.selection_items.begin(), ui_state.selection_items.end(),
            [](const SelectionItem& a, const SelectionItem& b) {
              if (a.type != b.type) {
                return static_cast<int>(a.type) < static_cast<int>(b.type);
              }
              return a.id < b.id;
            });
  ui_state.selection_items.erase(std::unique(ui_state.selection_items.begin(), ui_state.selection_items.end(),
                                             [](const SelectionItem& a, const SelectionItem& b) {
                                               return a.type == b.type && a.id == b.id;
                                             }),
                                 ui_state.selection_items.end());

  if (ui_state.selection_items.empty()) {
    ui_state.selected_type = SelectedType::kNone;
    ui_state.selected_id = wire::core::kInvalidObjectId;
    return;
  }
  ui_state.selected_type = ui_state.selection_items.front().type;
  ui_state.selected_id = ui_state.selection_items.front().id;
}

inline void ReplaceSelection(ViewerUiState& ui_state, std::vector<SelectionItem> items) {
  ui_state.selection_items = std::move(items);
  NormalizeSelection(ui_state);
}

inline void ClearSelection(ViewerUiState& ui_state) {
  ui_state.selection_items.clear();
  NormalizeSelection(ui_state);
}

inline void SetPrimarySelection(ViewerUiState& ui_state, SelectedType type, ObjectId id) {
  ReplaceSelection(ui_state, {{type, id}});
}

inline bool SelectionContains(const ViewerUiState& ui_state, SelectedType type, ObjectId id) {
  return std::find_if(ui_state.selection_items.begin(), ui_state.selection_items.end(),
                      [type, id](const SelectionItem& item) { return item.type == type && item.id == id; }) !=
         ui_state.selection_items.end();
}

inline int SelectionCountByType(const ViewerUiState& ui_state, SelectedType type) {
  return static_cast<int>(std::count_if(ui_state.selection_items.begin(), ui_state.selection_items.end(),
                                        [type](const SelectionItem& item) { return item.type == type; }));
}

struct ViewerPersistentSettings {
  int window_width = 1280;
  int window_height = 720;
  bool ui_unified_workspace = true;
  bool ui_show_workspace = true;
  float ui_workspace_width = 420.0f;
  float camera_fov_deg = 45.0f;
  float camera_walk_speed = 6.0f;
};

void EnsureDrawPathPointKinds(ViewerUiState& ui_state);
void DrawPathPushPoint(ViewerUiState& ui_state, const wire::core::Vec3d& point, wire::core::SupportKind support_kind,
                       ObjectId node_id = wire::core::kInvalidObjectId);
void DrawPathPopPoint(ViewerUiState& ui_state);
void DrawPathClearPoints(ViewerUiState& ui_state);
bool ExecuteBackboneRequest(CoreState& state, ViewerUiState& ui_state, const wire::core::BackboneSpec& request,
                            bool clear_draw_path_on_success, const char* success_log,
                            const char* failure_log);
void ExecuteGenerateFromDrawPath(CoreState& state, ViewerUiState& ui_state, bool from_enter_key);
bool SaveDrawPathReproCapture(const CoreState& state, const ViewerUiState& ui_state, std::string* out_path,
                              std::string* out_error);
void UpdateDrawPathInput(CoreState& state, const Camera3D& camera, ViewerUiState& ui_state);
void DrawPathPreview(const ViewerUiState& ui_state);
void DrawPathModePanel(CoreState& state, ViewerUiState& ui_state);
