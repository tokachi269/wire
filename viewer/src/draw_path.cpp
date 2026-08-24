#include "app_state.hpp"
#include "core_state_adapter.hpp"
#include "path_pick_policy.hpp"
#include "ui_common.hpp"

#include "city/wire/coord_utils.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <sstream>
#include <type_traits>
#include <utility>

#include "imgui.h"
#include "scene_query.hpp"

namespace {

template <typename Enum>
constexpr auto EnumOrdinal(Enum value) noexcept {
  return static_cast<std::underlying_type_t<Enum>>(value);
}

const char* CategoryLabelLocal(city::wire::ConnectionCategory category) {
  switch (category) {
  case city::wire::ConnectionCategory::kLowVoltage:
    return "LowVoltage";
  case city::wire::ConnectionCategory::kHighVoltage:
    return "HighVoltage";
  case city::wire::ConnectionCategory::kCommunication:
    return "Communication";
  case city::wire::ConnectionCategory::kOptical:
    return "Optical";
  case city::wire::ConnectionCategory::kDrop:
    return "Drop";
  default:
    return "Unknown";
  }
}

const char* PathDirectionModeLabelLocal(city::wire::PathDirectionMode mode) {
  switch (mode) {
  case city::wire::PathDirectionMode::kAuto:
    return "Auto";
  case city::wire::PathDirectionMode::kForward:
    return "Forward";
  case city::wire::PathDirectionMode::kReverse:
    return "Reverse";
  default:
    return "Unknown";
  }
}

const char* PathDirectionChosenLabelLocal(city::wire::PathDirectionChosen chosen) {
  switch (chosen) {
  case city::wire::PathDirectionChosen::kForward:
    return "Forward";
  case city::wire::PathDirectionChosen::kReverse:
    return "Reverse";
  default:
    return "Unknown";
  }
}

const char* SupportKindLabelLocal(city::wire::SupportKind kind) {
  switch (kind) {
  case city::wire::SupportKind::kPole:
    return "Pole";
  case city::wire::SupportKind::kMidair:
    return "Midair";
  case city::wire::SupportKind::kExternal:
    return "External";
  default:
    return "Unknown";
  }
}

const char* PickHitKindLabelLocal(city::wire::PickHitKind kind) {
  switch (kind) {
  case city::wire::PickHitKind::kNode:
    return "Node";
  case city::wire::PickHitKind::kSegment:
    return "Segment";
  case city::wire::PickHitKind::kGround:
    return "Ground";
  case city::wire::PickHitKind::kExternal:
    return "External";
  default:
    return "Empty";
  }
}

std::string BundleTemplatePreviewOrIdLocal(const city::wire::CoreView& view, city::wire::BundleKind kind) {
  const auto it = view.bundle_templates().find(kind);
  if (it == view.bundle_templates().end()) {
    return std::to_string(static_cast<int>(kind));
  }
  return it->second.name;
}

std::string GeneratedEndpointSourceSummaryLocal(const city::wire::CoreView& view, const std::vector<ObjectId>& span_ids) {
  int plain = 0;
  int socket = 0;
  int socket_override = 0;
  int fallback = 0;
  int attachment_inputs = 0;
  for (ObjectId span_id : span_ids) {
    const auto layout_view = view.span_layout(span_id);
    if (!layout_view.has_layout()) {
      continue;
    }
    const auto accumulate = [&](const city::wire::LayoutEndpoint& endpoint) {
      if (endpoint.attachment_request.kind != city::wire::EndpointAttachmentRequestKind::kNone) {
        ++attachment_inputs;
      }
      if (endpoint.endpoint_source == city::wire::LayoutEndpointSourceKind::kPlainSupport) {
        ++plain;
      } else if (endpoint.endpoint_source == city::wire::LayoutEndpointSourceKind::kAttachmentSocket) {
        ++socket;
      } else if (endpoint.endpoint_source == city::wire::LayoutEndpointSourceKind::kAttachmentSocketOverride) {
        ++socket_override;
      } else {
        ++fallback;
      }
    };
    accumulate(layout_view.entry->start);
    accumulate(layout_view.entry->end);
  }
  std::ostringstream oss;
  oss << "endpointSources plain=" << plain << " socket=" << socket << " override=" << socket_override
      << " fallback=" << fallback << " attachmentInput=" << attachment_inputs;
  return oss.str();
}

int ResolveBundleTemplateCountLocal(ViewerUiState& ui_state, const city::wire::BundleTemplate& bundle_template,
                                    city::wire::BundleKind kind) {
  return ResolveBundleTemplateCount(ui_state, bundle_template, kind);
}

std::string PoleTypePreviewLocal(const city::wire::CoreView& view, city::wire::PoleTypeId pole_type_id) {
  const auto it = view.pole_types().find(pole_type_id);
  if (it == view.pole_types().end()) {
    return std::to_string(pole_type_id);
  }
  return it->second.name;
}

std::size_t ClampedTypeIndexLocal(int current, std::size_t count) {
  if (count == 0) {
    return 0;
  }
  const int max_index = static_cast<int>(count - 1);
  return static_cast<std::size_t>(std::clamp(current, 0, max_index));
}

Vector3 ToRaylibLocal(const city::wire::Vec3d& ue_xyz) {
  return Vector3{static_cast<float>(ue_xyz.x), static_cast<float>(ue_xyz.z), static_cast<float>(ue_xyz.y)};
}

double PolylineLengthLocal(const std::vector<city::wire::Vec3d>& points) {
  return city::wire::PolylineLength(points);
}

void PushLogLocal(ViewerUiState& ui_state, const std::string& line) {
  ui_state.logs.push_back(line);
  if (ui_state.logs.size() > 256) {
    ui_state.logs.erase(ui_state.logs.begin());
  }
}

} // namespace

void EnsureDrawPathPointKinds(ViewerUiState& ui_state) {
  if (ui_state.draw_path_point_support_kinds.size() < ui_state.draw_path_points.size()) {
    ui_state.draw_path_point_support_kinds.resize(ui_state.draw_path_points.size(), city::wire::SupportKind::kPole);
  } else if (ui_state.draw_path_point_support_kinds.size() > ui_state.draw_path_points.size()) {
    ui_state.draw_path_point_support_kinds.resize(ui_state.draw_path_points.size());
  }
  if (ui_state.draw_path_point_node_ids.size() < ui_state.draw_path_points.size()) {
    ui_state.draw_path_point_node_ids.resize(ui_state.draw_path_points.size(), city::wire::kInvalidObjectId);
  } else if (ui_state.draw_path_point_node_ids.size() > ui_state.draw_path_points.size()) {
    ui_state.draw_path_point_node_ids.resize(ui_state.draw_path_points.size());
  }
}

void DrawPathPushPoint(ViewerUiState& ui_state, const city::wire::Vec3d& point, city::wire::SupportKind support_kind,
                       ObjectId node_id) {
  ui_state.draw_path_points.push_back(point);
  ui_state.draw_path_point_support_kinds.push_back(support_kind);
  ui_state.draw_path_point_node_ids.push_back(node_id);
}

void DrawPathPopPoint(ViewerUiState& ui_state) {
  if (ui_state.draw_path_points.empty()) {
    return;
  }
  ui_state.draw_path_points.pop_back();
  if (!ui_state.draw_path_point_support_kinds.empty()) {
    ui_state.draw_path_point_support_kinds.pop_back();
  }
  if (!ui_state.draw_path_point_node_ids.empty()) {
    ui_state.draw_path_point_node_ids.pop_back();
  }
}

void DrawPathClearPoints(ViewerUiState& ui_state) {
  ui_state.draw_path_points.clear();
  ui_state.draw_path_point_support_kinds.clear();
  ui_state.draw_path_point_node_ids.clear();
}

city::wire::ResolveBranchPickResult DirectResolvedDrawPathTarget(const city::wire::PickResult& pick) {
  city::wire::ResolveBranchPickResult resolved{};
  resolved.resolution = city::wire::PickBranchResolutionKind::kNode;
  resolved.resolved_node_id = pick.hit_id;
  resolved.position = pick.hit_pos_world;
  resolved.support_kind = (pick.hit_kind == city::wire::PickHitKind::kExternal) ? city::wire::SupportKind::kExternal
                                                                                 : city::wire::SupportKind::kPole;
  resolved.snapped_from_segment_endpoint = false;
  return resolved;
}

bool ExecuteBackboneRequest(CoreState& state, ViewerUiState& ui_state, const city::wire::BackboneSpec& request,
                            bool clear_draw_path_on_success, const char* success_log,
                            const char* failure_log) {
  const auto view = viewer_core_state::View(state);
  ui_state.last_draw_path_request = request;
  const auto result = viewer_core_state::GenerateFromBackboneSpec(state, request);
  if (!result.ok) {
    ui_state.last_error = result.error;
    PushLogLocal(ui_state, failure_log);
    return false;
  }

  ui_state.last_error.clear();
  ui_state.last_generated_poles = static_cast<int>(result.value.generated_pole_ids.size());
  ui_state.last_generated_spans = static_cast<int>(result.value.generated_span_ids.size());
  ui_state.last_generated_pole_ids = result.value.generated_pole_ids;
  ui_state.last_generated_span_ids = result.value.generated_span_ids;
  if (!result.value.generated_pole_ids.empty()) {
    SetPrimarySelection(ui_state, SelectedType::kPole, result.value.generated_pole_ids.back());
  } else if (!result.value.generated_span_ids.empty()) {
    SetPrimarySelection(ui_state, SelectedType::kSpan, result.value.generated_span_ids.back());
  }

  if (clear_draw_path_on_success) {
    DrawPathClearPoints(ui_state);
  }
  PushLogLocal(ui_state, std::string(success_log) + " poles=" + std::to_string(ui_state.last_generated_poles) +
                             " spans=" + std::to_string(ui_state.last_generated_spans));
  PushLogLocal(ui_state, "DrawPath attachment/socket request input: unsupported in BackboneSpec node/path request");
  PushLogLocal(ui_state, GeneratedEndpointSourceSummaryLocal(view, result.value.generated_span_ids));
  return true;
}

void ExecuteGenerateFromDrawPath(CoreState& state, ViewerUiState& ui_state, bool from_enter_key) {
  const auto view = viewer_core_state::View(state);
  EnsureDrawPathPointKinds(ui_state);
  if (ui_state.draw_path_points.size() < 2) {
    ui_state.last_error = "path needs at least 2 points";
    return;
  }

  const auto type_ids = SortedPoleTypeIds(view);
  if (type_ids.empty()) {
    ui_state.last_error = "no pole type available";
    return;
  }

  const std::size_t road_type_index = ClampedTypeIndexLocal(ui_state.road_pole_type_index, type_ids.size());
  ui_state.road_pole_type_index = static_cast<int>(road_type_index);

  const auto selected_templates = SelectedBundleTemplates(view, ui_state);
  if (selected_templates.empty()) {
    ui_state.last_error = "select at least one bundle template";
    return;
  }

  const int mode_index = std::clamp(ui_state.draw_direction_mode, 0, 2);
  ui_state.draw_direction_mode = mode_index;

  city::wire::BackboneSpec request{};
  request.path.polyline = ui_state.draw_path_points;
  for (std::size_t i = 0; i < ui_state.draw_path_points.size(); ++i) {
    const city::wire::SupportKind support_kind = ui_state.draw_path_point_support_kinds[i];
    const ObjectId node_id =
        (i < ui_state.draw_path_point_node_ids.size()) ? ui_state.draw_path_point_node_ids[i] : city::wire::kInvalidObjectId;
    if (support_kind == city::wire::SupportKind::kPole && node_id == city::wire::kInvalidObjectId) {
      continue;
    }
    city::wire::BackboneInputSpec::NodeSpec node_spec{};
    node_spec.point_index = i;
    node_spec.support_kind = support_kind;
    node_spec.node_id = node_id;
    request.path.node_specs.push_back(node_spec);
  }
  if (ui_state.draw_clicked_points_only) {
    request.interval_m = std::max(0.001, PolylineLengthLocal(request.path.polyline) + 1.0);
  } else {
    request.interval_m = ui_state.draw_interval_m;
  }
  request.pole_type_id = type_ids[ui_state.road_pole_type_index];
  request.pole_placement.enable_tilt = ui_state.tilt_all_max_deg > 0.0;
  request.pole_placement.max_tilt_deg = ui_state.tilt_all_max_deg;
  request.direction_mode = static_cast<city::wire::PathDirectionMode>(mode_index);
  for (city::wire::BundleKind kind : selected_templates) {
    const auto it = view.bundle_templates().find(kind);
    if (it == view.bundle_templates().end()) {
      ui_state.last_error = "selected bundle template is not available";
      return;
    }
    const city::wire::BundleTemplate& bundle_template = it->second;
    city::wire::BackboneBundleSpec bundle_request{};
    bundle_request.bundle_template_id = kind;
    bundle_request.layer = city::wire::SpanLayer::kUnknown;
    bundle_request.count = 0;
    if (bundle_template.count_rule == city::wire::BundleCountRuleKind::kRange) {
      bundle_request.count = ResolveBundleTemplateCountLocal(ui_state, bundle_template, kind);
    }
    request.bundles.push_back(bundle_request);
  }

  const std::string success_log =
      std::string(from_enter_key ? "Generated path (Enter)" : "Generated path") + " templates=" +
      BundleTemplateMultiPreview(view, ui_state);
  const char* failure_log = from_enter_key ? "Generate path (Enter) failed" : "Generate path failed";
  (void)ExecuteBackboneRequest(state, ui_state, request, !ui_state.draw_keep_path_after_generate, success_log.c_str(),
                               failure_log);
}


void UpdateDrawPathInput(CoreState& state, const Camera3D& camera, ViewerUiState& ui_state) {
  const auto view = viewer_core_state::View(state);
  ui_state.draw_hover_pick = {};
  ui_state.draw_hover_has_resolution = false;
  ui_state.draw_hover_status.clear();
  if (ui_state.mode != EditMode::kDrawPath) {
    ui_state.draw_hover_valid = false;
    return;
  }
  EnsureDrawPathPointKinds(ui_state);

  ImGuiIO& io = ImGui::GetIO();
  city::wire::Vec3d hover{};
  const bool has_ground_hit = TryPickGroundPoint(camera, ui_state.draw_plane_z, &hover);
  ui_state.draw_hover_valid = has_ground_hit;
  if (has_ground_hit) {
    ui_state.draw_hover_point = hover;
  }
  const bool accept_mouse_input =
      !io.WantCaptureMouse && ui_state.camera_drag_mode == CameraDragMode::kNone && !ui_state.camera_walk_mode;

  if (ui_state.draw_pick_enabled && accept_mouse_input) {
    ViewerSceneQuery scene_query{};
    const city::wire::PickResult raw_pick = scene_query.Raycast(view, camera, ui_state.draw_plane_z);
    const double hover_snap_radius_world = std::max(ui_state.draw_snap_radius_world, 1.25);
    city::wire::PickResult pick =
        CanonicalizeDrawPathPick(view, raw_pick, hover, has_ground_hit, hover_snap_radius_world);
    ui_state.draw_hover_pick = pick;
    bool blocked_pick_target = false;
    if (pick.hit_kind == city::wire::PickHitKind::kEmpty) {
      if (ui_state.draw_hover_status.empty()) {
        ui_state.draw_hover_status = "target: Empty";
      }
    } else {
      if (pick.hit_kind == city::wire::PickHitKind::kNode && raw_pick.hit_kind != city::wire::PickHitKind::kNode) {
        ui_state.draw_hover_status = "target: " + std::string(PickHitKindLabelLocal(raw_pick.hit_kind)) +
                                     " -> snapped: Node " +
                                     std::to_string(static_cast<unsigned long long>(pick.hit_id));
      }
      const std::vector<city::wire::BundleKind> pick_template_ids =
          ResolveTemplateKindsForPathPick(view, ui_state.draw_bundle_template_mask, pick);
      if (ui_state.draw_hover_status.empty()) {
        ui_state.draw_hover_status =
            std::string("target: ") + PickHitKindLabelLocal(pick.hit_kind) + " " + PickTargetLabel(pick);
      }
      if (pick.hit_kind == city::wire::PickHitKind::kNode || pick.hit_kind == city::wire::PickHitKind::kSegment ||
          pick.hit_kind == city::wire::PickHitKind::kExternal) {
        city::wire::EditResult<city::wire::ResolveBranchPickResult> resolved{};
        if (pick.hit_kind == city::wire::PickHitKind::kNode || pick.hit_kind == city::wire::PickHitKind::kExternal) {
          resolved.ok = true;
          resolved.value = DirectResolvedDrawPathTarget(pick);
        } else {
          city::wire::ResolveBranchPickOptions options{};
          options.selected_bundle_template_ids = pick_template_ids;
          options.snap_radius_world = ui_state.draw_snap_radius_world;
          options.create_midair_node = false;
          options.enforce_midair_template_policy = true;
          resolved = viewer_core_state::ResolveBranchPick(state, pick, options);
        }
        if (resolved.ok) {
          const std::string blocked_template = FindMidairBranchBlockedTemplateName(view, pick_template_ids);
          if (resolved.value.resolution == city::wire::PickBranchResolutionKind::kMidair &&
              !blocked_template.empty()) {
            ui_state.draw_hover_status += " -> warn: template " + blocked_template + " will not connect here";
          }
          ui_state.draw_hover_has_resolution = true;
          ui_state.draw_hover_resolution = resolved.value;
          ui_state.draw_hover_point = resolved.value.position;
          ui_state.draw_hover_valid = true;
          ui_state.draw_hover_status +=
              (resolved.value.resolution == city::wire::PickBranchResolutionKind::kNode)
                  ? " | resolved: Node"
                  : " | resolved: Midair";
          if (resolved.value.resolved_node_id != city::wire::kInvalidObjectId) {
            ui_state.draw_hover_status +=
                " " + std::to_string(static_cast<unsigned long long>(resolved.value.resolved_node_id));
          }
        } else {
          ui_state.draw_hover_status += " | blocked: " + resolved.error;
          blocked_pick_target = true;
        }
      }
    }
    if (blocked_pick_target) {
      ui_state.draw_hover_valid = false;
    }
  }

  if (accept_mouse_input) {
    if (ui_state.draw_hover_valid && IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
      if (ui_state.draw_hover_has_resolution) {
        city::wire::EditResult<city::wire::ResolveBranchPickResult> applied{};
        if (ui_state.draw_hover_pick.hit_kind == city::wire::PickHitKind::kNode ||
            ui_state.draw_hover_pick.hit_kind == city::wire::PickHitKind::kExternal ||
            ui_state.draw_hover_resolution.resolution == city::wire::PickBranchResolutionKind::kNode) {
          applied.ok = true;
          applied.value = ui_state.draw_hover_resolution;
        } else {
          const std::vector<city::wire::BundleKind> click_template_ids =
              ResolveTemplateKindsForPathPick(view, ui_state.draw_bundle_template_mask, ui_state.draw_hover_pick);
          city::wire::ResolveBranchPickOptions click_options{};
          click_options.selected_bundle_template_ids = click_template_ids;
          click_options.snap_radius_world = ui_state.draw_snap_radius_world;
          click_options.create_midair_node = true;
          click_options.enforce_midair_template_policy = true;
          applied = viewer_core_state::ResolveBranchPick(state, ui_state.draw_hover_pick, click_options);
        }
        if (!applied.ok) {
          ui_state.last_error = applied.error;
          PushLogLocal(ui_state, "DrawPath ResolveBranchPick(click) failed: " + applied.error);
        } else {
          ui_state.last_error.clear();
          DrawPathPushPoint(ui_state, applied.value.position, applied.value.support_kind, applied.value.resolved_node_id);
        }
      } else {
        DrawPathPushPoint(ui_state, ui_state.draw_hover_point, city::wire::SupportKind::kPole,
                          city::wire::kInvalidObjectId);
      }
    }
    if (IsMouseButtonPressed(MOUSE_RIGHT_BUTTON)) {
      DrawPathPopPoint(ui_state);
    }
  }

  if (!io.WantCaptureKeyboard && !ui_state.camera_walk_mode && !ui_state.camera_consumed_escape) {
    if (IsKeyPressed(KEY_BACKSPACE)) {
      DrawPathPopPoint(ui_state);
    }
    if (IsKeyPressed(KEY_ESCAPE)) {
      DrawPathClearPoints(ui_state);
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
    const city::wire::SupportKind support_kind =
        (i < ui_state.draw_path_point_support_kinds.size()) ? ui_state.draw_path_point_support_kinds[i]
                                                            : city::wire::SupportKind::kPole;
    const ObjectId node_id =
        (i < ui_state.draw_path_point_node_ids.size()) ? ui_state.draw_path_point_node_ids[i] : city::wire::kInvalidObjectId;
    Color point_color = Color{196, 156, 68, 255};
    if (support_kind == city::wire::SupportKind::kMidair) {
      point_color = Color{90, 154, 176, 255};
    } else if (support_kind == city::wire::SupportKind::kExternal) {
      point_color = Color{110, 154, 100, 255};
    }
    DrawSphere(ToRaylibLocal(ui_state.draw_path_points[i]), 0.12f, point_color);
    if (node_id != city::wire::kInvalidObjectId) {
      DrawSphereWires(ToRaylibLocal(ui_state.draw_path_points[i]), 0.18f, 10, 16, Color{128, 126, 120, 200});
    }
    if (i + 1 < ui_state.draw_path_points.size()) {
      DrawLine3D(ToRaylibLocal(ui_state.draw_path_points[i]), ToRaylibLocal(ui_state.draw_path_points[i + 1]),
                 Color{152, 126, 72, 255});
    }
  }
  if (ui_state.draw_hover_valid) {
    DrawSphere(ToRaylibLocal(ui_state.draw_hover_point), 0.08f, Color{96, 152, 132, 200});
    if (!ui_state.draw_path_points.empty()) {
      DrawLine3D(ToRaylibLocal(ui_state.draw_path_points.back()), ToRaylibLocal(ui_state.draw_hover_point),
                 Color{96, 152, 132, 180});
    }
  }
}
void DrawPathModePanel(CoreState& state, ViewerUiState& ui_state) {
  const auto view = viewer_core_state::View(state);
  EnsureDrawPathPointKinds(ui_state);
  ImGui::TextUnformatted("Draw Path");
  ImGui::TextUnformatted("LMB: add point on Backbone node/edge (with draw-plane fallback)");
  ImGui::TextUnformatted("RMB/Backspace: undo last");
  ImGui::TextUnformatted("Esc: clear path, Enter: generate");
  ImGui::Checkbox("Enable DrawPath Pick (LMB)", &ui_state.draw_pick_enabled);
  ImGui::Checkbox("Show Backbone Overlay", &ui_state.draw_show_backbone_overlay);
  ImGui::InputDouble("Draw Snap Radius (world)", &ui_state.draw_snap_radius_world, 0.05, 0.2, "%.2f");
  ui_state.draw_snap_radius_world = std::clamp(ui_state.draw_snap_radius_world, 0.05, 5.0);
  ImGui::InputDouble("Draw Plane Z", &ui_state.draw_plane_z, 0.1, 1.0, "%.2f");
  ImGui::InputDouble("Path Interval (m)", &ui_state.draw_interval_m, 0.5, 1.0, "%.2f");
  ui_state.draw_interval_m = std::max(0.0, ui_state.draw_interval_m);
  ImGui::Checkbox("Clicked Points Only (No Intermediate Pole)", &ui_state.draw_clicked_points_only);
  ImGui::Checkbox("Show Preview", &ui_state.draw_show_preview);
  ImGui::Checkbox("Keep Path After Generate", &ui_state.draw_keep_path_after_generate);
  ImGui::TextUnformatted("Template-driven bundle generation");
  ImGui::Text("Path points: %d", static_cast<int>(ui_state.draw_path_points.size()));
  const int midair_points = static_cast<int>(
      std::count(ui_state.draw_path_point_support_kinds.begin(), ui_state.draw_path_point_support_kinds.end(),
                 city::wire::SupportKind::kMidair));
  const int building_points = static_cast<int>(
      std::count(ui_state.draw_path_point_support_kinds.begin(), ui_state.draw_path_point_support_kinds.end(),
                 city::wire::SupportKind::kExternal));
  const int anchored_points = static_cast<int>(
      std::count_if(ui_state.draw_path_point_node_ids.begin(), ui_state.draw_path_point_node_ids.end(),
                    [](ObjectId node_id) { return node_id != city::wire::kInvalidObjectId; }));
  ImGui::Text("Support kind points: Midair=%d External=%d", midair_points, building_points);
  ImGui::Text("Anchored Backbone points: %d", anchored_points);
  if (ui_state.draw_hover_valid) {
    ImGui::Text("Hover: %.2f %.2f %.2f", ui_state.draw_hover_point.x, ui_state.draw_hover_point.y,
                ui_state.draw_hover_point.z);
  } else {
    ImGui::TextUnformatted("Hover: (no ground hit)");
  }
  if (!ui_state.draw_hover_status.empty()) {
    ImGui::TextWrapped("%s", ui_state.draw_hover_status.c_str());
  }
  if (ui_state.draw_hover_has_resolution) {
    ImGui::Text("Hover resolved: %s kind=%s",
                (ui_state.draw_hover_resolution.resolution == city::wire::PickBranchResolutionKind::kNode)
                    ? "Node"
                    : "Midair",
                SupportKindLabelLocal(ui_state.draw_hover_resolution.support_kind));
  }

  const auto type_ids = SortedPoleTypeIds(view);
  if (!type_ids.empty()) {
    const std::size_t road_type_index = ClampedTypeIndexLocal(ui_state.road_pole_type_index, type_ids.size());
    ui_state.road_pole_type_index = static_cast<int>(road_type_index);
    const city::wire::PoleTypeId road_type_id = type_ids[road_type_index];
    const std::string road_type_name = PoleTypePreviewLocal(view, road_type_id);
    if (ImGui::BeginCombo("Path PoleType", road_type_name.c_str())) {
      for (std::size_t i = 0; i < type_ids.size(); ++i) {
        const std::string label = PoleTypePreviewLocal(view, type_ids[i]);
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

  const auto template_ids = SortedBundleTemplateKinds(view);
  if (template_ids.empty()) {
    ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.35f, 1.0f), "No bundle template registered in core");
  } else {
    if (ImGui::BeginCombo("Bundle Templates", BundleTemplateMultiPreview(view, ui_state).c_str())) {
      for (const city::wire::BundleKind kind : template_ids) {
        bool selected = IsBundleTemplateSelected(ui_state, kind);
        if (ImGui::Selectable(BundleTemplatePreviewOrIdLocal(view, kind).c_str(), selected,
                              ImGuiSelectableFlags_DontClosePopups)) {
          selected = !selected;
          SetBundleTemplateSelected(ui_state, kind, selected);
        }
        if (selected) {
          ImGui::SetItemDefaultFocus();
        }
      }
      ImGui::EndCombo();
    }
    const auto selected_templates = SelectedBundleTemplates(view, ui_state);
    if (selected_templates.empty()) {
      ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.35f, 1.0f), "Select at least one bundle template");
    }
    for (city::wire::BundleKind kind : selected_templates) {
      const auto it = view.bundle_templates().find(kind);
      if (it == view.bundle_templates().end()) {
        continue;
      }
      const city::wire::BundleTemplate& bundle_template = it->second;
      ImGui::Separator();
      ImGui::TextUnformatted(BundleTemplatePreviewOrIdLocal(view, kind).c_str());
      ImGui::Text("Category: %s", CategoryLabelLocal(bundle_template.category));
      if (bundle_template.count_rule == city::wire::BundleCountRuleKind::kRange) {
        const int key = static_cast<int>(kind);
        int count = ResolveBundleTemplateCountLocal(ui_state, bundle_template, kind);
        ImGui::PushID(key);
        ImGui::InputInt("Count", &count);
        ImGui::PopID();
        count = std::clamp(count, bundle_template.min_count, bundle_template.max_count);
        ui_state.draw_bundle_count_by_template[key] = count;
        ImGui::Text("Allowed Range: %d..%d (default=%d)", bundle_template.min_count, bundle_template.max_count,
                    bundle_template.default_count);
      } else {
        ImGui::Text("Fixed Count: %d (template rule)", bundle_template.fixed_count);
      }
      ImGui::Text("Template Layer: %d", static_cast<int>(bundle_template.default_layer));
      ImGui::Text("Allow Mirror: %s", bundle_template.allow_mirror ? "true" : "false");
    }
  }
  if (ImGui::BeginCombo("Direction Mode",
                        PathDirectionModeLabelLocal(
                            static_cast<city::wire::PathDirectionMode>(ui_state.draw_direction_mode)))) {
    for (int i = 0; i < 3; ++i) {
      const bool selected = (i == ui_state.draw_direction_mode);
      const auto mode = static_cast<city::wire::PathDirectionMode>(i);
      if (ImGui::Selectable(PathDirectionModeLabelLocal(mode), selected)) {
        ui_state.draw_direction_mode = i;
      }
      if (selected) {
        ImGui::SetItemDefaultFocus();
      }
    }
    ImGui::EndCombo();
  }
  if (!ui_state.draw_clicked_points_only && ui_state.draw_interval_m <= 0.0) {
    ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.35f, 1.0f), "Path Interval must be > 0");
  }
  if (ImGui::Button("Flip Direction (Manual)")) {
    if (ui_state.draw_direction_mode == static_cast<int>(city::wire::PathDirectionMode::kReverse)) {
      ui_state.draw_direction_mode = static_cast<int>(city::wire::PathDirectionMode::kForward);
    } else {
      ui_state.draw_direction_mode = static_cast<int>(city::wire::PathDirectionMode::kReverse);
    }
  }
  const auto& dir_debug = view.last_path_direction_debug();
  ImGui::Text("Direction chosen: %s (mode=%s)", PathDirectionChosenLabelLocal(dir_debug.chosen),
              PathDirectionModeLabelLocal(dir_debug.requested_mode));
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
    DrawPathPopPoint(ui_state);
  }
  ImGui::SameLine();
  if (ImGui::Button("Clear Path")) {
    DrawPathClearPoints(ui_state);
  }
}
