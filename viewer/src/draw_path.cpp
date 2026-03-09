#include "app_state.hpp"
#include "path_pick_policy.hpp"

#include <algorithm>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <sstream>

#include "imgui.h"
#include "scene_query.hpp"

namespace {

const char* CategoryLabelLocal(wire::core::ConnectionCategory category) {
  switch (category) {
  case wire::core::ConnectionCategory::kLowVoltage:
    return "LowVoltage";
  case wire::core::ConnectionCategory::kHighVoltage:
    return "HighVoltage";
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

const char* ContextLabelLocal(wire::core::ConnectionContext context) {
  switch (context) {
  case wire::core::ConnectionContext::kTrunkContinue:
    return "TrunkContinue";
  case wire::core::ConnectionContext::kCornerPass:
    return "CornerPass";
  case wire::core::ConnectionContext::kBranchAdd:
    return "BranchAdd";
  case wire::core::ConnectionContext::kDropAdd:
    return "DropAdd";
  default:
    return "Unknown";
  }
}

const char* PathDirectionModeLabelLocal(wire::core::PathDirectionMode mode) {
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

const char* PathDirectionChosenLabelLocal(wire::core::PathDirectionChosen chosen) {
  switch (chosen) {
  case wire::core::PathDirectionChosen::kForward:
    return "Forward";
  case wire::core::PathDirectionChosen::kReverse:
    return "Reverse";
  default:
    return "Unknown";
  }
}

const char* SupportKindLabelLocal(wire::core::SupportKind kind) {
  switch (kind) {
  case wire::core::SupportKind::kPole:
    return "Pole";
  case wire::core::SupportKind::kMidair:
    return "Midair";
  case wire::core::SupportKind::kBuilding:
    return "Building";
  default:
    return "Unknown";
  }
}

const char* PickHitKindLabelLocal(wire::core::PickHitKind kind) {
  switch (kind) {
  case wire::core::PickHitKind::kNode:
    return "Node";
  case wire::core::PickHitKind::kSegment:
    return "Segment";
  case wire::core::PickHitKind::kGround:
    return "Ground";
  case wire::core::PickHitKind::kBuilding:
    return "Building";
  default:
    return "Empty";
  }
}

const char* ModeLabelLocal(EditMode mode) {
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

const char* SelectedTypeLabelLocal(SelectedType selected_type) {
  switch (selected_type) {
  case SelectedType::kNone:
    return "None";
  case SelectedType::kPole:
    return "Pole";
  case SelectedType::kPort:
    return "Port";
  case SelectedType::kSpan:
    return "Span";
  case SelectedType::kAnchor:
    return "Anchor";
  case SelectedType::kBundle:
    return "Bundle";
  case SelectedType::kAttachment:
    return "Attachment";
  case SelectedType::kSupportNode:
    return "SupportNode";
  default:
    return "Unknown";
  }
}

std::vector<wire::core::BundleKind> SortedBundleTemplateKindsLocal(const CoreState& state) {
  std::vector<wire::core::BundleKind> ids;
  for (const auto& [kind, _] : state.view().bundle_templates()) {
    ids.push_back(kind);
  }
  std::sort(ids.begin(), ids.end(), [](auto a, auto b) { return static_cast<int>(a) < static_cast<int>(b); });
  return ids;
}

bool IsBundleTemplateSelectedLocal(const ViewerUiState& ui_state, wire::core::BundleKind kind) {
  const auto bit = (1u << static_cast<unsigned>(kind));
  return (ui_state.draw_bundle_template_mask & bit) != 0;
}

void SetBundleTemplateSelectedLocal(ViewerUiState& ui_state, wire::core::BundleKind kind, bool selected) {
  const auto bit = (1u << static_cast<unsigned>(kind));
  if (selected) {
    ui_state.draw_bundle_template_mask |= bit;
  } else {
    ui_state.draw_bundle_template_mask &= ~bit;
  }
}

std::vector<wire::core::BundleKind> SelectedBundleTemplatesLocal(const CoreState& state, const ViewerUiState& ui_state) {
  std::vector<wire::core::BundleKind> out;
  for (wire::core::BundleKind kind : SortedBundleTemplateKindsLocal(state)) {
    if (IsBundleTemplateSelectedLocal(ui_state, kind)) {
      out.push_back(kind);
    }
  }
  return out;
}

std::string BundleTemplatePreviewLocal(const CoreState& state, wire::core::BundleKind kind) {
  const auto it = state.view().bundle_templates().find(kind);
  if (it == state.view().bundle_templates().end()) {
    return std::to_string(static_cast<int>(kind));
  }
  return it->second.name;
}

std::string BundleTemplateMultiPreviewLocal(const CoreState& state, const ViewerUiState& ui_state) {
  const auto selected = SelectedBundleTemplatesLocal(state, ui_state);
  if (selected.empty()) {
    return "None";
  }
  if (selected.size() == 1) {
    return BundleTemplatePreviewLocal(state, selected.front());
  }
  std::ostringstream oss;
  for (std::size_t i = 0; i < selected.size(); ++i) {
    if (i > 0) {
      oss << ", ";
    }
    oss << BundleTemplatePreviewLocal(state, selected[i]);
  }
  return oss.str();
}

int ResolveBundleTemplateCountLocal(ViewerUiState& ui_state, const wire::core::BundleTemplate& bundle_template,
                                    wire::core::BundleKind kind) {
  const int key = static_cast<int>(kind);
  auto it = ui_state.draw_bundle_count_by_template.find(key);
  int count = (it == ui_state.draw_bundle_count_by_template.end()) ? bundle_template.default_count : it->second;
  count = std::clamp(count, bundle_template.min_count, bundle_template.max_count);
  ui_state.draw_bundle_count_by_template[key] = count;
  return count;
}

std::vector<wire::core::PoleTypeId> SortedPoleTypeIdsLocal(const CoreState& state) {
  std::vector<wire::core::PoleTypeId> ids;
  for (const auto& [id, _] : state.view().pole_types()) {
    ids.push_back(id);
  }
  std::sort(ids.begin(), ids.end());
  return ids;
}

std::size_t ClampedTypeIndexLocal(int current, std::size_t count) {
  if (count == 0) {
    return 0;
  }
  const int max_index = static_cast<int>(count - 1);
  return static_cast<std::size_t>(std::clamp(current, 0, max_index));
}

Vector3 ToRaylibLocal(const wire::core::Vec3d& ue_xyz) {
  return Vector3{static_cast<float>(ue_xyz.x), static_cast<float>(ue_xyz.z), static_cast<float>(ue_xyz.y)};
}

double PolylineLengthLocal(const std::vector<wire::core::Vec3d>& points) {
  double length = 0.0;
  for (std::size_t i = 0; i + 1 < points.size(); ++i) {
    const wire::core::Vec3d d = points[i + 1] - points[i];
    length += std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
  }
  return length;
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
    ui_state.draw_path_point_support_kinds.resize(ui_state.draw_path_points.size(), wire::core::SupportKind::kPole);
  } else if (ui_state.draw_path_point_support_kinds.size() > ui_state.draw_path_points.size()) {
    ui_state.draw_path_point_support_kinds.resize(ui_state.draw_path_points.size());
  }
  if (ui_state.draw_path_point_node_ids.size() < ui_state.draw_path_points.size()) {
    ui_state.draw_path_point_node_ids.resize(ui_state.draw_path_points.size(), wire::core::kInvalidObjectId);
  } else if (ui_state.draw_path_point_node_ids.size() > ui_state.draw_path_points.size()) {
    ui_state.draw_path_point_node_ids.resize(ui_state.draw_path_points.size());
  }
}

void DrawPathPushPoint(ViewerUiState& ui_state, const wire::core::Vec3d& point, wire::core::SupportKind support_kind,
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

void DrawPathClearWithSessionReset(ViewerUiState& ui_state) {
  DrawPathClearPoints(ui_state);
  ui_state.last_generation_session = 0;
}

void UpdateBranchPickInput(CoreState& state, const Camera3D& camera, ViewerUiState& ui_state) {
  ui_state.branch_hover_pick = {};
  ui_state.branch_hover_has_resolution = false;
  ui_state.branch_hover_status.clear();
  if (ui_state.mode != EditMode::kBranch || !ui_state.branch_pick_enabled) {
    return;
  }
  ImGuiIO& io = ImGui::GetIO();
  if (io.WantCaptureMouse || ui_state.camera_drag_mode != CameraDragMode::kNone || ui_state.camera_walk_mode) {
    return;
  }

  ViewerSceneQuery scene_query{};
  const wire::core::PickResult pick = scene_query.Raycast(state, camera, ui_state.draw_plane_z);
  ui_state.branch_hover_pick = pick;
  if (pick.hit_kind == wire::core::PickHitKind::kEmpty) {
    ui_state.branch_hover_status = "hover: Empty";
    return;
  }
  ui_state.branch_hover_status =
      std::string("hover: ") + PickHitKindLabelLocal(pick.hit_kind) + " id=" + PickTargetLabel(pick);

  wire::core::CoreState::ResolveBranchPickOptions options{};
  options.bundle_template_id = ResolveBundleTemplateForPathPick(state, ui_state.draw_bundle_template_mask, pick);
  options.snap_radius_world = ui_state.branch_snap_radius_world;
  options.create_midair_node = false;
  const auto resolved = state.ResolveBranchPick(pick, options);
  if (!resolved.ok) {
    ui_state.branch_hover_status += " -> blocked: " + resolved.error;
    if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
      ui_state.last_error = resolved.error;
      PushLogLocal(ui_state, "ResolveBranchPick failed: " + resolved.error);
    }
    return;
  }
  const std::string blocked_template =
      FindMidairBranchBlockedTemplateName(state, ResolveTemplateKindsForPathPick(state, ui_state.draw_bundle_template_mask, pick));
  if (resolved.value.resolution == wire::core::CoreState::PickBranchResolutionKind::kMidair && !blocked_template.empty()) {
    ui_state.branch_hover_status += " -> blocked: midair branch disabled by template " + blocked_template;
    if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
      ui_state.last_error = "bundle template does not allow midair branch";
      PushLogLocal(ui_state, "ResolveBranchPick failed: bundle template does not allow midair branch");
    }
    return;
  }
  ui_state.branch_hover_has_resolution = true;
  ui_state.branch_hover_resolution = resolved.value;
  ui_state.branch_hover_status +=
      (resolved.value.resolution == wire::core::CoreState::PickBranchResolutionKind::kNode) ? " -> Node" : " -> Midair";
  if (!IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
    return;
  }

  ui_state.branch_last_pick = pick;
  ui_state.branch_last_pick_summary = ui_state.branch_hover_status;
  ui_state.last_error.clear();
  wire::core::CoreState::ResolveBranchPickOptions click_options = options;
  click_options.create_midair_node = true;
  const auto applied = state.ResolveBranchPick(pick, click_options);
  if (!applied.ok) {
    ui_state.last_error = applied.error;
    PushLogLocal(ui_state, "ResolveBranchPick(click) failed: " + applied.error);
    return;
  }
  ui_state.branch_target_x = applied.value.position.x;
  ui_state.branch_target_y = applied.value.position.y;
  ui_state.branch_target_z = applied.value.position.z;
  if (applied.value.resolution == wire::core::CoreState::PickBranchResolutionKind::kNode) {
    if (applied.value.resolved_node_id != wire::core::kInvalidObjectId &&
        state.view().edit_state().poles.find(applied.value.resolved_node_id) != nullptr) {
      SetPrimarySelection(ui_state, SelectedType::kPole, applied.value.resolved_node_id);
    }
    if (pick.hit_kind == wire::core::PickHitKind::kSegment) {
      if (applied.value.resolved_node_id == pick.segment_node_a_id) {
        ui_state.branch_t = 0.0;
      } else if (applied.value.resolved_node_id == pick.segment_node_b_id) {
        ui_state.branch_t = 1.0;
      }
      ui_state.branch_source_span_id = pick.hit_id;
    }
    PushLogLocal(ui_state, applied.value.snapped_from_segment_endpoint ? "Branch pick snapped to segment endpoint node"
                                                                       : "Branch pick resolved to node");
  } else {
    PushLogLocal(ui_state, "Branch pick resolved to Midair support node");
  }
}
bool ExecuteBackboneRequest(CoreState& state, ViewerUiState& ui_state, const wire::core::BackboneSpec& request,
                            bool allow_session_regen, bool clear_draw_path_on_success, const char* success_log,
                            const char* failure_log) {
  const bool use_session_regen = ui_state.draw_regenerate_last_session && ui_state.last_generation_session != 0;
  const bool run_regen = allow_session_regen && use_session_regen;
  const auto result = run_regen ? state.RegenerateSessionAutoParts(ui_state.last_generation_session, request)
                                : state.GenerateFromBackboneSpec(request);
  if (!result.ok) {
    ui_state.last_error = result.error;
    PushLogLocal(ui_state, failure_log);
    return false;
  }

  ui_state.last_error.clear();
  ui_state.last_generated_poles = static_cast<int>(result.value.generated_pole_ids.size());
  ui_state.last_generated_spans = static_cast<int>(result.value.generated_span_ids.size());
  std::uint64_t resolved_session_id = run_regen ? ui_state.last_generation_session : 0;
  if (resolved_session_id == 0) {
    if (!result.value.generated_span_ids.empty()) {
      const auto* last_span = state.view().edit_state().spans.find(result.value.generated_span_ids.back());
      if (last_span != nullptr) {
        resolved_session_id = last_span->generation.generation_session_id;
      }
    } else if (!result.value.generated_pole_ids.empty()) {
      const auto* last_pole = state.view().edit_state().poles.find(result.value.generated_pole_ids.back());
      if (last_pole != nullptr) {
        resolved_session_id = last_pole->generation.generation_session_id;
      }
    }
  }
  ui_state.last_generation_session = resolved_session_id;
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
  return true;
}

void ExecuteGenerateFromDrawPath(CoreState& state, ViewerUiState& ui_state, bool from_enter_key) {
  EnsureDrawPathPointKinds(ui_state);
  if (ui_state.draw_path_points.size() < 2) {
    ui_state.last_error = "path needs at least 2 points";
    return;
  }

  const auto type_ids = SortedPoleTypeIdsLocal(state);
  if (type_ids.empty()) {
    ui_state.last_error = "no pole type available";
    return;
  }

  const std::size_t road_type_index = ClampedTypeIndexLocal(ui_state.road_pole_type_index, type_ids.size());
  ui_state.road_pole_type_index = static_cast<int>(road_type_index);

  wire::core::RoadSegment road{};
  road.id = ui_state.road_id++;
  road.polyline = ui_state.draw_path_points;

  const auto selected_templates = SelectedBundleTemplatesLocal(state, ui_state);
  if (selected_templates.empty()) {
    ui_state.last_error = "select at least one bundle template";
    return;
  }

  const int mode_index = std::clamp(ui_state.draw_direction_mode, 0, 2);
  ui_state.draw_direction_mode = mode_index;

  wire::core::BackboneSpec request{};
  request.path.polyline = road.polyline;
  for (std::size_t i = 0; i < ui_state.draw_path_points.size(); ++i) {
    const wire::core::SupportKind support_kind = ui_state.draw_path_point_support_kinds[i];
    const ObjectId node_id =
        (i < ui_state.draw_path_point_node_ids.size()) ? ui_state.draw_path_point_node_ids[i] : wire::core::kInvalidObjectId;
    if (support_kind == wire::core::SupportKind::kPole && node_id == wire::core::kInvalidObjectId) {
      continue;
    }
    wire::core::BackboneInputSpec::NodeSpec node_spec{};
    node_spec.point_index = i;
    node_spec.support_kind = support_kind;
    node_spec.node_id = node_id;
    request.path.node_specs.push_back(node_spec);
  }
  if (ui_state.draw_clicked_points_only) {
    request.interval_m = std::max(0.001, PolylineLengthLocal(road.polyline) + 1.0);
  } else {
    request.interval_m = ui_state.draw_interval_m;
  }
  request.pole_type_id = type_ids[ui_state.road_pole_type_index];
  request.direction_mode = static_cast<wire::core::PathDirectionMode>(mode_index);
  for (wire::core::BundleKind kind : selected_templates) {
    const auto it = state.view().bundle_templates().find(kind);
    if (it == state.view().bundle_templates().end()) {
      ui_state.last_error = "selected bundle template is not available";
      return;
    }
    const wire::core::BundleTemplate& bundle_template = it->second;
    wire::core::BackboneBundleSpec bundle_request{};
    bundle_request.bundle_template_id = kind;
    bundle_request.layer = wire::core::SpanLayer::kUnknown;
    bundle_request.count = 0;
    if (bundle_template.count_rule == wire::core::BundleCountRuleKind::kRange) {
      bundle_request.count = ResolveBundleTemplateCountLocal(ui_state, bundle_template, kind);
    }
    request.bundles.push_back(bundle_request);
  }

  const std::string success_log =
      std::string(from_enter_key ? "Generated path (Enter)" : "Generated path") + " templates=" +
      BundleTemplateMultiPreviewLocal(state, ui_state);
  const char* failure_log = from_enter_key ? "Generate path (Enter) failed" : "Generate path failed";
  (void)ExecuteBackboneRequest(state, ui_state, request, true, !ui_state.draw_keep_path_after_generate,
                               success_log.c_str(), failure_log);
}

bool SaveDrawPathReproCapture(const CoreState& state, const ViewerUiState& ui_state, std::string* out_path,
                              std::string* out_error) {
  namespace fs = std::filesystem;

  std::time_t now = std::time(nullptr);
  std::tm tm_now{};
#if defined(_WIN32)
  localtime_s(&tm_now, &now);
#else
  localtime_r(&now, &tm_now);
#endif
  char stamp[32]{};
  std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", &tm_now);

  std::error_code ec;
  const fs::path capture_dir = fs::path("captures");
  fs::create_directories(capture_dir, ec);
  if (ec) {
    if (out_error != nullptr) {
      *out_error = "failed to create captures directory";
    }
    return false;
  }
  const fs::path capture_path = capture_dir / fs::path(std::string("drawpath_repro_") + stamp + ".txt");
  std::ofstream ofs(capture_path.string(), std::ios::trunc);
  if (!ofs.is_open()) {
    if (out_error != nullptr) {
      *out_error = "failed to open capture file for writing";
    }
    return false;
  }

  const auto selected_templates = SelectedBundleTemplatesLocal(state, ui_state);
  const auto& view = state.view();
  const auto& dir_debug = view.last_path_direction_debug();
  const auto& slot_debug_records = view.slot_selection_debug_records();

  ofs << "capture.version=2\n";
  ofs << "capture.timestamp_unix=" << static_cast<long long>(now) << "\n";
  ofs << "capture.mode=" << ModeLabelLocal(ui_state.mode) << "\n";
  ofs << "capture.selected_type=" << SelectedTypeLabelLocal(ui_state.selected_type) << "\n";
  ofs << "capture.selected_id=" << static_cast<unsigned long long>(ui_state.selected_id) << "\n";
  ofs << "capture.last_error=" << ui_state.last_error << "\n";
  ofs << "capture.last_generation_session=" << static_cast<unsigned long long>(ui_state.last_generation_session) << "\n";
  ofs << "capture.last_generated_poles=" << ui_state.last_generated_poles << "\n";
  ofs << "capture.last_generated_spans=" << ui_state.last_generated_spans << "\n";

  ofs << "draw.path_count=" << ui_state.draw_path_points.size() << "\n";
  ofs << "draw.path_support_kind_count=" << ui_state.draw_path_point_support_kinds.size() << "\n";
  ofs << "draw.path_node_id_count=" << ui_state.draw_path_point_node_ids.size() << "\n";
  for (std::size_t i = 0; i < ui_state.draw_path_points.size(); ++i) {
    const auto& p = ui_state.draw_path_points[i];
    ofs << "draw.path[" << i << "]=" << p.x << "," << p.y << "," << p.z << "\n";
    const wire::core::SupportKind support_kind =
        (i < ui_state.draw_path_point_support_kinds.size()) ? ui_state.draw_path_point_support_kinds[i]
                                                            : wire::core::SupportKind::kPole;
    ofs << "draw.path[" << i << "].support_kind=" << SupportKindLabelLocal(support_kind) << "\n";
    const ObjectId node_id =
        (i < ui_state.draw_path_point_node_ids.size()) ? ui_state.draw_path_point_node_ids[i] : wire::core::kInvalidObjectId;
    ofs << "draw.path[" << i << "].node_id=" << static_cast<unsigned long long>(node_id) << "\n";
  }
  ofs << "draw.interval_m=" << ui_state.draw_interval_m << "\n";
  ofs << "draw.clicked_points_only=" << (ui_state.draw_clicked_points_only ? 1 : 0) << "\n";
  ofs << "draw.direction_mode="
      << PathDirectionModeLabelLocal(
             static_cast<wire::core::PathDirectionMode>(std::clamp(ui_state.draw_direction_mode, 0, 2)))
      << "\n";
  ofs << "draw.keep_path_after_generate=" << (ui_state.draw_keep_path_after_generate ? 1 : 0) << "\n";
  ofs << "draw.regenerate_last_session=" << (ui_state.draw_regenerate_last_session ? 1 : 0) << "\n";
  ofs << "draw.plane_z=" << ui_state.draw_plane_z << "\n";

  const auto type_ids = SortedPoleTypeIdsLocal(state);
  if (!type_ids.empty()) {
    const std::size_t idx = ClampedTypeIndexLocal(ui_state.road_pole_type_index, type_ids.size());
    ofs << "draw.pole_type_id=" << static_cast<unsigned long long>(type_ids[idx]) << "\n";
  } else {
    ofs << "draw.pole_type_id=none\n";
  }

  ofs << "draw.bundle_template_count=" << selected_templates.size() << "\n";
  for (std::size_t i = 0; i < selected_templates.size(); ++i) {
    const wire::core::BundleKind kind = selected_templates[i];
    const auto it = view.bundle_templates().find(kind);
    ofs << "draw.bundle[" << i << "].kind=" << static_cast<int>(kind) << "\n";
    if (it != view.bundle_templates().end()) {
      const auto& tpl = it->second;
      ofs << "draw.bundle[" << i << "].name=" << tpl.name << "\n";
      ofs << "draw.bundle[" << i << "].category=" << CategoryLabelLocal(tpl.category) << "\n";
      ofs << "draw.bundle[" << i << "].default_layer=" << static_cast<int>(tpl.default_layer) << "\n";
      ofs << "draw.bundle[" << i << "].count_rule=" << static_cast<int>(tpl.count_rule) << "\n";
      if (tpl.count_rule == wire::core::BundleCountRuleKind::kFixed) {
        ofs << "draw.bundle[" << i << "].count=" << tpl.fixed_count << "\n";
      } else {
        const int key = static_cast<int>(kind);
        auto it_count = ui_state.draw_bundle_count_by_template.find(key);
        int count = (it_count == ui_state.draw_bundle_count_by_template.end()) ? tpl.default_count : it_count->second;
        count = std::clamp(count, tpl.min_count, tpl.max_count);
        ofs << "draw.bundle[" << i << "].count=" << count << "\n";
      }
    }
  }

  ofs << "result.direction.requested_mode=" << PathDirectionModeLabelLocal(dir_debug.requested_mode) << "\n";
  ofs << "result.direction.chosen=" << PathDirectionChosenLabelLocal(dir_debug.chosen) << "\n";
  ofs << "result.direction.cost.forward.total=" << dir_debug.forward_cost.total << "\n";
  ofs << "result.direction.cost.reverse.total=" << dir_debug.reverse_cost.total << "\n";
  ofs << "result.direction.cost.forward.cross=" << dir_debug.forward_cost.estimated_cross_penalty << "\n";
  ofs << "result.direction.cost.forward.side_flip=" << dir_debug.forward_cost.side_flip_penalty << "\n";
  ofs << "result.direction.cost.forward.layer_jump=" << dir_debug.forward_cost.layer_jump_penalty << "\n";
  ofs << "result.direction.cost.forward.corner=" << dir_debug.forward_cost.corner_compression_penalty << "\n";
  ofs << "result.direction.cost.forward.branch=" << dir_debug.forward_cost.branch_conflict_penalty << "\n";

  if (ui_state.draw_capture_include_slot_debug) {
    const int tail = std::max(0, ui_state.draw_capture_slot_debug_tail);
    const int count = static_cast<int>(slot_debug_records.size());
    const int begin = std::max(0, count - tail);
    ofs << "result.slot_debug.total_count=" << count << "\n";
    ofs << "result.slot_debug.dump_from=" << begin << "\n";
    for (int i = begin; i < count; ++i) {
      const auto& event = slot_debug_records[static_cast<std::size_t>(i)];
      ofs << "slot_debug[" << i << "].pole_id=" << static_cast<unsigned long long>(event.pole_id) << "\n";
      ofs << "slot_debug[" << i << "].peer_pole_id=" << static_cast<unsigned long long>(event.peer_pole_id) << "\n";
      ofs << "slot_debug[" << i << "].category=" << CategoryLabelLocal(event.category) << "\n";
      ofs << "slot_debug[" << i << "].context=" << ContextLabelLocal(event.connection_context) << "\n";
      ofs << "slot_debug[" << i << "].selected_slot_id=" << event.selected_slot_id << "\n";
      ofs << "slot_debug[" << i << "].result=" << event.result << "\n";
    }
  } else {
    ofs << "result.slot_debug.included=0\n";
  }

  const auto& backbone = view.last_generation_backbone();
  ofs << "result.backbone.node_count=" << backbone.nodes.size() << "\n";
  for (std::size_t i = 0; i < backbone.nodes.size(); ++i) {
    const auto& node = backbone.nodes[i];
    ofs << "result.backbone.node[" << i << "].id=" << static_cast<unsigned long long>(node.node_id) << "\n";
    ofs << "result.backbone.node[" << i << "].support_kind=" << SupportKindLabelLocal(node.support_kind) << "\n";
    ofs << "result.backbone.node[" << i << "].position=" << node.position.x << "," << node.position.y << ","
        << node.position.z << "\n";
    ofs << "result.backbone.node[" << i << "].pole_id=" << static_cast<unsigned long long>(node.pole_id) << "\n";
    ofs << "result.backbone.node[" << i << "].path_point_index=" << node.path_point_index << "\n";
    ofs << "result.backbone.node[" << i << "].has_tangent_hint=" << (node.has_tangent_hint ? 1 : 0) << "\n";
    if (node.has_tangent_hint) {
      ofs << "result.backbone.node[" << i << "].tangent_hint=" << node.tangent_hint.x << "," << node.tangent_hint.y
          << "," << node.tangent_hint.z << "\n";
    }
    ofs << "result.backbone.node[" << i << "].bundle_mode_count=" << node.bundle_modes.size() << "\n";
    for (std::size_t j = 0; j < node.bundle_modes.size(); ++j) {
      const auto& bundle_mode = node.bundle_modes[j];
      ofs << "result.backbone.node[" << i << "].bundle_mode[" << j
          << "].template_id=" << static_cast<int>(bundle_mode.bundle_template_id) << "\n";
      ofs << "result.backbone.node[" << i << "].bundle_mode[" << j
          << "].mode=" << static_cast<int>(bundle_mode.mode) << "\n";
    }
    if (node.pole_id != wire::core::kInvalidObjectId) {
      if (const auto* pole = view.edit_state().poles.find(node.pole_id); pole != nullptr) {
        ofs << "result.backbone.node[" << i << "].pole_yaw_deg=" << pole->world_transform.rotation_euler_deg.z
            << "\n";
        ofs << "result.backbone.node[" << i
            << "].sharp_orientation_applied=" << (pole->context.sharp_orientation_applied ? 1 : 0) << "\n";
        ofs << "result.backbone.node[" << i << "].sharp_theta_deg=" << pole->context.sharp_theta_deg << "\n";
        ofs << "result.backbone.node[" << i << "].sharp_bisector_dir=" << pole->context.sharp_bisector_dir.x << ","
            << pole->context.sharp_bisector_dir.y << "," << pole->context.sharp_bisector_dir.z << "\n";
        ofs << "result.backbone.node[" << i << "].sharp_side_dir=" << pole->context.sharp_side_dir.x << ","
            << pole->context.sharp_side_dir.y << "," << pole->context.sharp_side_dir.z << "\n";
      }
    }
  }
  ofs << "result.backbone.edge_count=" << backbone.edges.size() << "\n";
  for (std::size_t i = 0; i < backbone.edges.size(); ++i) {
    const auto& edge = backbone.edges[i];
    ofs << "result.backbone.edge[" << i << "].node_a_id=" << static_cast<unsigned long long>(edge.node_a) << "\n";
    ofs << "result.backbone.edge[" << i << "].node_b_id=" << static_cast<unsigned long long>(edge.node_b) << "\n";
  }
  ofs << "result.backbone.edge_orientation_count=" << backbone.edge_orientations.size() << "\n";
  for (std::size_t i = 0; i < backbone.edge_orientations.size(); ++i) {
    const auto& edge_orientation = backbone.edge_orientations[i];
    ofs << "result.backbone.edge_orientation[" << i
        << "].node_a_id=" << static_cast<unsigned long long>(edge_orientation.node_a_id) << "\n";
    ofs << "result.backbone.edge_orientation[" << i
        << "].node_b_id=" << static_cast<unsigned long long>(edge_orientation.node_b_id) << "\n";
    ofs << "result.backbone.edge_orientation[" << i
        << "].bundle_template_id=" << static_cast<int>(edge_orientation.bundle_template_id) << "\n";
    ofs << "result.backbone.edge_orientation[" << i
        << "].orientation=" << static_cast<int>(edge_orientation.orientation) << "\n";
    ofs << "result.backbone.edge_orientation[" << i
        << "].flipped_from_previous=" << (edge_orientation.flipped_from_previous ? 1 : 0) << "\n";
    ofs << "result.backbone.edge_orientation[" << i
        << "].flip_reason=" << static_cast<int>(edge_orientation.flip_reason) << "\n";
    ofs << "result.backbone.edge_orientation[" << i << "].turn_angle_deg=" << edge_orientation.turn_angle_deg
        << "\n";
  }

  ofs << "state.poles=" << view.edit_state().poles.size() << "\n";
  ofs << "state.ports=" << view.edit_state().ports.size() << "\n";
  ofs << "state.bundles=" << view.edit_state().bundles.size() << "\n";
  ofs << "state.spans=" << view.edit_state().spans.size() << "\n";

  ofs.flush();
  if (!ofs.good()) {
    if (out_error != nullptr) {
      *out_error = "failed to write capture file";
    }
    return false;
  }

  if (out_path != nullptr) {
    *out_path = capture_path.string();
  }
  return true;
}
void UpdateDrawPathInput(CoreState& state, const Camera3D& camera, ViewerUiState& ui_state) {
  ui_state.draw_hover_pick = {};
  ui_state.draw_hover_has_resolution = false;
  ui_state.draw_hover_status.clear();
  if (ui_state.mode != EditMode::kDrawPath) {
    ui_state.draw_hover_valid = false;
    return;
  }
  EnsureDrawPathPointKinds(ui_state);

  ImGuiIO& io = ImGui::GetIO();
  wire::core::Vec3d hover{};
  const bool has_ground_hit = TryPickGroundPoint(camera, ui_state.draw_plane_z, &hover);
  ui_state.draw_hover_valid = has_ground_hit;
  if (has_ground_hit) {
    ui_state.draw_hover_point = hover;
  }
  const bool accept_mouse_input =
      !io.WantCaptureMouse && ui_state.camera_drag_mode == CameraDragMode::kNone && !ui_state.camera_walk_mode;

  if (ui_state.draw_pick_enabled && accept_mouse_input) {
    ViewerSceneQuery scene_query{};
    const wire::core::PickResult pick = scene_query.Raycast(state, camera, ui_state.draw_plane_z);
    ui_state.draw_hover_pick = pick;
    bool blocked_pick_target = false;
    if (pick.hit_kind == wire::core::PickHitKind::kEmpty) {
      ui_state.draw_hover_status = "hover: Empty";
    } else {
      ui_state.draw_hover_status =
          std::string("hover: ") + PickHitKindLabelLocal(pick.hit_kind) + " id=" + PickTargetLabel(pick);
      if (pick.hit_kind == wire::core::PickHitKind::kNode || pick.hit_kind == wire::core::PickHitKind::kSegment ||
          pick.hit_kind == wire::core::PickHitKind::kBuilding) {
        wire::core::CoreState::ResolveBranchPickOptions options{};
        options.bundle_template_id = ResolveBundleTemplateForPathPick(state, ui_state.draw_bundle_template_mask, pick);
        options.snap_radius_world = ui_state.draw_snap_radius_world;
        options.create_midair_node = false;
        options.enforce_midair_template_policy = false;
        const auto resolved = state.ResolveBranchPick(pick, options);
        if (resolved.ok) {
          const std::string blocked_template = FindMidairBranchBlockedTemplateName(
              state, ResolveTemplateKindsForPathPick(state, ui_state.draw_bundle_template_mask, pick));
          if (resolved.value.resolution == wire::core::CoreState::PickBranchResolutionKind::kMidair &&
              !blocked_template.empty()) {
            ui_state.draw_hover_status += " -> warn: template " + blocked_template + " will not connect here";
          }
          ui_state.draw_hover_has_resolution = true;
          ui_state.draw_hover_resolution = resolved.value;
          ui_state.draw_hover_point = resolved.value.position;
          ui_state.draw_hover_valid = true;
          ui_state.draw_hover_status += (resolved.value.resolution == wire::core::CoreState::PickBranchResolutionKind::kNode)
                                            ? " -> Node"
                                            : " -> Midair";
        } else {
          ui_state.draw_hover_status += " -> blocked: " + resolved.error;
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
        wire::core::CoreState::ResolveBranchPickOptions click_options{};
        click_options.bundle_template_id =
            ResolveBundleTemplateForPathPick(state, ui_state.draw_bundle_template_mask, ui_state.draw_hover_pick);
        click_options.snap_radius_world = ui_state.draw_snap_radius_world;
        click_options.create_midair_node = true;
        click_options.enforce_midair_template_policy = false;
        const auto applied = state.ResolveBranchPick(ui_state.draw_hover_pick, click_options);
        if (!applied.ok) {
          ui_state.last_error = applied.error;
          PushLogLocal(ui_state, "DrawPath ResolveBranchPick(click) failed: " + applied.error);
        } else {
          ui_state.last_error.clear();
          DrawPathPushPoint(ui_state, applied.value.position, applied.value.support_kind, applied.value.resolved_node_id);
        }
      } else {
        DrawPathPushPoint(ui_state, ui_state.draw_hover_point, wire::core::SupportKind::kPole,
                          wire::core::kInvalidObjectId);
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
      DrawPathClearWithSessionReset(ui_state);
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
    const wire::core::SupportKind support_kind =
        (i < ui_state.draw_path_point_support_kinds.size()) ? ui_state.draw_path_point_support_kinds[i]
                                                            : wire::core::SupportKind::kPole;
    const ObjectId node_id =
        (i < ui_state.draw_path_point_node_ids.size()) ? ui_state.draw_path_point_node_ids[i] : wire::core::kInvalidObjectId;
    Color point_color = Color{196, 156, 68, 255};
    if (support_kind == wire::core::SupportKind::kMidair) {
      point_color = Color{90, 154, 176, 255};
    } else if (support_kind == wire::core::SupportKind::kBuilding) {
      point_color = Color{110, 154, 100, 255};
    }
    DrawSphere(ToRaylibLocal(ui_state.draw_path_points[i]), 0.12f, point_color);
    if (node_id != wire::core::kInvalidObjectId) {
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
  ImGui::Checkbox("Regenerate Last Session (Enter Extend)", &ui_state.draw_regenerate_last_session);
  ImGui::TextUnformatted("Template-driven bundle generation");
  ImGui::Text("Path points: %d", static_cast<int>(ui_state.draw_path_points.size()));
  const int midair_points = static_cast<int>(
      std::count(ui_state.draw_path_point_support_kinds.begin(), ui_state.draw_path_point_support_kinds.end(),
                 wire::core::SupportKind::kMidair));
  const int building_points = static_cast<int>(
      std::count(ui_state.draw_path_point_support_kinds.begin(), ui_state.draw_path_point_support_kinds.end(),
                 wire::core::SupportKind::kBuilding));
  const int anchored_points = static_cast<int>(
      std::count_if(ui_state.draw_path_point_node_ids.begin(), ui_state.draw_path_point_node_ids.end(),
                    [](ObjectId node_id) { return node_id != wire::core::kInvalidObjectId; }));
  ImGui::Text("Support kind points: Midair=%d Building=%d", midair_points, building_points);
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
                (ui_state.draw_hover_resolution.resolution == wire::core::CoreState::PickBranchResolutionKind::kNode)
                    ? "Node"
                    : "Midair",
                SupportKindLabelLocal(ui_state.draw_hover_resolution.support_kind));
  }

  const auto type_ids = SortedPoleTypeIdsLocal(state);
  if (!type_ids.empty()) {
    const std::size_t road_type_index = ClampedTypeIndexLocal(ui_state.road_pole_type_index, type_ids.size());
    ui_state.road_pole_type_index = static_cast<int>(road_type_index);
    const wire::core::PoleTypeId road_type_id = type_ids[road_type_index];
    const auto road_type_it = state.view().pole_types().find(road_type_id);
    const std::string road_type_name =
        (road_type_it != state.view().pole_types().end()) ? road_type_it->second.name : std::to_string(road_type_id);
    if (ImGui::BeginCombo("Path PoleType", road_type_name.c_str())) {
      for (std::size_t i = 0; i < type_ids.size(); ++i) {
        const auto it = state.view().pole_types().find(type_ids[i]);
        const std::string label = (it != state.view().pole_types().end()) ? it->second.name : std::to_string(type_ids[i]);
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

  const auto template_ids = SortedBundleTemplateKindsLocal(state);
  if (template_ids.empty()) {
    ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.35f, 1.0f), "No bundle template registered in core");
  } else {
    if (ImGui::BeginCombo("Bundle Templates", BundleTemplateMultiPreviewLocal(state, ui_state).c_str())) {
      for (const wire::core::BundleKind kind : template_ids) {
        bool selected = IsBundleTemplateSelectedLocal(ui_state, kind);
        if (ImGui::Selectable(BundleTemplatePreviewLocal(state, kind).c_str(), selected,
                              ImGuiSelectableFlags_DontClosePopups)) {
          selected = !selected;
          SetBundleTemplateSelectedLocal(ui_state, kind, selected);
        }
        if (selected) {
          ImGui::SetItemDefaultFocus();
        }
      }
      ImGui::EndCombo();
    }
    const auto selected_templates = SelectedBundleTemplatesLocal(state, ui_state);
    if (selected_templates.empty()) {
      ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.35f, 1.0f), "Select at least one bundle template");
    }
    for (wire::core::BundleKind kind : selected_templates) {
      const auto it = state.view().bundle_templates().find(kind);
      if (it == state.view().bundle_templates().end()) {
        continue;
      }
      const wire::core::BundleTemplate& bundle_template = it->second;
      ImGui::Separator();
      ImGui::TextUnformatted(BundleTemplatePreviewLocal(state, kind).c_str());
      ImGui::Text("Category: %s", CategoryLabelLocal(bundle_template.category));
      if (bundle_template.count_rule == wire::core::BundleCountRuleKind::kRange) {
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
                            static_cast<wire::core::PathDirectionMode>(ui_state.draw_direction_mode)))) {
    for (int i = 0; i < 3; ++i) {
      const bool selected = (i == ui_state.draw_direction_mode);
      const auto mode = static_cast<wire::core::PathDirectionMode>(i);
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
    if (ui_state.draw_direction_mode == static_cast<int>(wire::core::PathDirectionMode::kReverse)) {
      ui_state.draw_direction_mode = static_cast<int>(wire::core::PathDirectionMode::kForward);
    } else {
      ui_state.draw_direction_mode = static_cast<int>(wire::core::PathDirectionMode::kReverse);
    }
  }
  const auto& dir_debug = state.view().last_path_direction_debug();
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
  if (ImGui::Button("Save Repro Capture")) {
    std::string path{};
    std::string error{};
    if (SaveDrawPathReproCapture(state, ui_state, &path, &error)) {
      ui_state.last_repro_capture_path = path;
      PushLogLocal(ui_state, "Saved repro capture: " + path);
    } else {
      ui_state.last_error = error;
      PushLogLocal(ui_state, "Save repro capture failed");
    }
  }
  ImGui::SameLine();
  if (ImGui::Button("Undo Last Point")) {
    DrawPathPopPoint(ui_state);
  }
  ImGui::SameLine();
  if (ImGui::Button("Clear Path")) {
    DrawPathClearWithSessionReset(ui_state);
  }
  ImGui::Checkbox("Capture Slot Debug", &ui_state.draw_capture_include_slot_debug);
  ImGui::InputInt("Capture Slot Debug Tail", &ui_state.draw_capture_slot_debug_tail);
  ui_state.draw_capture_slot_debug_tail = std::clamp(ui_state.draw_capture_slot_debug_tail, 0, 5000);
  if (!ui_state.last_repro_capture_path.empty()) {
    ImGui::TextWrapped("Last capture: %s", ui_state.last_repro_capture_path.c_str());
  }
}

void DrawBranchModePanel(CoreState& state, ViewerUiState& ui_state) {
  ImGui::TextUnformatted("Branch / Drop");
  ImGui::Separator();
  ImGui::TextUnformatted("Raycast Pick Routing");
  ImGui::Checkbox("Enable Branch Pick (LMB)", &ui_state.branch_pick_enabled);
  ImGui::InputDouble("Snap Radius (world)", &ui_state.branch_snap_radius_world, 0.05, 0.2, "%.2f");
  ui_state.branch_snap_radius_world = std::clamp(ui_state.branch_snap_radius_world, 0.05, 5.0);
  ImGui::TextUnformatted("Segment hit near endpoint -> snap to node");
  ImGui::TextUnformatted("Segment hit away from endpoint -> Midair candidate");
  if (!ui_state.branch_last_pick_summary.empty()) {
    ImGui::TextWrapped("%s", ui_state.branch_last_pick_summary.c_str());
    ImGui::Text("Last hit kind: %s", PickHitKindLabelLocal(ui_state.branch_last_pick.hit_kind));
  }

  ImGui::Separator();
  ImGui::TextUnformatted("Manual Drop Commands");
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
      PushLogLocal(ui_state, "AddDropFromSpan failed");
    } else {
      ui_state.last_error.clear();
      SetPrimarySelection(ui_state, SelectedType::kSpan, result.value.span_id);
      PushLogLocal(ui_state, "DropFromSpan span=" + std::to_string(result.value.span_id) +
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
      PushLogLocal(ui_state, "AddDropFromPole failed");
    } else {
      ui_state.last_error.clear();
      SetPrimarySelection(ui_state, SelectedType::kSpan, result.value.span_id);
      PushLogLocal(ui_state, "DropFromPole span=" + std::to_string(result.value.span_id) +
                                " sourcePort=" + std::to_string(result.value.source_port_id));
    }
  }
}


