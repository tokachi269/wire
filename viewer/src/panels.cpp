#include "panels.hpp"

#include <algorithm>
#include <functional>
#include <string>
#include <vector>

#include "imgui.h"
#include "raylib.h"
#include "ui_common.hpp"

namespace {

std::string DirtyBitsToText(wire::core::DirtyBits bits) {
  std::string out;
  if ((bits & wire::core::DirtyBits::kTopology) != wire::core::DirtyBits::kNone) out += "T";
  if ((bits & wire::core::DirtyBits::kGeometry) != wire::core::DirtyBits::kNone) out += "G";
  if ((bits & wire::core::DirtyBits::kBounds) != wire::core::DirtyBits::kNone) out += "B";
  if ((bits & wire::core::DirtyBits::kRender) != wire::core::DirtyBits::kNone) out += "R";
  if ((bits & wire::core::DirtyBits::kRaycast) != wire::core::DirtyBits::kNone) out += "X";
  return out.empty() ? std::string("-") : out;
}

const char* SpanLayerLabel(wire::core::SpanLayer layer) {
  switch (layer) {
  case wire::core::SpanLayer::kHighVoltage:
    return "HighVoltage";
  case wire::core::SpanLayer::kLowVoltage:
    return "LowVoltage";
  case wire::core::SpanLayer::kCommunication:
    return "Communication";
  case wire::core::SpanLayer::kOptical:
    return "Optical";
  default:
    return "Unknown";
  }
}

const char* CableMaterialStyleLabel(wire::core::CableMaterialStyleKind kind) {
  switch (kind) {
  case wire::core::CableMaterialStyleKind::kGeneric:
    return "Generic";
  case wire::core::CableMaterialStyleKind::kBareConductor:
    return "BareConductor";
  case wire::core::CableMaterialStyleKind::kInsulated:
    return "Insulated";
  case wire::core::CableMaterialStyleKind::kOptical:
    return "Optical";
  default:
    return "Unknown";
  }
}

const char* ContinuityPolicyLabel(wire::core::CableContinuityPolicyHint policy) {
  switch (policy) {
  case wire::core::CableContinuityPolicyHint::kAuto:
    return "Auto";
  case wire::core::CableContinuityPolicyHint::kPreferG1:
    return "PreferG1";
  case wire::core::CableContinuityPolicyHint::kPreferG2:
    return "PreferG2";
  default:
    return "Unknown";
  }
}

const char* DetailCurveContinuityModeLabel(wire::core::DetailCurveContinuityMode mode) {
  switch (mode) {
  case wire::core::DetailCurveContinuityMode::kG1:
    return "G1";
  case wire::core::DetailCurveContinuityMode::kG2:
    return "G2";
  default:
    return "Unknown";
  }
}

const char* DetailCurveContinuityReasonLabel(wire::core::DetailCurveContinuityReason reason) {
  switch (reason) {
  case wire::core::DetailCurveContinuityReason::kAutoBalanced:
    return "AutoBalanced";
  case wire::core::DetailCurveContinuityReason::kSmoothPassThrough:
    return "SmoothPassThrough";
  case wire::core::DetailCurveContinuityReason::kPolicyPreferG1:
    return "PolicyPreferG1";
  case wire::core::DetailCurveContinuityReason::kShortSpan:
    return "ShortSpan";
  case wire::core::DetailCurveContinuityReason::kBranchPass:
    return "BranchPass";
  case wire::core::DetailCurveContinuityReason::kCornerPass:
    return "CornerPass";
  case wire::core::DetailCurveContinuityReason::kEndpointConstraintPriority:
    return "EndpointConstraintPriority";
  case wire::core::DetailCurveContinuityReason::kConflictingTangents:
    return "ConflictingTangents";
  case wire::core::DetailCurveContinuityReason::kContextInsufficient:
    return "ContextInsufficient";
  case wire::core::DetailCurveContinuityReason::kPoorQualityFallback:
    return "PoorQualityFallback";
  default:
    return "Unknown";
  }
}

const char* BundleSupportStyleLabel(wire::core::BundleSupportStyleHint style) {
  switch (style) {
  case wire::core::BundleSupportStyleHint::kAuto:
    return "Auto";
  case wire::core::BundleSupportStyleHint::kCenterPreferred:
    return "CenterPreferred";
  case wire::core::BundleSupportStyleHint::kSideStructurePreferred:
    return "SideStructurePreferred";
  default:
    return "Unknown";
  }
}

const char* BundleBranchPolicyLabel(wire::core::BundleBranchPolicyHint policy) {
  switch (policy) {
  case wire::core::BundleBranchPolicyHint::kAuto:
    return "Auto";
  case wire::core::BundleBranchPolicyHint::kPreferPassThrough:
    return "PreferPassThrough";
  case wire::core::BundleBranchPolicyHint::kPreferExplicitBranch:
    return "PreferExplicitBranch";
  default:
    return "Unknown";
  }
}

const char* BackboneFlowKindLabel(wire::core::BackboneFlowKind kind) {
  switch (kind) {
  case wire::core::BackboneFlowKind::kMain:
    return "Main";
  case wire::core::BackboneFlowKind::kBranch:
    return "Branch";
  default:
    return "Unknown";
  }
}

const char* BackboneFlowDecisionRuleLabel(wire::core::BackboneFlowDecisionRule rule) {
  switch (rule) {
  case wire::core::BackboneFlowDecisionRule::kDefaultMain:
    return "DefaultMain";
  case wire::core::BackboneFlowDecisionRule::kJunctionOrderMain:
    return "JunctionOrderMain";
  case wire::core::BackboneFlowDecisionRule::kJunctionOrderBranch:
    return "JunctionOrderBranch";
  case wire::core::BackboneFlowDecisionRule::kExistingChainMain:
    return "ExistingChainMain";
  case wire::core::BackboneFlowDecisionRule::kExistingChainBranch:
    return "ExistingChainBranch";
  default:
    return "Unknown";
  }
}

wire::core::BundleKind BundleTemplateForCategory(wire::core::ConnectionCategory category) {
  switch (category) {
  case wire::core::ConnectionCategory::kHighVoltage:
    return wire::core::BundleKind::kHighVoltage;
  case wire::core::ConnectionCategory::kCommunication:
    return wire::core::BundleKind::kCommunication;
  case wire::core::ConnectionCategory::kOptical:
    return wire::core::BundleKind::kOptical;
  case wire::core::ConnectionCategory::kLowVoltage:
  case wire::core::ConnectionCategory::kDrop:
  default:
    return wire::core::BundleKind::kLowVoltage;
  }
}

const char* PoleForwardRuleLabel(wire::core::PoleForwardRule rule) {
  switch (rule) {
  case wire::core::PoleForwardRule::kFallback:
    return "Fallback";
  case wire::core::PoleForwardRule::kPrimaryIncident:
    return "PrimaryIncident";
  case wire::core::PoleForwardRule::kMainChainSingle:
    return "MainChainSingle";
  case wire::core::PoleForwardRule::kMainChainBisector:
    return "MainChainBisector";
  default:
    return "Unknown";
  }
}

std::vector<wire::core::CableTemplateId> SortedCableTemplateIds(const CoreState& state) {
  std::vector<wire::core::CableTemplateId> ids;
  ids.reserve(state.view().cable_templates().size());
  for (const auto& [id, _] : state.view().cable_templates()) {
    ids.push_back(id);
  }
  std::sort(ids.begin(), ids.end());
  return ids;
}

void LoadCableTemplateState(const CoreState& state, ViewerUiState& ui_state, wire::core::CableTemplateId id) {
  const auto it = state.view().cable_templates().find(id);
  if (it == state.view().cable_templates().end()) {
    return;
  }
  ui_state.selected_cable_template_id = id;
  ui_state.cable_template_name = it->second.name;
  ui_state.cable_outer_diameter = it->second.outer_diameter_m;
  ui_state.cable_bend_stiffness = it->second.bend_stiffness;
  ui_state.cable_min_bend_radius = it->second.min_bend_radius_m;
  ui_state.cable_material_style = static_cast<int>(it->second.material_style);
  ui_state.cable_requires_insulator = it->second.requires_insulator;
  ui_state.cable_sag_factor = it->second.sag_factor;
  ui_state.cable_slack_factor = it->second.slack_factor;
  ui_state.cable_continuity_policy = static_cast<int>(it->second.continuity_policy);
}

void LoadBundleTemplateState(const CoreState& state, ViewerUiState& ui_state, wire::core::BundleKind id) {
  const auto it = state.view().bundle_templates().find(id);
  if (it == state.view().bundle_templates().end()) {
    return;
  }
  ui_state.selected_bundle_template_id = id;
  ui_state.bundle_template_cable_template_id = it->second.cable_template_id;
  ui_state.bundle_template_default_layer = static_cast<int>(it->second.default_layer);
  ui_state.bundle_template_allow_mirror = it->second.allow_mirror;
  ui_state.bundle_template_allow_midair_node = it->second.allow_midair_node;
  ui_state.bundle_template_allow_midair_branch = it->second.allow_midair_branch;
  ui_state.bundle_template_support_style = static_cast<int>(it->second.support_style);
  ui_state.bundle_template_branch_policy = static_cast<int>(it->second.branch_policy);
  ui_state.bundle_template_continuity_policy = static_cast<int>(it->second.continuity_policy);
}

void DrawObjectList(ViewerUiState& ui_state, const char* header, SelectedType type, const std::vector<ObjectId>& ids,
                    std::function<std::string(ObjectId)> make_label) {
  if (!ImGui::CollapsingHeader(header, ImGuiTreeNodeFlags_DefaultOpen)) {
    return;
  }

  for (ObjectId id : ids) {
    const std::string label = make_label(id);
    const bool is_selected = SelectionContains(ui_state, type, id);
    if (ImGui::Selectable(label.c_str(), is_selected)) {
      SetPrimarySelection(ui_state, type, id);
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

  const auto view = state.view();
  const auto& edit = view.edit_state();
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
    ImGui::Text("sharpOrientationApplied: %s", pole->context.sharp_orientation_applied ? "true" : "false");
    ImGui::Text("sharpTheta: %.2f", pole->context.sharp_theta_deg);
    ImGui::Text("sharpBisector: %.3f %.3f %.3f", pole->context.sharp_bisector_dir.x, pole->context.sharp_bisector_dir.y,
                pole->context.sharp_bisector_dir.z);
    ImGui::Text("sharpSideDir: %.3f %.3f %.3f", pole->context.sharp_side_dir.x, pole->context.sharp_side_dir.y,
                pole->context.sharp_side_dir.z);
    ImGui::Text("flip180: %s", pole->orientation_control.flip_180 ? "true" : "false");
    ImGui::Text("manualYawOverride: %s", pole->orientation_control.manual_yaw_override ? "true" : "false");
    if (const auto it = view.pole_orientation_debug_records().find(pole->id);
        it != view.pole_orientation_debug_records().end()) {
      const auto& orientation = it->second;
      ImGui::Text("forwardRule: %s", PoleForwardRuleLabel(orientation.rule));
      ImGui::Text("forwardDir: %.3f %.3f %.3f", orientation.adopted_forward.x, orientation.adopted_forward.y,
                  orientation.adopted_forward.z);
      ImGui::Text("mainNeighbors: %llu / %llu",
                  static_cast<unsigned long long>(orientation.primary_neighbor_id),
                  static_cast<unsigned long long>(orientation.secondary_neighbor_id));
    }
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
    ImGui::Text("Generated: %s", span->generation.generated ? "true" : "false");
    ImGui::Text("Gen Session: %llu", static_cast<unsigned long long>(span->generation.generation_session_id));
    ImGui::Text("GeneratedByRule: %s", span->generated_by_rule ? "true" : "false");
    ImGui::Text("PlacementContext: %s", ContextLabel(span->placement_context));
    ImGui::Text("placementOverride: %s", span->placement_override_flag ? "true" : "false");
    ImGui::Text("orientationOverride: %s", span->orientation_override_flag ? "true" : "false");
    const auto* curve = state.find_curve_cache(span->id);
    if (curve != nullptr) {
      ImGui::Text("curveSamples: %d", static_cast<int>(curve->points.size()));
      ImGui::Text("curveLength: %.2f", curve->detail.Length());
      ImGui::Text("arcSamples: %d", static_cast<int>(curve->detail.arc_length_table.size()));
      ImGui::Text("continuity: %s (%s)", DetailCurveContinuityModeLabel(curve->detail.quality.adopted_continuity),
                  DetailCurveContinuityReasonLabel(curve->detail.quality.continuity_reason));
      ImGui::Text("continuityPolicy: %s attemptedG2=%s degraded=%s",
                  ContinuityPolicyLabel(curve->detail.quality.requested_policy),
                  curve->detail.quality.attempted_g2 ? "true" : "false",
                  curve->detail.quality.degraded_to_g1 ? "true" : "false");
      ImGui::Text("handles: %.2f / %.2f scale=%.2f", curve->detail.quality.handle_length_start_m,
                  curve->detail.quality.handle_length_end_m, curve->detail.quality.tangent_scale);
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
    const auto* runtime_state = state.view().find_span_runtime_state(span->id);
    if (runtime_state != nullptr) {
      ImGui::Separator();
      ImGui::Text("dataVersion: %llu", static_cast<unsigned long long>(runtime_state->data_version));
      ImGui::Text("geometryVersion: %llu", static_cast<unsigned long long>(runtime_state->geometry_version));
      ImGui::Text("boundsVersion: %llu", static_cast<unsigned long long>(runtime_state->bounds_version));
      ImGui::Text("renderVersion: %llu", static_cast<unsigned long long>(runtime_state->render_version));
      ImGui::Text("dirtyBits: %s", DirtyBitsToText(runtime_state->dirty_bits).c_str());
    }
    for (const auto& assignment : view.last_lane_assignments()) {
      const bool same_forward = assignment.bundle_id == span->bundle_id && assignment.pole_a_id == span->endpoint_node_a_id &&
                                assignment.pole_b_id == span->endpoint_node_b_id;
      const bool same_reverse = assignment.bundle_id == span->bundle_id && assignment.pole_a_id == span->endpoint_node_b_id &&
                                assignment.pole_b_id == span->endpoint_node_a_id;
      if (!same_forward && !same_reverse) {
        continue;
      }
      ImGui::Separator();
      ImGui::Text("flowKind: %s", BackboneFlowKindLabel(assignment.flow_kind));
      ImGui::Text("flowRule: %s", BackboneFlowDecisionRuleLabel(assignment.flow_decision_rule));
      ImGui::Text("branchSupport: %s downOffset=%.2f", assignment.uses_branch_support ? "true" : "false",
                  assignment.branch_down_offset_m);
      ImGui::Text("mirror: %s flippedPrev=%s turn=%.2f", assignment.mirrored ? "true" : "false",
                  assignment.flipped_from_previous ? "true" : "false", assignment.turn_angle_deg);
      break;
    }
    if (const auto* visual = state.view().find_span_visual_cache(span->id); visual != nullptr) {
      ImGui::Text("branchSupportPlacements: %d", static_cast<int>(visual->branch_supports.size()));
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
    ImGui::Text("displayOffset: %.3f", attachment->display_offset_m);
    return;
  }
  case SelectedType::kSupportNode: {
    const wire::core::BackboneResult backbone = state.BuildBackboneResult();
    const auto it =
        std::find_if(backbone.nodes.begin(), backbone.nodes.end(),
                     [&](const wire::core::SupportNode& node) { return node.node_id == ui_state.selected_id; });
    if (it == backbone.nodes.end()) {
      ImGui::TextUnformatted("Selected support node is missing");
      return;
    }
    ImGui::Text("Type: SupportNode");
    ImGui::Text("ID: %llu", static_cast<unsigned long long>(it->node_id));
    ImGui::Text("SupportKind: %s", SupportKindLabel(it->support_kind));
    ImGui::Text("PoleId: %llu", static_cast<unsigned long long>(it->pole_id));
    ImGui::Text("Pos: %.2f %.2f %.2f", it->position.x, it->position.y, it->position.z);
    ImGui::Text("pathPointIndex: %d", it->path_point_index);
    ImGui::Text("sourceEdge: %s (%llu -> %llu @ %.3f)", it->has_source_edge ? "true" : "false",
                static_cast<unsigned long long>(it->source_edge_node_a_id),
                static_cast<unsigned long long>(it->source_edge_node_b_id), it->source_edge_t);
    ImGui::Text("hasTangentHint: %s", it->has_tangent_hint ? "true" : "false");
    if (it->has_tangent_hint) {
      ImGui::Text("tangentHint: %.3f %.3f %.3f", it->tangent_hint.x, it->tangent_hint.y, it->tangent_hint.z);
    }
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
  const auto view = state.view();
  ImGui::Separator();
  ImGui::TextUnformatted("Edit Selected");
  ImGui::InputDouble("Edit X", &ui_state.edit_x);
  ImGui::InputDouble("Edit Y", &ui_state.edit_y);
  ImGui::InputDouble("Edit Z", &ui_state.edit_z);

  if (ui_state.selected_type == SelectedType::kPole) {
    if (ImGui::Button("Load Pole Pos")) {
      const auto* pole = view.poles().find(ui_state.selected_id);
      if (pole != nullptr) {
        ui_state.edit_x = pole->world_transform.position.x;
        ui_state.edit_y = pole->world_transform.position.y;
        ui_state.edit_z = pole->world_transform.position.z;
      }
    }
    ImGui::SameLine();
    if (ImGui::Button("Move Pole")) {
      const auto* pole = view.poles().find(ui_state.selected_id);
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
      const auto* port = view.ports().find(ui_state.selected_id);
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
      const auto* anchor = view.anchors().find(ui_state.selected_id);
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
        SetPrimarySelection(ui_state, SelectedType::kPort, result.value.new_port_id);
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
        ClearSelection(ui_state);
      }
    }
  } else {
    ImGui::TextUnformatted("Select Pole/Port/Anchor/Span to edit");
  }
}

void DrawPlacementModePanel(CoreState& state, ViewerUiState& ui_state) {
  const auto& all_categories = AllCategories();
  ImGui::TextUnformatted("Placement");
  const auto type_ids = SortedPoleTypeIds(state);
  if (type_ids.empty()) {
    ImGui::TextUnformatted("No PoleType available");
    return;
  }
  const std::size_t type_index = ClampedTypeIndex(ui_state.placement_pole_type_index, type_ids.size());
  ui_state.placement_pole_type_index = static_cast<int>(type_index);
  const wire::core::PoleTypeId selected_type_id = type_ids[type_index];
  const auto type_it = state.view().pole_types().find(selected_type_id);
  const std::string selected_type_name =
      (type_it != state.view().pole_types().end()) ? type_it->second.name : std::to_string(selected_type_id);

  if (ImGui::BeginCombo("PoleType", selected_type_name.c_str())) {
    for (std::size_t i = 0; i < type_ids.size(); ++i) {
      const auto it = state.view().pole_types().find(type_ids[i]);
      const std::string label = (it != state.view().pole_types().end()) ? it->second.name : std::to_string(type_ids[i]);
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
    SetPrimarySelection(ui_state, SelectedType::kPole, add_pole_result.value);
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
  const auto road_type_it = state.view().pole_types().find(road_type_id);
  const std::string road_type_name =
      (road_type_it != state.view().pole_types().end()) ? road_type_it->second.name : std::to_string(road_type_id);
  if (ImGui::BeginCombo("Road PoleType", road_type_name.c_str())) {
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

  const int road_category_index =
      std::clamp(ui_state.road_category_index, 0, static_cast<int>(all_categories.size() - 1));
  ui_state.road_category_index = road_category_index;
  if (ImGui::BeginCombo("Road Category", CategoryLabel(all_categories[road_category_index]))) {
    for (int i = 0; i < static_cast<int>(all_categories.size()); ++i) {
      const bool selected = (i == road_category_index);
      if (ImGui::Selectable(CategoryLabel(all_categories[i]), selected)) {
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
    wire::core::BackboneSpec request{};
    request.path.polyline = road.polyline;
    request.interval_m = ui_state.road_interval;
    request.pole_type_id = type_ids[ui_state.road_pole_type_index];
    wire::core::BackboneBundleSpec bundle{};
    bundle.bundle_template_id = BundleTemplateForCategory(all_categories[ui_state.road_category_index]);
    request.bundles.push_back(bundle);
    if (ExecuteBackboneRequest(state, ui_state, request, false, false, "Generated road", "Generate road line failed")) {
      ui_state.road_id += 1;
    }
  }
  ImGui::Text("Last generated poles: %d", ui_state.last_generated_poles);
  ImGui::Text("Last generated spans: %d", ui_state.last_generated_spans);
  ImGui::Text("Last generation session: %llu", static_cast<unsigned long long>(ui_state.last_generation_session));
}

void DrawConnectionModePanel(CoreState& state, ViewerUiState& ui_state) {
  const auto& all_categories = AllCategories();
  const auto& all_contexts = AllConnectionContexts();
  ImGui::TextUnformatted("Connection (Pole -> Pole)");
  ImGui::InputScalar("Source PoleId", ImGuiDataType_U64, &ui_state.connect_pole_a_id);
  ImGui::InputScalar("Target PoleId", ImGuiDataType_U64, &ui_state.connect_pole_b_id);

  const int category_index = std::clamp(ui_state.connect_category, 0, static_cast<int>(all_categories.size() - 1));
  ui_state.connect_category = category_index;
  const int context_index = std::clamp(ui_state.connect_context, 0, static_cast<int>(all_contexts.size() - 1));
  ui_state.connect_context = context_index;
  if (ImGui::BeginCombo("Category", CategoryLabel(all_categories[category_index]))) {
    for (int i = 0; i < static_cast<int>(all_categories.size()); ++i) {
      const bool selected = (i == category_index);
      if (ImGui::Selectable(CategoryLabel(all_categories[i]), selected)) {
        ui_state.connect_category = i;
      }
      if (selected) {
        ImGui::SetItemDefaultFocus();
      }
    }
    ImGui::EndCombo();
  }
  if (ImGui::BeginCombo("Context", ContextLabel(all_contexts[context_index]))) {
    for (int i = 0; i < static_cast<int>(all_contexts.size()); ++i) {
      const bool selected = (i == context_index);
      if (ImGui::Selectable(ContextLabel(all_contexts[i]), selected)) {
        ui_state.connect_context = i;
      }
      if (selected) {
        ImGui::SetItemDefaultFocus();
      }
    }
    ImGui::EndCombo();
  }

  if (ImGui::Button("Connect Pole -> Pole")) {
    const int selected_context_index = std::clamp(ui_state.connect_context, 0, static_cast<int>(all_contexts.size() - 1));
    const wire::core::ConnectionCategory selected_category = all_categories[ui_state.connect_category];
    wire::core::CoreState::AddConnectionByPoleOptions options{};
    options.connection_context = all_contexts[selected_context_index];
    options.use_bundle_template = true;
    switch (selected_category) {
    case wire::core::ConnectionCategory::kHighVoltage:
      options.bundle_template_id = wire::core::BundleKind::kHighVoltage;
      break;
    case wire::core::ConnectionCategory::kCommunication:
      options.bundle_template_id = wire::core::BundleKind::kCommunication;
      break;
    case wire::core::ConnectionCategory::kOptical:
      options.bundle_template_id = wire::core::BundleKind::kOptical;
      break;
    case wire::core::ConnectionCategory::kLowVoltage:
    case wire::core::ConnectionCategory::kDrop:
    default:
      options.bundle_template_id = wire::core::BundleKind::kLowVoltage;
      break;
    }
    const auto result = state.AddConnectionByPole(ui_state.connect_pole_a_id, ui_state.connect_pole_b_id,
                                                   selected_category, options);
    if (!result.ok) {
      ui_state.last_error = result.error;
      PushLog(ui_state, "Connect Pole->Pole failed");
    } else {
      ui_state.last_error.clear();
      SetPrimarySelection(ui_state, SelectedType::kSpan, result.value.span_id);
      PushLog(ui_state, "Connected span=" + std::to_string(result.value.span_id) +
                            " portA=" + std::to_string(result.value.port_a_id) +
                            " slotA=" + std::to_string(result.value.slot_a_id) +
                            " portB=" + std::to_string(result.value.port_b_id) +
                            " slotB=" + std::to_string(result.value.slot_b_id));
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
    const auto type_it = state.view().pole_types().find(type_ids[idx]);
    const std::string selected_name =
        (type_it != state.view().pole_types().end()) ? type_it->second.name : std::to_string(type_ids[idx]);
    if (ImGui::BeginCombo("Set PoleType", selected_name.c_str())) {
      for (std::size_t i = 0; i < type_ids.size(); ++i) {
        const auto it = state.view().pole_types().find(type_ids[i]);
        const std::string label = (it != state.view().pole_types().end()) ? it->second.name : std::to_string(type_ids[i]);
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
        ImGui::Text("templateSlot=%d cat=%s layer=%d side=%s role=%s used=0 [empty]", slot.slot_id,
                    CategoryLabel(slot.category), slot.layer, SlotSideLabel(slot.side), SlotRoleLabel(slot.role));
        continue;
      }
      const auto* port = it->second;
      const auto usage_it = state.view().connection_index().spans_by_port.find(port->id);
      const int usage =
          (usage_it == state.view().connection_index().spans_by_port.end()) ? 0 : static_cast<int>(usage_it->second.size());
      ImGui::Text("templateSlot=%d cat=%s layer=%d side=%s role=%s used=%d -> %s", slot.slot_id, CategoryLabel(slot.category),
                  slot.layer, SlotSideLabel(slot.side), SlotRoleLabel(slot.role), usage, port->display_id.c_str());
    }
  } else {
    for (const auto* port : detail.owned_ports) {
      const auto it = state.view().connection_index().spans_by_port.find(port->id);
      const int usage = (it == state.view().connection_index().spans_by_port.end()) ? 0 : static_cast<int>(it->second.size());
      ImGui::Text("%s cat=%s templateSlot=%d used=%d", port->display_id.c_str(), CategoryLabel(port->category),
                  port->source_slot_id, usage);
    }
  }

  ImGui::Separator();
  ImGui::Text("Anchors: %d", static_cast<int>(detail.owned_anchors.size()));
  for (const auto* anchor : detail.owned_anchors) {
    ImGui::Text("%s templateSlot=%d", anchor->display_id.c_str(), anchor->source_slot_id);
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
      SetPrimarySelection(ui_state, SelectedType::kPort, result.value);
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
      SetPrimarySelection(ui_state, SelectedType::kSpan, result.value);
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
  const auto view = state.view();
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
  ImGui::Text("Poles:%d  Ports:%d  Spans:%d  Bundles:%d", static_cast<int>(view.poles().size()),
              static_cast<int>(view.ports().size()), static_cast<int>(view.spans().size()),
              static_cast<int>(view.bundles().size()));
  ImGui::SameLine();
  ImGui::Text("|  Mode: %s", ModeLabel(ui_state.mode));
  ImGui::SameLine();
  ImGui::Text("|  FOV %.1f", ui_state.camera_fov_deg);
  ImGui::SameLine();
  ImGui::Text("|  Walk %s", ui_state.camera_walk_mode ? "ON" : "OFF");
  if (ui_state.selected_type == SelectedType::kPole) {
    if (const auto* pole = view.poles().find(ui_state.selected_id); pole != nullptr) {
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
  const auto view = state.view();
  if (ui_state.selected_type == SelectedType::kPole &&
      ui_state.selected_id != wire::core::kInvalidObjectId) {
    if (const auto* pole = view.poles().find(ui_state.selected_id); pole != nullptr) {
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

  ImGui::Separator();
  ImGui::Text("Selection Count: %d", static_cast<int>(ui_state.selection_items.size()));
  ImGui::Text("Poles=%d Midair=%d Spans=%d", SelectionCountByType(ui_state, SelectedType::kPole),
              SelectionCountByType(ui_state, SelectedType::kSupportNode),
              SelectionCountByType(ui_state, SelectedType::kSpan));
  if (ImGui::Button("Clear Selection")) {
    ClearSelection(ui_state);
  }
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
  const auto view = state.view();
  const wire::core::BackboneResult backbone = state.BuildBackboneResult();
  DrawObjectList(
      ui_state, "Poles", SelectedType::kPole,
      [&]() {
        std::vector<ObjectId> ids;
        ids.reserve(view.poles().size());
        for (const auto& pole : view.poles().items()) {
          ids.push_back(pole.id);
        }
        return ids;
      }(),
      [&](ObjectId id) {
        const auto* pole = view.poles().find(id);
        if (pole == nullptr) {
          return std::to_string(id);
        }
        return pole->display_id + " " + pole->name;
      });

  DrawObjectList(
      ui_state, "Ports", SelectedType::kPort,
      [&]() {
        std::vector<ObjectId> ids;
        ids.reserve(view.ports().size());
        for (const auto& port : view.ports().items()) {
          ids.push_back(port.id);
        }
        return ids;
      }(),
      [&](ObjectId id) {
        const auto* port = view.ports().find(id);
        if (port == nullptr) {
          return std::to_string(id);
        }
        return port->display_id;
      });

  DrawObjectList(
      ui_state, "Spans", SelectedType::kSpan,
      [&]() {
        std::vector<ObjectId> ids;
        ids.reserve(view.spans().size());
        for (const auto& span : view.spans().items()) {
          ids.push_back(span.id);
        }
        return ids;
      }(),
      [&](ObjectId id) {
        const auto* span = view.spans().find(id);
        if (span == nullptr) {
          return std::to_string(id);
        }
        return span->display_id;
      });

  DrawObjectList(
      ui_state, "Midair SupportNodes", SelectedType::kSupportNode,
      [&]() {
        std::vector<ObjectId> ids;
        for (const wire::core::SupportNode& node : backbone.nodes) {
          if (node.support_kind == wire::core::SupportKind::kMidair) {
            ids.push_back(node.node_id);
          }
        }
        return ids;
      }(),
      [&](ObjectId id) {
        const auto it = std::find_if(backbone.nodes.begin(), backbone.nodes.end(),
                                     [id](const wire::core::SupportNode& node) { return node.node_id == id; });
        if (it == backbone.nodes.end()) {
          return std::to_string(id);
        }
        return std::string("midair ") + std::to_string(static_cast<unsigned long long>(it->node_id));
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
    const auto& gs = state.view().geometry_settings();
    ui_state.geometry_samples = gs.curve_samples;
    ui_state.geometry_sag_enabled = gs.sag_enabled;
    ui_state.geometry_sag_factor = gs.sag_factor;
    ui_state.geometry_pole_clearance = gs.pole_clearance_m;
    ui_state.geometry_settings_loaded = true;
  }
  if (!ui_state.layout_settings_loaded) {
    const auto& ls = state.view().layout_settings();
    ui_state.layout_angle_correction_enabled = ls.angle_correction_enabled;
    ui_state.layout_corner_threshold_deg = ls.corner_threshold_deg;
    ui_state.layout_min_side_scale = ls.min_side_scale;
    ui_state.layout_max_side_scale = ls.max_side_scale;
    ui_state.layout_settings_loaded = true;
  }
  if (!ui_state.visual_settings_loaded) {
    const auto& vs = state.view().visual_settings();
    ui_state.visual_enable_support_structures = vs.enable_support_structures;
    ui_state.visual_enable_insulators = vs.enable_insulators;
    ui_state.visual_support_center_threshold = vs.support_center_threshold_m;
    ui_state.visual_support_arm_extra = vs.support_arm_extra_m;
    ui_state.visual_insulator_radius = vs.insulator_radius_m;
    ui_state.visual_insulator_length = vs.insulator_length_m;
    ui_state.visual_settings_loaded = true;
  }
  if (!ui_state.cable_template_loaded) {
    for (const auto& [id, tpl] : state.view().cable_templates()) {
      (void)tpl;
      LoadCableTemplateState(state, ui_state, id);
      break;
    }
    ui_state.cable_template_loaded = true;
  }
  if (!ui_state.bundle_template_loaded) {
    for (const auto& [id, _] : state.view().bundle_templates()) {
      LoadBundleTemplateState(state, ui_state, id);
      break;
    }
    ui_state.bundle_template_loaded = true;
  }

  const auto& recalc = state.view().last_recalc_stats();
  ImGui::Text("Dirty T/G/B/R/X: %d / %d / %d / %d / %d",
              static_cast<int>(state.view().dirty_queue().topology_dirty_span_ids.size()),
              static_cast<int>(state.view().dirty_queue().geometry_dirty_span_ids.size()),
              static_cast<int>(state.view().dirty_queue().bounds_dirty_span_ids.size()),
              static_cast<int>(state.view().dirty_queue().render_dirty_span_ids.size()),
              static_cast<int>(state.view().dirty_queue().raycast_dirty_span_ids.size()));
  ImGui::Text("Last Recalc total=%d geom=%d bounds=%d render=%d", static_cast<int>(recalc.total_processed()),
              static_cast<int>(recalc.geometry_processed), static_cast<int>(recalc.bounds_processed),
              static_cast<int>(recalc.render_processed));
  ImGui::Checkbox("Auto Recalc", &ui_state.auto_recalc);
  ImGui::SameLine();
  if (ImGui::Button("Run Recalc")) {
    wire::core::CommitOptions options{};
    options.run_recalc = true;
    options.run_validate = false;
    const auto stats = state.Commit(options).recalc_stats;
    PushLog(ui_state, "Recalc processed=" + std::to_string(stats.total_processed()));
  }

  if (ImGui::CollapsingHeader("Camera", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::SliderFloat("FOV", &ui_state.camera_fov_deg, 20.0f, 110.0f, "%.1f deg");
    ImGui::SliderFloat("Walk Speed", &ui_state.camera_walk_speed, 0.5f, 80.0f, "%.1f");
    ImGui::SliderFloat("Mouse Sensitivity", &ui_state.camera_mouse_sensitivity, 0.001f, 0.02f, "%.4f");
    if (ImGui::Button(ui_state.camera_walk_mode ? "Stop Walk (Shift+F)" : "Start Walk (Shift+F)")) {
      ui_state.camera_walk_mode = !ui_state.camera_walk_mode;
      if (ui_state.camera_walk_mode) {
        DisableCursor();
        PushLog(ui_state, "Camera: Walk mode ON (WASD + mouse, Q/E up/down, Esc exit)");
      } else {
        EnableCursor();
        PushLog(ui_state, "Camera: Walk mode OFF");
      }
    }
    ImGui::TextUnformatted("Alt+LMB: set orbit pivot on draw plane");
    ImGui::TextUnformatted("[ / ]: FOV down/up");
  }

  if (ImGui::CollapsingHeader("Geometry/Layout", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::SliderInt("Curve Samples", &ui_state.geometry_samples, 2, 64);
    ImGui::Checkbox("Sag Enabled", &ui_state.geometry_sag_enabled);
    ImGui::InputDouble("Sag Factor", &ui_state.geometry_sag_factor, 0.005, 0.01, "%.4f");
    ImGui::InputDouble("Pole Clearance", &ui_state.geometry_pole_clearance, 0.005, 0.01, "%.3f");
    if (ImGui::Button("Apply Geometry")) {
      wire::core::GeometrySettings settings{};
      settings.curve_samples = ui_state.geometry_samples;
      settings.sag_enabled = ui_state.geometry_sag_enabled;
      settings.sag_factor = ui_state.geometry_sag_factor;
      settings.pole_clearance_m = ui_state.geometry_pole_clearance;
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

  if (ImGui::CollapsingHeader("Pole Tilt / Templates", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::InputDouble("Tilt X (deg)", &ui_state.tilt_all_x_deg, 0.5, 1.0, "%.2f");
    ImGui::InputDouble("Tilt Y (deg)", &ui_state.tilt_all_y_deg, 0.5, 1.0, "%.2f");
    {
      std::vector<ObjectId> selected_pole_ids{};
      selected_pole_ids.reserve(ui_state.selection_items.size());
      for (const SelectionItem& item : ui_state.selection_items) {
        if (item.type == SelectedType::kPole) {
          selected_pole_ids.push_back(item.id);
        }
      }
      if (ImGui::Button("Apply Tilt To Selected Poles")) {
        if (selected_pole_ids.empty()) {
          PushLog(ui_state, "No poles selected");
        } else {
          const auto tilt = state.ApplyPoleTilt(selected_pole_ids, ui_state.tilt_all_x_deg, ui_state.tilt_all_y_deg);
          if (!tilt.ok) {
            ui_state.last_error = tilt.error;
            PushLog(ui_state, "ApplyPoleTilt(selected) failed");
          } else {
            ui_state.last_error.clear();
            PushLog(ui_state, "Applied tilt to selected poles count=" +
                                  std::to_string(static_cast<unsigned long long>(selected_pole_ids.size())));
          }
        }
      }
      ImGui::SameLine();
      ImGui::Text("selected=%d", static_cast<int>(selected_pole_ids.size()));
    }
    ImGui::Checkbox("Select Poles", &ui_state.selection_include_poles);
    ImGui::SameLine();
    ImGui::Checkbox("Select Midair", &ui_state.selection_include_midair_nodes);
    ImGui::SameLine();
    ImGui::Checkbox("Select Spans", &ui_state.selection_include_spans);
    ImGui::TextUnformatted("Viewport: LMB click select, Shift+LMB drag box select");
    if (ImGui::Button("Apply Tilt To All Poles")) {
      const auto tilt = state.ApplyPoleTilt({}, ui_state.tilt_all_x_deg, ui_state.tilt_all_y_deg);
      if (!tilt.ok) {
        ui_state.last_error = tilt.error;
        PushLog(ui_state, "ApplyPoleTilt failed");
      } else {
        ui_state.last_error.clear();
        PushLog(ui_state, "Applied tilt to all poles");
      }
    }
    ImGui::SameLine();
    if (ImGui::Button("Reset All Span Reference Lengths")) {
      const auto reset = state.ResetAllSpanReferenceLengths(true);
      if (!reset.ok) {
        ui_state.last_error = reset.error;
        PushLog(ui_state, "ResetAllSpanReferenceLengths failed");
      } else {
        ui_state.last_error.clear();
        PushLog(ui_state, "Span reference lengths reset from current geometry");
      }
    }

    ImGui::Separator();
    const auto cable_ids = SortedCableTemplateIds(state);
    if (const auto it = state.view().cable_templates().find(ui_state.selected_cable_template_id);
        it != state.view().cable_templates().end()) {
      if (ImGui::BeginCombo("Cable Template", it->second.name.c_str())) {
        for (wire::core::CableTemplateId id : cable_ids) {
          const auto jt = state.view().cable_templates().find(id);
          if (jt == state.view().cable_templates().end()) {
            continue;
          }
          const bool selected = (id == ui_state.selected_cable_template_id);
          if (ImGui::Selectable(jt->second.name.c_str(), selected)) {
            LoadCableTemplateState(state, ui_state, id);
          }
          if (selected) {
            ImGui::SetItemDefaultFocus();
          }
        }
        ImGui::EndCombo();
      }
    }
    ImGui::Text("Cable Name: %s", ui_state.cable_template_name.c_str());
    ImGui::InputDouble("Cable Outer Diameter", &ui_state.cable_outer_diameter, 0.001, 0.005, "%.4f");
    ImGui::InputDouble("Cable Bend Stiffness", &ui_state.cable_bend_stiffness, 0.1, 0.2, "%.3f");
    ImGui::InputDouble("Cable Min Bend Radius", &ui_state.cable_min_bend_radius, 0.01, 0.05, "%.3f");
    const auto selected_material = static_cast<wire::core::CableMaterialStyleKind>(ui_state.cable_material_style);
    if (ImGui::BeginCombo("Cable Material", CableMaterialStyleLabel(selected_material))) {
      for (int raw = static_cast<int>(wire::core::CableMaterialStyleKind::kGeneric);
           raw <= static_cast<int>(wire::core::CableMaterialStyleKind::kOptical); ++raw) {
        const auto material = static_cast<wire::core::CableMaterialStyleKind>(raw);
        const bool selected = (raw == ui_state.cable_material_style);
        if (ImGui::Selectable(CableMaterialStyleLabel(material), selected)) {
          ui_state.cable_material_style = raw;
        }
        if (selected) {
          ImGui::SetItemDefaultFocus();
        }
      }
      ImGui::EndCombo();
    }
    ImGui::Checkbox("Cable Requires Insulator", &ui_state.cable_requires_insulator);
    ImGui::InputDouble("Cable Sag Factor", &ui_state.cable_sag_factor, 0.005, 0.01, "%.4f");
    ImGui::InputDouble("Cable Slack Factor", &ui_state.cable_slack_factor, 0.005, 0.01, "%.4f");
    const auto selected_cable_continuity =
        static_cast<wire::core::CableContinuityPolicyHint>(ui_state.cable_continuity_policy);
    if (ImGui::BeginCombo("Cable Continuity", ContinuityPolicyLabel(selected_cable_continuity))) {
      for (int raw = static_cast<int>(wire::core::CableContinuityPolicyHint::kAuto);
           raw <= static_cast<int>(wire::core::CableContinuityPolicyHint::kPreferG2); ++raw) {
        const auto policy = static_cast<wire::core::CableContinuityPolicyHint>(raw);
        const bool selected = (raw == ui_state.cable_continuity_policy);
        if (ImGui::Selectable(ContinuityPolicyLabel(policy), selected)) {
          ui_state.cable_continuity_policy = raw;
        }
        if (selected) {
          ImGui::SetItemDefaultFocus();
        }
      }
      ImGui::EndCombo();
    }
    if (ImGui::Button("Apply Cable Template")) {
      const auto it = state.view().cable_templates().find(ui_state.selected_cable_template_id);
      if (it == state.view().cable_templates().end()) {
        ui_state.last_error = "selected cable template missing";
      } else {
        wire::core::CableTemplate tpl = it->second;
        tpl.outer_diameter_m = ui_state.cable_outer_diameter;
        tpl.bend_stiffness = ui_state.cable_bend_stiffness;
        tpl.min_bend_radius_m = ui_state.cable_min_bend_radius;
        tpl.material_style = static_cast<wire::core::CableMaterialStyleKind>(ui_state.cable_material_style);
        tpl.requires_insulator = ui_state.cable_requires_insulator;
        tpl.sag_factor = ui_state.cable_sag_factor;
        tpl.slack_factor = ui_state.cable_slack_factor;
        tpl.continuity_policy =
            static_cast<wire::core::CableContinuityPolicyHint>(ui_state.cable_continuity_policy);
        const auto apply = state.UpdateCableTemplate(tpl, ui_state.preferred_visible_span_ids);
        if (!apply.ok) {
          ui_state.last_error = apply.error;
          PushLog(ui_state, "UpdateCableTemplate failed");
        } else {
          ui_state.last_error.clear();
          LoadCableTemplateState(state, ui_state, tpl.id);
          PushLog(ui_state, "Cable template updated; visible-first=" +
                                    std::to_string(ui_state.preferred_visible_span_count) + " dirty spans=" +
                                    std::to_string(apply.change_set.dirty_span_ids.size()));
        }
      }
    }
    ImGui::Text("Preferred Visible Spans: %d", ui_state.preferred_visible_span_count);

    ImGui::Separator();
    const auto bundle_ids = SortedBundleTemplateKinds(state);
    if (const auto it = state.view().bundle_templates().find(ui_state.selected_bundle_template_id);
        it != state.view().bundle_templates().end()) {
      if (ImGui::BeginCombo("Bundle Template", it->second.name.c_str())) {
        for (wire::core::BundleKind id : bundle_ids) {
          const auto jt = state.view().bundle_templates().find(id);
          if (jt == state.view().bundle_templates().end()) {
            continue;
          }
          const bool selected = (id == ui_state.selected_bundle_template_id);
          if (ImGui::Selectable(jt->second.name.c_str(), selected)) {
            LoadBundleTemplateState(state, ui_state, id);
          }
          if (selected) {
            ImGui::SetItemDefaultFocus();
          }
        }
        ImGui::EndCombo();
      }
      ImGui::Text("Category: %s", CategoryLabel(it->second.category));
      ImGui::Text("Count Rule: %s", it->second.count_rule == wire::core::BundleCountRuleKind::kFixed ? "Fixed" : "Range");
      if (it->second.count_rule == wire::core::BundleCountRuleKind::kFixed) {
        ImGui::Text("Fixed Count: %d", it->second.fixed_count);
      } else {
        ImGui::Text("Count Range: %d..%d default=%d", it->second.min_count, it->second.max_count,
                    it->second.default_count);
      }
      const auto selected_bundle_cable_it =
          state.view().cable_templates().find(ui_state.bundle_template_cable_template_id);
      const char* selected_bundle_cable_label =
          selected_bundle_cable_it != state.view().cable_templates().end() ? selected_bundle_cable_it->second.name.c_str()
                                                                           : "(missing)";
      if (ImGui::BeginCombo("Bundle Cable Template", selected_bundle_cable_label)) {
        for (wire::core::CableTemplateId id : cable_ids) {
          const auto cable_it = state.view().cable_templates().find(id);
          if (cable_it == state.view().cable_templates().end()) {
            continue;
          }
          const bool selected = (id == ui_state.bundle_template_cable_template_id);
          if (ImGui::Selectable(cable_it->second.name.c_str(), selected)) {
            ui_state.bundle_template_cable_template_id = id;
          }
          if (selected) {
            ImGui::SetItemDefaultFocus();
          }
        }
        ImGui::EndCombo();
      }
      const auto selected_layer = static_cast<wire::core::SpanLayer>(ui_state.bundle_template_default_layer);
      if (ImGui::BeginCombo("Bundle Default Layer", SpanLayerLabel(selected_layer))) {
        constexpr wire::core::SpanLayer kLayers[] = {
            wire::core::SpanLayer::kHighVoltage, wire::core::SpanLayer::kLowVoltage,
            wire::core::SpanLayer::kCommunication, wire::core::SpanLayer::kOptical};
        for (wire::core::SpanLayer layer : kLayers) {
          const bool selected = (layer == selected_layer);
          if (ImGui::Selectable(SpanLayerLabel(layer), selected)) {
            ui_state.bundle_template_default_layer = static_cast<int>(layer);
          }
          if (selected) {
            ImGui::SetItemDefaultFocus();
          }
        }
        ImGui::EndCombo();
      }
      ImGui::Checkbox("Bundle Allow Mirror", &ui_state.bundle_template_allow_mirror);
      ImGui::Checkbox("Bundle Allow Midair Node", &ui_state.bundle_template_allow_midair_node);
      ImGui::Checkbox("Bundle Allow Midair Branch", &ui_state.bundle_template_allow_midair_branch);
      const auto selected_support_style =
          static_cast<wire::core::BundleSupportStyleHint>(ui_state.bundle_template_support_style);
      if (ImGui::BeginCombo("Bundle Support Style", BundleSupportStyleLabel(selected_support_style))) {
        for (int raw = static_cast<int>(wire::core::BundleSupportStyleHint::kAuto);
             raw <= static_cast<int>(wire::core::BundleSupportStyleHint::kSideStructurePreferred); ++raw) {
          const auto style = static_cast<wire::core::BundleSupportStyleHint>(raw);
          const bool selected = (raw == ui_state.bundle_template_support_style);
          if (ImGui::Selectable(BundleSupportStyleLabel(style), selected)) {
            ui_state.bundle_template_support_style = raw;
          }
          if (selected) {
            ImGui::SetItemDefaultFocus();
          }
        }
        ImGui::EndCombo();
      }
      const auto selected_branch_policy =
          static_cast<wire::core::BundleBranchPolicyHint>(ui_state.bundle_template_branch_policy);
      if (ImGui::BeginCombo("Bundle Branch Policy", BundleBranchPolicyLabel(selected_branch_policy))) {
        for (int raw = static_cast<int>(wire::core::BundleBranchPolicyHint::kAuto);
             raw <= static_cast<int>(wire::core::BundleBranchPolicyHint::kPreferExplicitBranch); ++raw) {
          const auto policy = static_cast<wire::core::BundleBranchPolicyHint>(raw);
          const bool selected = (raw == ui_state.bundle_template_branch_policy);
          if (ImGui::Selectable(BundleBranchPolicyLabel(policy), selected)) {
            ui_state.bundle_template_branch_policy = raw;
          }
          if (selected) {
            ImGui::SetItemDefaultFocus();
          }
        }
        ImGui::EndCombo();
      }
      const auto selected_bundle_continuity =
          static_cast<wire::core::CableContinuityPolicyHint>(ui_state.bundle_template_continuity_policy);
      if (ImGui::BeginCombo("Bundle Continuity", ContinuityPolicyLabel(selected_bundle_continuity))) {
        for (int raw = static_cast<int>(wire::core::CableContinuityPolicyHint::kAuto);
             raw <= static_cast<int>(wire::core::CableContinuityPolicyHint::kPreferG2); ++raw) {
          const auto policy = static_cast<wire::core::CableContinuityPolicyHint>(raw);
          const bool selected = (raw == ui_state.bundle_template_continuity_policy);
          if (ImGui::Selectable(ContinuityPolicyLabel(policy), selected)) {
            ui_state.bundle_template_continuity_policy = raw;
          }
          if (selected) {
            ImGui::SetItemDefaultFocus();
          }
        }
        ImGui::EndCombo();
      }
      if (ImGui::Button("Apply Bundle Template")) {
        wire::core::BundleTemplate tpl = it->second;
        tpl.cable_template_id = ui_state.bundle_template_cable_template_id;
        tpl.default_layer = static_cast<wire::core::SpanLayer>(ui_state.bundle_template_default_layer);
        tpl.allow_mirror = ui_state.bundle_template_allow_mirror;
        tpl.allow_midair_node = ui_state.bundle_template_allow_midair_node;
        tpl.allow_midair_branch = ui_state.bundle_template_allow_midair_branch;
        tpl.support_style = static_cast<wire::core::BundleSupportStyleHint>(ui_state.bundle_template_support_style);
        tpl.branch_policy = static_cast<wire::core::BundleBranchPolicyHint>(ui_state.bundle_template_branch_policy);
        tpl.continuity_policy =
            static_cast<wire::core::CableContinuityPolicyHint>(ui_state.bundle_template_continuity_policy);
        const auto apply = state.UpdateBundleTemplate(tpl);
        if (!apply.ok) {
          ui_state.last_error = apply.error;
          PushLog(ui_state, "UpdateBundleTemplate failed");
        } else {
          ui_state.last_error.clear();
          const auto& deps = state.view().template_dependency_state();
          if (!deps.bundles_requiring_regeneration.empty() || !deps.sessions_requiring_regeneration.empty()) {
            PushLog(ui_state, "Bundle template updated; regeneration required bundles=" +
                                      std::to_string(deps.bundles_requiring_regeneration.size()) + " sessions=" +
                                      std::to_string(deps.sessions_requiring_regeneration.size()));
          } else {
            PushLog(ui_state, "Bundle template updated; dirty spans=" +
                                      std::to_string(apply.change_set.dirty_span_ids.size()));
          }
          LoadBundleTemplateState(state, ui_state, tpl.id);
        }
      }
    }
    const auto& template_deps = state.view().template_dependency_state();
    ImGui::Text("Regen Required Bundles: %d", static_cast<int>(template_deps.bundles_requiring_regeneration.size()));
    ImGui::Text("Regen Required Sessions: %d", static_cast<int>(template_deps.sessions_requiring_regeneration.size()));

    ImGui::Separator();
    ImGui::Checkbox("Enable Support Structures", &ui_state.visual_enable_support_structures);
    ImGui::Checkbox("Enable Insulators", &ui_state.visual_enable_insulators);
    ImGui::InputDouble("Support Center Threshold", &ui_state.visual_support_center_threshold, 0.005, 0.01, "%.3f");
    ImGui::InputDouble("Support Arm Extra", &ui_state.visual_support_arm_extra, 0.01, 0.05, "%.3f");
    ImGui::InputDouble("Insulator Radius", &ui_state.visual_insulator_radius, 0.005, 0.01, "%.3f");
    ImGui::InputDouble("Insulator Length", &ui_state.visual_insulator_length, 0.005, 0.01, "%.3f");
    if (ImGui::Button("Apply Visual Cache Settings")) {
      wire::core::VisualSettings settings{};
      settings.enable_support_structures = ui_state.visual_enable_support_structures;
      settings.enable_insulators = ui_state.visual_enable_insulators;
      settings.support_center_threshold_m = ui_state.visual_support_center_threshold;
      settings.support_arm_extra_m = ui_state.visual_support_arm_extra;
      settings.insulator_radius_m = ui_state.visual_insulator_radius;
      settings.insulator_length_m = ui_state.visual_insulator_length;
      const auto apply = state.UpdateVisualSettings(settings, true);
      if (!apply.ok) {
        ui_state.last_error = apply.error;
        PushLog(ui_state, "UpdateVisualSettings failed");
      } else {
        ui_state.last_error.clear();
        PushLog(ui_state, "Visual cache settings updated");
      }
    }
  }

  if (ImGui::CollapsingHeader("Debug View", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::Checkbox("Show Span AABB", &ui_state.show_whole_aabb);
    ImGui::Checkbox("Show Segment AABB", &ui_state.show_segment_aabb);
    ImGui::Checkbox("Highlight Selected Bundle", &ui_state.show_selected_bundle_highlight);
    wire::core::CommitOptions options{};
    options.run_recalc = false;
    options.run_validate = true;
    const wire::core::ValidationResult validation = state.Commit(options).validation;
    ImGui::Text("Validation: %s", validation.ok() ? "OK" : "ERROR");
  }

  if (ImGui::CollapsingHeader("Slot Selection Debug")) {
    if (ImGui::Button("Clear Slot Debug Log")) {
      state.clear_slot_selection_debug_records();
      ui_state.selected_slot_debug_index = 0;
    }
    const auto& debug_records = state.view().slot_selection_debug_records();
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
      ImGui::Text("Selected templateSlot=%d result=%s", event.selected_slot_id, event.result.c_str());
    }
  }

  if (ImGui::CollapsingHeader("Backbone Junction Debug")) {
    wire::core::BackboneResult backbone = state.view().last_generation_backbone();
    if (backbone.edges.empty() && backbone.nodes.empty() && backbone.junctions.empty()) {
      backbone = state.BuildBackboneResult();
    }
    ImGui::Text("SupportNodes: %d", static_cast<int>(backbone.nodes.size()));
    ImGui::Text("Edges: %d", static_cast<int>(backbone.edges.size()));
    ImGui::Text("Junctions(deg>=3): %d", static_cast<int>(backbone.junctions.size()));
    for (const auto& node : backbone.nodes) {
      ImGui::Text("Node=%llu kind=%s pole=%llu pos=(%.2f,%.2f,%.2f)", static_cast<unsigned long long>(node.node_id),
                  SupportKindLabel(node.support_kind), static_cast<unsigned long long>(node.pole_id), node.position.x,
                  node.position.y, node.position.z);
      if (!node.bundle_modes.empty()) {
        for (const auto& mode : node.bundle_modes) {
          ImGui::Text("  bundle=%d mode=%s", static_cast<int>(mode.bundle_template_id), BundleNodeModeLabel(mode.mode));
        }
      }
    }
    for (const auto& junction : backbone.junctions) {
      ImGui::Separator();
      ImGui::Text("Node=%llu session=%llu continuity=%s", static_cast<unsigned long long>(junction.node_id),
                  static_cast<unsigned long long>(junction.prioritized_session_id),
                  junction.used_neighbor_continuity ? "true" : "false");
      for (const auto& inc : junction.incidents) {
        ImGui::Text("  -> neighbor=%llu order=%d primary=%s srcSession=%llu",
                    static_cast<unsigned long long>(inc.neighbor_node_id), inc.order,
                    inc.primary ? "true" : "false", static_cast<unsigned long long>(inc.source_session_id));
      }
    }
    if (!backbone.edge_orientations.empty()) {
      ImGui::Separator();
      ImGui::TextUnformatted("Edge Orientations");
      for (const auto& orientation : backbone.edge_orientations) {
        ImGui::Text("  %llu -> %llu bundle=%d flow=%s mirror=%s branchSupport=%s down=%.2f flipPrev=%s",
                    static_cast<unsigned long long>(orientation.node_a_id),
                    static_cast<unsigned long long>(orientation.node_b_id),
                    static_cast<int>(orientation.bundle_template_id), BackboneFlowKindLabel(orientation.flow_kind),
                    orientation.orientation == wire::core::LaneOrientation::kReversed ? "true" : "false",
                    orientation.uses_branch_support ? "true" : "false", orientation.branch_down_offset_m,
                    orientation.flipped_from_previous ? "true" : "false");
        ImGui::Text("    flowRule=%s", BackboneFlowDecisionRuleLabel(orientation.flow_decision_rule));
      }
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

} // namespace

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




