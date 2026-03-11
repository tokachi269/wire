#include "wire/core/core_state.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string_view>
#include <unordered_set>

namespace wire::core {
namespace {

std::string DisplayOrFallback(std::string_view prefix, std::string_view display_id, std::uint64_t stable_id) {
  if (!display_id.empty()) {
    return std::string(display_id);
  }
  return std::string(prefix) + " " + std::to_string(static_cast<unsigned long long>(stable_id));
}

std::string BoolText(bool value) { return value ? "true" : "false"; }

const char* PoleForwardRuleText(PoleForwardRule rule) {
  switch (rule) {
  case PoleForwardRule::kFallback:
    return "Fallback";
  case PoleForwardRule::kPrimaryIncident:
    return "PrimaryIncident";
  case PoleForwardRule::kMainChainSingle:
    return "MainChainSingle";
  case PoleForwardRule::kMainChainBisector:
    return "MainChainBisector";
  default:
    return "Unknown";
  }
}

const char* PoleSupportAxisRuleText(PoleSupportAxisRule rule) {
  switch (rule) {
  case PoleSupportAxisRule::kFallback:
    return "Fallback";
  case PoleSupportAxisRule::kPrimaryIncident:
    return "PrimaryIncident";
  case PoleSupportAxisRule::kMainChainSingle:
    return "MainChainSingle";
  case PoleSupportAxisRule::kMainChainPair:
    return "MainChainPair";
  default:
    return "Unknown";
  }
}

const char* FlowKindText(BackboneFlowKind kind) {
  switch (kind) {
  case BackboneFlowKind::kMain:
    return "Main";
  case BackboneFlowKind::kBranch:
    return "Branch";
  default:
    return "Unknown";
  }
}

const char* FlowDecisionRuleText(BackboneFlowDecisionRule rule) {
  switch (rule) {
  case BackboneFlowDecisionRule::kDefaultMain:
    return "DefaultMain";
  case BackboneFlowDecisionRule::kJunctionOrderMain:
    return "JunctionOrderMain";
  case BackboneFlowDecisionRule::kJunctionOrderBranch:
    return "JunctionOrderBranch";
  case BackboneFlowDecisionRule::kExistingChainMain:
    return "ExistingChainMain";
  case BackboneFlowDecisionRule::kExistingChainBranch:
    return "ExistingChainBranch";
  default:
    return "Unknown";
  }
}

const char* SupportLayoutOriginText(SupportLayoutOriginKind origin) {
  switch (origin) {
  case SupportLayoutOriginKind::kMainSupport:
    return "MainSupport";
  case SupportLayoutOriginKind::kBranchSupport:
    return "BranchSupport";
  case SupportLayoutOriginKind::kAerialBranch:
    return "AerialBranch";
  case SupportLayoutOriginKind::kFallback:
  default:
    return "Fallback";
  }
}

const char* SupportLayoutEndpointSourceText(SupportLayoutEndpointSourceKind source) {
  switch (source) {
  case SupportLayoutEndpointSourceKind::kPlainSupport:
    return "PlainSupport";
  case SupportLayoutEndpointSourceKind::kAttachmentSocket:
    return "AttachmentSocket";
  case SupportLayoutEndpointSourceKind::kAttachmentSocketOverride:
    return "AttachmentSocketOverride";
  case SupportLayoutEndpointSourceKind::kFallback:
  default:
    return "Fallback";
  }
}

const char* PortPlacementSourceText(PortPlacementSourceKind source) {
  switch (source) {
  case PortPlacementSourceKind::kUnknown:
    return "Unknown";
  case PortPlacementSourceKind::kTemplateSlot:
    return "TemplateSlot";
  case PortPlacementSourceKind::kGenerated:
    return "Generated";
  case PortPlacementSourceKind::kManualEdit:
    return "ManualEdit";
  case PortPlacementSourceKind::kAerialBranch:
    return "AerialBranch";
  case PortPlacementSourceKind::kBranchSupport:
    return "BranchSupport";
  default:
    return "Unknown";
  }
}

const char* CurveEndpointModeText(CurveEndpointMode mode) {
  switch (mode) {
  case CurveEndpointMode::kDirectThrough:
    return "DirectThrough";
  case CurveEndpointMode::kOffsetEndpoint:
    return "OffsetEndpoint";
  default:
    return "Unknown";
  }
}

const char* TangentRuleText(DetailCurveEndpointTangentRule rule) {
  switch (rule) {
  case DetailCurveEndpointTangentRule::kFallbackChord:
    return "FallbackChord";
  case DetailCurveEndpointTangentRule::kMainFlowBlend:
    return "MainFlowBlend";
  case DetailCurveEndpointTangentRule::kBranchChordPriority:
    return "BranchChordPriority";
  case DetailCurveEndpointTangentRule::kTerminateEndpointPriority:
    return "TerminateEndpointPriority";
  case DetailCurveEndpointTangentRule::kAttachmentEndpointPriority:
    return "AttachmentEndpointPriority";
  default:
    return "Unknown";
  }
}

const char* ContinuityModeText(DetailCurveContinuityMode mode) {
  switch (mode) {
  case DetailCurveContinuityMode::kG1:
    return "G1";
  case DetailCurveContinuityMode::kG2:
    return "G2";
  default:
    return "Unknown";
  }
}

const char* ContinuityReasonText(DetailCurveContinuityReason reason) {
  switch (reason) {
  case DetailCurveContinuityReason::kAutoBalanced:
    return "AutoBalanced";
  case DetailCurveContinuityReason::kSmoothPassThrough:
    return "SmoothPassThrough";
  case DetailCurveContinuityReason::kPolicyPreferG1:
    return "PolicyPreferG1";
  case DetailCurveContinuityReason::kShortSpan:
    return "ShortSpan";
  case DetailCurveContinuityReason::kBranchPass:
    return "BranchPass";
  case DetailCurveContinuityReason::kCornerPass:
    return "CornerPass";
  case DetailCurveContinuityReason::kEndpointConstraintPriority:
    return "EndpointConstraintPriority";
  case DetailCurveContinuityReason::kConflictingTangents:
    return "ConflictingTangents";
  case DetailCurveContinuityReason::kContextInsufficient:
    return "ContextInsufficient";
  case DetailCurveContinuityReason::kPoorQualityFallback:
    return "PoorQualityFallback";
  default:
    return "Unknown";
  }
}

const char* ContinuityPolicyText(CableContinuityPolicyHint policy) {
  switch (policy) {
  case CableContinuityPolicyHint::kAuto:
    return "Auto";
  case CableContinuityPolicyHint::kPreferG1:
    return "PreferG1";
  case CableContinuityPolicyHint::kPreferG2:
    return "PreferG2";
  default:
    return "Unknown";
  }
}

std::uint64_t StableBackboneEdgeId(ObjectId a, ObjectId b) {
  const std::uint64_t lo = std::min(a, b);
  const std::uint64_t hi = std::max(a, b);
  return (lo << 32) ^ hi;
}

void AddLink(std::vector<RelatedEntityLink>* links, std::unordered_set<std::uint64_t>* seen, std::string label,
             EntityKind kind, std::uint64_t stable_id) {
  if (stable_id == 0 || links == nullptr || seen == nullptr) {
    return;
  }
  const std::uint64_t key = (static_cast<std::uint64_t>(kind) << 56) ^ stable_id;
  if (!seen->insert(key).second) {
    return;
  }
  links->push_back({std::move(label), EntityRef{kind, stable_id}});
}

const SegmentLaneAssignment* FindLaneAssignmentForSpan(const CoreState& state, const Span& span) {
  for (const auto& assignment : state.view().last_lane_assignments()) {
    const bool same_forward = assignment.bundle_id == span.bundle_id && assignment.pole_a_id == span.endpoint_node_a_id &&
                              assignment.pole_b_id == span.endpoint_node_b_id;
    const bool same_reverse = assignment.bundle_id == span.bundle_id && assignment.pole_a_id == span.endpoint_node_b_id &&
                              assignment.pole_b_id == span.endpoint_node_a_id;
    if (same_forward || same_reverse) {
      return &assignment;
    }
  }
  return nullptr;
}

std::optional<JunctionInfo> FindJunctionByNode(const CoreState& state, ObjectId node_id) {
  for (const JunctionInfo& junction : state.view().last_generation_backbone().junctions) {
    if (junction.node_id == node_id) {
      return junction;
    }
  }
  const BackboneResult rebuilt = state.BuildBackboneResult();
  for (const JunctionInfo& junction : rebuilt.junctions) {
    if (junction.node_id == node_id) {
      return junction;
    }
  }
  return std::nullopt;
}

std::optional<SupportNode> FindSupportNodeById(const CoreState& state, ObjectId node_id) {
  for (const SupportNode& node : state.view().last_generation_backbone().nodes) {
    if (node.node_id == node_id) {
      return node;
    }
  }
  const BackboneResult rebuilt = state.BuildBackboneResult();
  for (const SupportNode& node : rebuilt.nodes) {
    if (node.node_id == node_id) {
      return node;
    }
  }
  return std::nullopt;
}

EntityMeta MakeMeta(EntityKind kind, std::uint64_t stable_id, std::string display_name, EntityRoleKind role, bool editable,
                    bool overrideable, std::string provenance) {
  EntityMeta meta{};
  meta.ref = EntityRef{kind, stable_id};
  meta.display_name = std::move(display_name);
  meta.role = role;
  meta.editable = editable;
  meta.overrideable = overrideable;
  meta.provenance = std::move(provenance);
  return meta;
}

VariationBreakdownView MakeVariationBreakdownView(const HierarchicalVariationSample& sample) {
  VariationBreakdownView view{};
  view.flow_key = sample.flow_key;
  view.world_bias = sample.world_bias;
  view.flow_bias = sample.flow_bias;
  view.pole_delta = sample.pole_delta;
  view.local_jitter = sample.local_jitter;
  view.final_value = sample.final_value;
  return view;
}

SupportLayoutEndpointView MakeSupportLayoutEndpointView(const SupportLayoutEndpoint& endpoint) {
  SupportLayoutEndpointView view{};
  view.endpoint_node_id = endpoint.endpoint_node_id;
  view.owner_pole_id = endpoint.owner_pole_id;
  view.port_id = endpoint.port_id;
  view.attachment_id = endpoint.attachment_id;
  view.socket_id = endpoint.socket_id;
  view.flow_kind = endpoint.flow_kind;
  view.side = endpoint.side;
  view.origin = SupportLayoutOriginText(endpoint.origin);
  view.endpoint_source = SupportLayoutEndpointSourceText(endpoint.endpoint_source);
  view.attachment_input_present = endpoint.attachment_input_present;
  view.socket_override_active = endpoint.socket_override_active;
  view.port_source = PortPlacementSourceText(endpoint.port_source);
  view.endpoint_mode = CurveEndpointModeText(endpoint.endpoint_mode);
  view.support_world = endpoint.support_world;
  view.endpoint_world = endpoint.endpoint_world;
  view.departure_dir = endpoint.departure_dir;
  view.endpoint_offset = endpoint.endpoint_offset;
  view.local_departure_length_m = endpoint.local_departure_length_m;
  view.automatic_branch_down_offset_m = endpoint.automatic_branch_down_offset_m;
  view.branch_down_offset_m = endpoint.branch_down_offset_m;
  view.down_offset_variation = MakeVariationBreakdownView(endpoint.down_offset_variation);
  return view;
}

} // namespace

std::optional<EntityMeta> CoreView::describe_entity(EntityRef ref) const {
  if (!ref.valid()) {
    return std::nullopt;
  }
  switch (ref.kind) {
  case EntityKind::kPole: {
    const Pole* pole = poles().find(ref.stable_id);
    if (pole == nullptr) {
      return std::nullopt;
    }
    return MakeMeta(ref.kind, ref.stable_id, DisplayOrFallback("Pole", pole->display_id, ref.stable_id),
                    EntityRoleKind::kAuthoritative, true, true, "entity.pole");
  }
  case EntityKind::kSpan: {
    const Span* span = spans().find(ref.stable_id);
    if (span == nullptr) {
      return std::nullopt;
    }
    return MakeMeta(ref.kind, ref.stable_id, DisplayOrFallback("Span", span->display_id, ref.stable_id),
                    EntityRoleKind::kAuthoritative, true, true, "entity.span");
  }
  case EntityKind::kBundle: {
    const Bundle* bundle = bundles().find(ref.stable_id);
    if (bundle == nullptr) {
      return std::nullopt;
    }
    return MakeMeta(ref.kind, ref.stable_id, DisplayOrFallback("Bundle", bundle->display_id, ref.stable_id),
                    EntityRoleKind::kAuthoritative, true, false, "entity.bundle");
  }
  case EntityKind::kSupportLayout: {
    if (find_span_support_layout(ref.stable_id) == nullptr) {
      return std::nullopt;
    }
    return MakeMeta(ref.kind, ref.stable_id, "SupportLayout for span " + std::to_string(ref.stable_id),
                    EntityRoleKind::kDerived, false, false, "derived.support_layout");
  }
  case EntityKind::kDetailCurve: {
    if (state_.find_curve_cache(ref.stable_id) == nullptr) {
      return std::nullopt;
    }
    return MakeMeta(ref.kind, ref.stable_id, "DetailCurve for span " + std::to_string(ref.stable_id),
                    EntityRoleKind::kDetailDerived, false, false, "detail.curve_cache");
  }
  case EntityKind::kJunction: {
    if (!FindJunctionByNode(state_, ref.stable_id).has_value()) {
      return std::nullopt;
    }
    return MakeMeta(ref.kind, ref.stable_id, "Junction " + std::to_string(ref.stable_id), EntityRoleKind::kDerived,
                    false, true, "derived.backbone");
  }
  case EntityKind::kSupportNode: {
    if (!FindSupportNodeById(state_, ref.stable_id).has_value()) {
      return std::nullopt;
    }
    return MakeMeta(ref.kind, ref.stable_id, "SupportNode " + std::to_string(ref.stable_id), EntityRoleKind::kDerived,
                    false, false, "derived.backbone");
  }
  case EntityKind::kOverride:
    return MakeMeta(ref.kind, ref.stable_id, "Override " + std::to_string(ref.stable_id), EntityRoleKind::kOverride,
                    true, true, "override.surface");
  default:
    break;
  }
  return std::nullopt;
}

std::optional<PoleInspectionView> CoreView::inspect_pole(ObjectId pole_id) const {
  const Pole* pole = poles().find(pole_id);
  if (pole == nullptr) {
    return std::nullopt;
  }

  PoleInspectionView result{};
  result.meta = *describe_entity({EntityKind::kPole, pole_id});
  result.pole_id = pole_id;
  result.position = pole->world_transform.position;
  result.height_m = pole->height_m;
  result.tilt_deg = {pole->world_transform.rotation_euler_deg.x, pole->world_transform.rotation_euler_deg.y,
                     pole->world_transform.rotation_euler_deg.z};
  result.manual_yaw_override = state_.resolve_pole_manual_yaw_override(*pole).has_value();
  result.manual_yaw_deg = state_.resolve_pole_manual_yaw_override(*pole).value_or(0.0);
  result.flip_180_override = state_.resolve_pole_flip_180_override(*pole).value_or(false);
  result.placement_override = pole->placement_override_flag;
  result.orientation_override = state_.has_pole_orientation_override(pole_id);
  result.final_yaw_deg = state_.effective_pole_yaw_deg(*pole);
  result.has_final_yaw = true;
  result.layout_yaw_deg = state_.effective_pole_layout_yaw_deg(*pole);
  result.has_layout_yaw = true;
  if (const auto it = pole_orientation_debug_records().find(pole_id); it != pole_orientation_debug_records().end()) {
    result.forward_rule = it->second.rule;
    result.support_axis_rule = it->second.support_axis_rule;
    result.primary_neighbor_id = it->second.primary_neighbor_id;
    result.secondary_neighbor_id = it->second.secondary_neighbor_id;
    result.forward_dir = it->second.adopted_forward;
    result.has_forward = true;
    result.support_axis_dir = it->second.adopted_support_axis;
    const double support_axis_len2 = result.support_axis_dir.x * result.support_axis_dir.x +
                                     result.support_axis_dir.y * result.support_axis_dir.y +
                                     result.support_axis_dir.z * result.support_axis_dir.z;
    result.has_support_axis = support_axis_len2 > 1e-12;
    result.automatic_yaw_deg =
        std::atan2(it->second.adopted_forward.y, it->second.adopted_forward.x) * (180.0 / 3.14159265358979323846);
  }

  std::unordered_set<std::uint64_t> seen{};
  const auto ports_it = relation_index().ports_by_pole.find(pole_id);
  if (ports_it != relation_index().ports_by_pole.end()) {
    for (ObjectId port_id : ports_it->second) {
      const auto spans_it = connection_index().spans_by_port.find(port_id);
      if (spans_it == connection_index().spans_by_port.end()) {
        continue;
      }
      for (ObjectId span_id : spans_it->second) {
        AddLink(&result.links, &seen, "Span " + std::to_string(span_id), EntityKind::kSpan, span_id);
        if (const SpanSupportLayoutEntry* layout = find_span_support_layout(span_id); layout != nullptr &&
            (layout->start.owner_pole_id == pole_id || layout->end.owner_pole_id == pole_id)) {
          AddLink(&result.links, &seen, "SupportLayout " + std::to_string(span_id), EntityKind::kSupportLayout, span_id);
        }
      }
    }
  }
  if (pole->pole_type_id != kInvalidPoleTypeId) {
    AddLink(&result.links, &seen, "PoleTemplate " + std::to_string(pole->pole_type_id), EntityKind::kTemplate,
            pole->pole_type_id);
  }
  AddLink(&result.links, &seen, "Override " + std::to_string(pole_id), EntityKind::kOverride, pole_id);
  return result;
}

std::optional<SpanInspectionView> CoreView::inspect_span(ObjectId span_id) const {
  const Span* span = spans().find(span_id);
  if (span == nullptr) {
    return std::nullopt;
  }

  SpanInspectionView result{};
  result.meta = *describe_entity({EntityKind::kSpan, span_id});
  result.span_id = span_id;
  result.port_a_id = span->port_a_id;
  result.port_b_id = span->port_b_id;
  if (span->bundle_id != kInvalidObjectId) {
    result.bundle_ref = {EntityKind::kBundle, span->bundle_id};
  }
  if (const Port* port_a = ports().find(span->port_a_id); port_a != nullptr && port_a->owner_pole_id != kInvalidObjectId) {
    result.start_pole_ref = {EntityKind::kPole, port_a->owner_pole_id};
  }
  if (const Port* port_b = ports().find(span->port_b_id); port_b != nullptr && port_b->owner_pole_id != kInvalidObjectId) {
    result.end_pole_ref = {EntityKind::kPole, port_b->owner_pole_id};
  }
  if (const SpanSupportLayoutEntry* layout = find_span_support_layout(span_id); layout != nullptr) {
    result.support_layout_ref = {EntityKind::kSupportLayout, span_id};
    result.flow_kind = layout->flow_kind;
    result.uses_branch_support = layout->start.origin == SupportLayoutOriginKind::kBranchSupport ||
                                 layout->end.origin == SupportLayoutOriginKind::kBranchSupport;
    result.branch_down_offset_m = std::max(layout->start.branch_down_offset_m, layout->end.branch_down_offset_m);
  }
  if (const CurveCacheEntry* curve = state_.find_curve_cache(span_id); curve != nullptr) {
    result.detail_curve_ref = {EntityKind::kDetailCurve, span_id};
    result.requested_continuity = curve->detail.quality.requested_policy;
    result.adopted_continuity = curve->detail.quality.adopted_continuity;
    result.continuity_reason = curve->detail.quality.continuity_reason;
    result.degraded_to_g1 = curve->detail.quality.degraded_to_g1;
    result.sag_amplitude_m = curve->detail.sag_amplitude_m;
    result.curve_length_m = curve->detail.Length();
  }
  if (const SegmentLaneAssignment* assignment = FindLaneAssignmentForSpan(state_, *span); assignment != nullptr) {
    result.flow_kind = assignment->flow_kind;
    result.flow_rule = assignment->flow_decision_rule;
    result.uses_branch_support = assignment->uses_branch_support;
    result.branch_down_offset_m = assignment->branch_down_offset_m;
    result.mirrored = assignment->mirrored;
    result.flipped_from_previous = assignment->flipped_from_previous;
    result.turn_angle_deg = assignment->turn_angle_deg;
  }

  std::unordered_set<std::uint64_t> seen{};
  if (result.start_pole_ref.valid()) {
    AddLink(&result.links, &seen, "Start Pole", result.start_pole_ref.kind, result.start_pole_ref.stable_id);
  }
  if (result.end_pole_ref.valid()) {
    AddLink(&result.links, &seen, "End Pole", result.end_pole_ref.kind, result.end_pole_ref.stable_id);
  }
  if (result.bundle_ref.valid()) {
    AddLink(&result.links, &seen, "Bundle", result.bundle_ref.kind, result.bundle_ref.stable_id);
  }
  if (result.support_layout_ref.valid()) {
    AddLink(&result.links, &seen, "SupportLayout", result.support_layout_ref.kind, result.support_layout_ref.stable_id);
  }
  if (result.detail_curve_ref.valid()) {
    AddLink(&result.links, &seen, "DetailCurve", result.detail_curve_ref.kind, result.detail_curve_ref.stable_id);
  }
  AddLink(&result.links, &seen, "Override " + std::to_string(span_id), EntityKind::kOverride, span_id);
  return result;
}

std::optional<SupportLayoutInspectionView> CoreView::inspect_support_layout(ObjectId span_id) const {
  const SpanSupportLayoutEntry* layout = find_span_support_layout(span_id);
  if (layout == nullptr) {
    return std::nullopt;
  }
  SupportLayoutInspectionView result{};
  result.meta = *describe_entity({EntityKind::kSupportLayout, span_id});
  result.source_span = {EntityKind::kSpan, span_id};
  result.flow_kind = layout->flow_kind;
  result.pass_mode = layout->pass_mode;
  result.variation_flow_key = layout->variation_flow_key;
  result.start_endpoint = MakeSupportLayoutEndpointView(layout->start);
  result.end_endpoint = MakeSupportLayoutEndpointView(layout->end);

  std::unordered_set<std::uint64_t> seen{};
  AddLink(&result.links, &seen, "Source Span", EntityKind::kSpan, span_id);
  if (layout->start.owner_pole_id != kInvalidObjectId) {
    AddLink(&result.links, &seen, "Start Pole", EntityKind::kPole, layout->start.owner_pole_id);
  }
  if (layout->end.owner_pole_id != kInvalidObjectId) {
    AddLink(&result.links, &seen, "End Pole", EntityKind::kPole, layout->end.owner_pole_id);
  }
  if (state_.find_curve_cache(span_id) != nullptr) {
    AddLink(&result.links, &seen, "DetailCurve", EntityKind::kDetailCurve, span_id);
  }
  return result;
}

std::optional<DetailCurveInspectionView> CoreView::inspect_detail_curve(ObjectId span_id) const {
  const CurveCacheEntry* curve = state_.find_curve_cache(span_id);
  if (curve == nullptr) {
    return std::nullopt;
  }

  DetailCurveInspectionView result{};
  result.meta = *describe_entity({EntityKind::kDetailCurve, span_id});
  result.source_span = {EntityKind::kSpan, span_id};
  result.requested_continuity = curve->detail.quality.requested_policy;
  result.shape_policy = curve->detail.quality.shape_policy;
  result.adopted_continuity = curve->detail.quality.adopted_continuity;
  result.continuity_reason = curve->detail.quality.continuity_reason;
  result.attempted_g2 = curve->detail.quality.attempted_g2;
  result.degraded_to_g1 = curve->detail.quality.degraded_to_g1;
  result.sag_amplitude_m = curve->detail.sag_amplitude_m;
  result.curve_length_m = curve->detail.Length();
  result.tangent_scale = curve->detail.quality.tangent_scale;
  result.base_handle_scale = curve->detail.quality.base_handle_scale;
  result.policy_handle_scale = curve->detail.quality.policy_handle_scale;
  result.start_angle_scale = curve->detail.quality.start_angle_scale;
  result.end_angle_scale = curve->detail.quality.end_angle_scale;
  result.handle_length_start_m = curve->detail.quality.handle_length_start_m;
  result.handle_length_end_m = curve->detail.quality.handle_length_end_m;
  result.control_points = curve->detail.control_points;
  result.arc_length_sample_count = curve->detail.arc_length_table.size();
  result.visible_interval_count = curve->detail.visible_intervals.size();
  result.hidden_interval_count = curve->detail.hidden_intervals.size();
  result.replacement_path_count = curve->detail.replacement_paths.size();
  result.start_tangent_rule = curve->detail.quality.start_tangent_rule;
  result.end_tangent_rule = curve->detail.quality.end_tangent_rule;
  result.start_support_weight = curve->detail.quality.start_support_weight;
  result.end_support_weight = curve->detail.quality.end_support_weight;
  result.start_chord_weight = curve->detail.quality.start_chord_weight;
  result.end_chord_weight = curve->detail.quality.end_chord_weight;
  result.start_departure_length_m = curve->detail.quality.start_departure_length_m;
  result.end_departure_length_m = curve->detail.quality.end_departure_length_m;
  result.start_lateral_ratio_limit = curve->detail.quality.start_lateral_ratio_limit;
  result.end_lateral_ratio_limit = curve->detail.quality.end_lateral_ratio_limit;
  if (const SpanSupportLayoutEntry* layout = find_span_support_layout(span_id); layout != nullptr) {
    result.start_endpoint_source = SupportLayoutEndpointSourceText(layout->start.endpoint_source);
    result.end_endpoint_source = SupportLayoutEndpointSourceText(layout->end.endpoint_source);
    result.start_attachment_input_present = layout->start.attachment_input_present;
    result.end_attachment_input_present = layout->end.attachment_input_present;
    result.start_socket_id = layout->start.socket_id;
    result.end_socket_id = layout->end.socket_id;
  }
  result.sag_base_ratio = curve->detail.quality.sag_base_ratio;
  result.sag_length_scale = curve->detail.quality.sag_length_scale;
  result.sag_pass_scale = curve->detail.quality.sag_pass_scale;
  result.sag_rigidity_scale = curve->detail.quality.sag_rigidity_scale;
  result.sag_variation = MakeVariationBreakdownView(curve->detail.quality.sag_variation);

  std::unordered_set<std::uint64_t> seen{};
  AddLink(&result.links, &seen, "Source Span", EntityKind::kSpan, span_id);
  if (const SpanSupportLayoutEntry* layout = find_span_support_layout(span_id); layout != nullptr) {
    AddLink(&result.links, &seen, "SupportLayout", EntityKind::kSupportLayout, span_id);
    if (layout->start.owner_pole_id != kInvalidObjectId) {
      AddLink(&result.links, &seen, "Start Pole", EntityKind::kPole, layout->start.owner_pole_id);
    }
    if (layout->end.owner_pole_id != kInvalidObjectId) {
      AddLink(&result.links, &seen, "End Pole", EntityKind::kPole, layout->end.owner_pole_id);
    }
  }
  return result;
}

std::optional<JunctionInspectionView> CoreView::inspect_junction(ObjectId node_id) const {
  const auto junction = FindJunctionByNode(state_, node_id);
  if (!junction.has_value()) {
    return std::nullopt;
  }
  JunctionInspectionView result{};
  result.meta = *describe_entity({EntityKind::kJunction, node_id});
  result.node_id = node_id;
  result.incidents = junction->incidents;
  result.has_primary = std::any_of(junction->incidents.begin(), junction->incidents.end(),
                                   [](const JunctionIncident& incident) { return incident.primary; });

  std::unordered_set<std::uint64_t> seen{};
  for (const JunctionIncident& incident : junction->incidents) {
    AddLink(&result.links, &seen, "Neighbor " + std::to_string(incident.neighbor_node_id), EntityKind::kSupportNode,
            incident.neighbor_node_id);
    AddLink(&result.links, &seen, "Edge " + std::to_string(StableBackboneEdgeId(node_id, incident.neighbor_node_id)),
            EntityKind::kBackboneEdge, StableBackboneEdgeId(node_id, incident.neighbor_node_id));
  }
  if (const auto node = FindSupportNodeById(state_, node_id); node.has_value() && node->pole_id != kInvalidObjectId) {
    AddLink(&result.links, &seen, "Owner Pole", EntityKind::kPole, node->pole_id);
  }
  return result;
}

std::optional<TemplateInspectionView> CoreView::inspect_pole_template(PoleTypeId pole_type_id) const {
  const auto it = pole_types().find(pole_type_id);
  if (it == pole_types().end()) {
    return std::nullopt;
  }
  TemplateInspectionView result{};
  result.meta = MakeMeta(EntityKind::kTemplate, pole_type_id,
                         DisplayOrFallback("PoleTemplate", it->second.name, pole_type_id),
                         EntityRoleKind::kAuthoritative, true, false, "definition.pole_type");
  result.template_kind = TemplateKind::kPoleType;
  result.properties.push_back({"port_slots", std::to_string(static_cast<unsigned long long>(it->second.port_slots.size())),
                               PropertyAccessKind::kEditable});
  result.properties.push_back(
      {"anchor_slots", std::to_string(static_cast<unsigned long long>(it->second.anchor_slots.size())),
       PropertyAccessKind::kEditable});
  return result;
}

std::optional<TemplateInspectionView> CoreView::inspect_cable_template(CableTemplateId cable_template_id) const {
  const auto it = cable_templates().find(cable_template_id);
  if (it == cable_templates().end()) {
    return std::nullopt;
  }
  const CableTemplate& tpl = it->second;
  TemplateInspectionView result{};
  result.meta = MakeMeta(EntityKind::kTemplate, cable_template_id,
                         DisplayOrFallback("CableTemplate", tpl.name, cable_template_id),
                         EntityRoleKind::kAuthoritative, true, false, "definition.cable_template");
  result.template_kind = TemplateKind::kCable;
  result.properties.push_back({"continuity", ContinuityPolicyText(tpl.continuity_policy), PropertyAccessKind::kEditable});
  result.properties.push_back({"stiffness", std::to_string(tpl.bend_stiffness), PropertyAccessKind::kEditable});
  result.properties.push_back(
      {"min_bend_radius_m", std::to_string(tpl.min_bend_radius_m), PropertyAccessKind::kEditable});
  result.properties.push_back({"sag_factor", std::to_string(tpl.sag_factor), PropertyAccessKind::kEditable});
  result.properties.push_back({"slack_factor", std::to_string(tpl.slack_factor), PropertyAccessKind::kEditable});
  return result;
}

std::optional<TemplateInspectionView> CoreView::inspect_bundle_template(BundleKind bundle_template_id) const {
  const auto it = bundle_templates().find(bundle_template_id);
  if (it == bundle_templates().end()) {
    return std::nullopt;
  }
  const BundleTemplate& tpl = it->second;
  TemplateInspectionView result{};
  result.meta = MakeMeta(EntityKind::kTemplate, static_cast<std::uint64_t>(bundle_template_id),
                         DisplayOrFallback("BundleTemplate", tpl.name, static_cast<std::uint64_t>(bundle_template_id)),
                         EntityRoleKind::kAuthoritative, true, false, "definition.bundle_template");
  result.template_kind = TemplateKind::kBundle;
  result.properties.push_back({"continuity", ContinuityPolicyText(tpl.continuity_policy), PropertyAccessKind::kEditable});
  result.properties.push_back({"allow_mirror", BoolText(tpl.allow_mirror), PropertyAccessKind::kEditable});
  result.properties.push_back({"allow_midair_node", BoolText(tpl.allow_midair_node), PropertyAccessKind::kEditable});
  result.properties.push_back({"allow_midair_branch", BoolText(tpl.allow_midair_branch), PropertyAccessKind::kEditable});
  result.properties.push_back(
      {"support_style", std::to_string(static_cast<int>(tpl.support_style)), PropertyAccessKind::kEditable});
  result.properties.push_back(
      {"branch_policy", std::to_string(static_cast<int>(tpl.branch_policy)), PropertyAccessKind::kEditable});
  result.properties.push_back({"cable_template_id", std::to_string(static_cast<unsigned long long>(tpl.cable_template_id)),
                               PropertyAccessKind::kEditable});
  if (tpl.cable_template_id != kInvalidCableTemplateId) {
    result.links.push_back({"CableTemplate", {EntityKind::kTemplate, tpl.cable_template_id}});
  }
  return result;
}

std::optional<TemplateInspectionView> CoreView::inspect_attachment_template(
    AttachmentTemplateId attachment_template_id) const {
  const auto it = attachment_templates().find(attachment_template_id);
  if (it == attachment_templates().end()) {
    return std::nullopt;
  }
  const AttachmentTemplate& tpl = it->second;
  TemplateInspectionView result{};
  result.meta = MakeMeta(EntityKind::kTemplate, attachment_template_id,
                         DisplayOrFallback("AttachmentTemplate", tpl.name, attachment_template_id),
                         EntityRoleKind::kAuthoritative, true, false, "definition.attachment_template");
  result.template_kind = TemplateKind::kAttachment;
  result.properties.push_back({"line_interaction_mode", std::to_string(static_cast<int>(tpl.line_interaction_mode)),
                               PropertyAccessKind::kEditable});
  result.properties.push_back({"socket_count", std::to_string(static_cast<unsigned long long>(tpl.sockets.size())),
                               PropertyAccessKind::kEditable});
  result.properties.push_back({"internal_path_count",
                               std::to_string(static_cast<unsigned long long>(tpl.internal_paths.size())),
                               PropertyAccessKind::kEditable});
  return result;
}

std::optional<OverrideInspectionView> CoreView::inspect_overrides(EntityRef target) const {
  if (!target.valid()) {
    return std::nullopt;
  }
  OverrideInspectionView result{};
  result.meta = MakeMeta(EntityKind::kOverride, target.stable_id, "Override surface", EntityRoleKind::kOverride, true,
                         true, "override.surface");
  result.target = target;
  result.links.push_back({"Target", target});

  if (target.kind == EntityKind::kPole) {
    const Pole* pole = poles().find(target.stable_id);
    if (pole == nullptr) {
      return std::nullopt;
    }
    std::string auto_yaw = "n/a";
    if (const auto it = pole_orientation_debug_records().find(target.stable_id); it != pole_orientation_debug_records().end()) {
      const double yaw_deg = std::atan2(it->second.adopted_forward.y, it->second.adopted_forward.x) * (180.0 / 3.14159265358979323846);
      auto_yaw = std::to_string(yaw_deg);
    }
    const std::optional<double> manual_yaw = state_.resolve_pole_manual_yaw_override(*pole);
    const std::optional<bool> flip_180 = state_.resolve_pole_flip_180_override(*pole);
    result.entries.push_back({OverrideTopicKind::kPoleYaw,
                              "manualYaw",
                              auto_yaw,
                              manual_yaw.has_value() ? std::to_string(*manual_yaw) : "auto",
                              std::to_string(state_.effective_pole_yaw_deg(*pole)),
                              manual_yaw.has_value(),
                              PropertyAccessKind::kOverrideable});
    result.entries.push_back({OverrideTopicKind::kPoleForward,
                              "flip180",
                              "false",
                              flip_180.has_value() ? BoolText(*flip_180) : "auto",
                              BoolText(flip_180.value_or(false)),
                              flip_180.has_value(),
                              PropertyAccessKind::kOverrideable});
    return result;
  }
  if (target.kind == EntityKind::kSpan) {
    const Span* span = spans().find(target.stable_id);
    if (span == nullptr) {
      return std::nullopt;
    }
    const int socket_a = state_.resolve_span_endpoint_socket_id(*span, true);
    const int socket_b = state_.resolve_span_endpoint_socket_id(*span, false);
    const bool socket_a_active = state_.has_span_endpoint_socket_override(span->id, true);
    const bool socket_b_active = state_.has_span_endpoint_socket_override(span->id, false);
    result.entries.push_back({OverrideTopicKind::kAttachmentSocket,
                              "endpointSocketA",
                              "-1",
                              socket_a_active ? std::to_string(socket_a) : "auto",
                              std::to_string(socket_a),
                              socket_a_active,
                              PropertyAccessKind::kOverrideable});
    result.entries.push_back({OverrideTopicKind::kAttachmentSocket,
                              "endpointSocketB",
                              "-1",
                              socket_b_active ? std::to_string(socket_b) : "auto",
                              std::to_string(socket_b),
                              socket_b_active,
                              PropertyAccessKind::kOverrideable});
    double automatic_down_offset = 0.0;
    double final_down_offset = 0.0;
    bool down_active = false;
    if (const SpanSupportLayoutEntry* layout = find_span_support_layout(span->id); layout != nullptr) {
      automatic_down_offset =
          std::max(layout->start.automatic_branch_down_offset_m, layout->end.automatic_branch_down_offset_m);
      final_down_offset = std::max(layout->start.branch_down_offset_m, layout->end.branch_down_offset_m);
      down_active = state_.has_span_branch_down_offset_override(span->id);
    }
    result.entries.push_back({OverrideTopicKind::kBranchDownOffset,
                              "branchDownOffset",
                              std::to_string(automatic_down_offset),
                              down_active ? std::to_string(final_down_offset) : "auto",
                              std::to_string(final_down_offset),
                              down_active,
                              PropertyAccessKind::kOverrideable});
    result.entries.push_back({OverrideTopicKind::kMirror, "mirrorSelection", "auto", "not implemented", "auto", false,
                              PropertyAccessKind::kOverrideable});
    result.entries.push_back({OverrideTopicKind::kFlowClassification, "flowClassification", "auto", "not implemented",
                              "auto", false, PropertyAccessKind::kOverrideable});
    return result;
  }
  return std::nullopt;
}

std::vector<DecisionTraceEntry> CoreView::collect_decision_trace(EntityRef ref) const {
  std::vector<DecisionTraceEntry> trace{};
  if (!ref.valid()) {
    return trace;
  }

  if (ref.kind == EntityKind::kPole) {
    if (const auto it = pole_orientation_debug_records().find(ref.stable_id); it != pole_orientation_debug_records().end()) {
      std::ostringstream summary;
      summary << "forward=" << it->second.adopted_forward.x << "," << it->second.adopted_forward.y << ","
              << it->second.adopted_forward.z << " supportAxis=" << it->second.adopted_support_axis.x << ","
              << it->second.adopted_support_axis.y << "," << it->second.adopted_support_axis.z << " neighbors="
              << it->second.primary_neighbor_id << "/" << it->second.secondary_neighbor_id;
      trace.push_back({DecisionTraceTopic::kPoleOrientation, PoleForwardRuleText(it->second.rule), summary.str()});
      std::ostringstream support_summary;
      support_summary << "axis=" << it->second.adopted_support_axis.x << "," << it->second.adopted_support_axis.y << ","
                      << it->second.adopted_support_axis.z << " neighbors=" << it->second.primary_neighbor_id << "/"
                      << it->second.secondary_neighbor_id;
      trace.push_back(
          {DecisionTraceTopic::kSupportLayoutSelection, PoleSupportAxisRuleText(it->second.support_axis_rule),
           support_summary.str()});
    }
    if (const Pole* pole = poles().find(ref.stable_id); pole != nullptr && state_.has_pole_orientation_override(pole->id)) {
      std::ostringstream summary;
      summary << "autoYaw=";
      if (const auto it = pole_orientation_debug_records().find(ref.stable_id); it != pole_orientation_debug_records().end()) {
        summary << (std::atan2(it->second.adopted_forward.y, it->second.adopted_forward.x) * (180.0 / 3.14159265358979323846));
      } else {
        summary << "n/a";
      }
      summary << " manualYaw=";
      if (const auto manual_yaw = state_.resolve_pole_manual_yaw_override(*pole); manual_yaw.has_value()) {
        summary << *manual_yaw;
      } else {
        summary << "auto";
      }
      summary << " flip180=";
      if (const auto flip = state_.resolve_pole_flip_180_override(*pole); flip.has_value()) {
        summary << BoolText(*flip);
      } else {
        summary << "auto";
      }
      summary << " finalYaw=" << state_.effective_pole_yaw_deg(*pole);
      trace.push_back({DecisionTraceTopic::kOverrideResolution, "PoleOrientationOverride", summary.str()});
    }
    return trace;
  }

  if (ref.kind == EntityKind::kSpan || ref.kind == EntityKind::kSupportLayout || ref.kind == EntityKind::kDetailCurve) {
    const ObjectId span_id = static_cast<ObjectId>(ref.stable_id);
    bool has_flow_trace = false;
    if (const Span* span = spans().find(span_id); span != nullptr) {
      if (const SegmentLaneAssignment* assignment = FindLaneAssignmentForSpan(state_, *span); assignment != nullptr) {
        std::ostringstream summary;
        summary << "flow=" << FlowKindText(assignment->flow_kind)
                << " branchSupport=" << BoolText(assignment->uses_branch_support)
                << " mirrored=" << BoolText(assignment->mirrored);
        trace.push_back({DecisionTraceTopic::kFlowClassification, FlowDecisionRuleText(assignment->flow_decision_rule),
                         summary.str()});
        has_flow_trace = true;
      }
    }
    if (const SpanSupportLayoutEntry* layout = find_span_support_layout(span_id); layout != nullptr) {
      if (!has_flow_trace) {
        std::ostringstream flow_summary;
        flow_summary << "flow=" << FlowKindText(layout->flow_kind)
                     << " pass=" << static_cast<int>(layout->pass_mode)
                     << " flowKey=" << layout->variation_flow_key;
        trace.push_back(
            {DecisionTraceTopic::kFlowClassification, "SupportLayoutFlow", flow_summary.str()});
      }
      std::ostringstream summary;
      summary << "start=" << SupportLayoutOriginText(layout->start.origin) << " dep=" << layout->start.local_departure_length_m
              << " end=" << SupportLayoutOriginText(layout->end.origin) << " dep=" << layout->end.local_departure_length_m
              << " down=" << std::max(layout->start.branch_down_offset_m, layout->end.branch_down_offset_m);
      trace.push_back({DecisionTraceTopic::kSupportLayoutSelection, FlowKindText(layout->flow_kind), summary.str()});
      std::ostringstream endpoint_summary;
      endpoint_summary << "start=" << SupportLayoutEndpointSourceText(layout->start.endpoint_source)
                       << " input=" << (layout->start.attachment_input_present ? "attachment" : "none")
                       << " socket=" << layout->start.socket_id
                       << " override=" << BoolText(layout->start.socket_override_active)
                       << " end=" << SupportLayoutEndpointSourceText(layout->end.endpoint_source)
                       << " input=" << (layout->end.attachment_input_present ? "attachment" : "none")
                       << " socket=" << layout->end.socket_id
                       << " override=" << BoolText(layout->end.socket_override_active);
      trace.push_back(
          {DecisionTraceTopic::kSupportLayoutSelection, "AttachmentEndpointSelection", endpoint_summary.str()});
      if (state_.has_span_branch_down_offset_override(span_id)) {
        std::ostringstream override_summary;
        override_summary << "autoDown="
                         << std::max(layout->start.automatic_branch_down_offset_m, layout->end.automatic_branch_down_offset_m)
                         << " finalDown=" << std::max(layout->start.branch_down_offset_m, layout->end.branch_down_offset_m);
        trace.push_back({DecisionTraceTopic::kOverrideResolution, "BranchDownOffsetOverride", override_summary.str()});
      }
    }
    if (const CurveCacheEntry* curve = state_.find_curve_cache(span_id); curve != nullptr) {
      std::ostringstream tangent;
      tangent << "start=" << TangentRuleText(curve->detail.quality.start_tangent_rule) << " support/chord="
              << curve->detail.quality.start_support_weight << "/" << curve->detail.quality.start_chord_weight << " end="
              << TangentRuleText(curve->detail.quality.end_tangent_rule) << " support/chord="
              << curve->detail.quality.end_support_weight << "/" << curve->detail.quality.end_chord_weight;
      trace.push_back({DecisionTraceTopic::kTangentGeneration,
                       (curve->detail.start_constraint.pass_mode == CurvePassMode::kBranch) ? "BranchTangentRule"
                                                                                           : "MainTangentRule",
                       tangent.str()});

      std::ostringstream continuity;
      continuity << "requested=" << ContinuityPolicyText(curve->detail.quality.requested_policy)
                 << " adopted=" << ContinuityModeText(curve->detail.quality.adopted_continuity);
      trace.push_back({DecisionTraceTopic::kContinuitySelection, ContinuityReasonText(curve->detail.quality.continuity_reason),
                       continuity.str()});

      if (curve->detail.quality.degraded_to_g1) {
        std::ostringstream degrade;
        degrade << "attemptedG2=" << BoolText(curve->detail.quality.attempted_g2)
                << " fallbackIterations=" << curve->detail.quality.fallback_iterations;
        trace.push_back(
            {DecisionTraceTopic::kContinuityDegrade, ContinuityReasonText(curve->detail.quality.continuity_reason),
             degrade.str()});
      }

      std::ostringstream sag;
      sag << "amp=" << curve->detail.sag_amplitude_m << " base=" << curve->detail.quality.sag_base_ratio
          << " len=" << curve->detail.quality.sag_length_scale << " pass=" << curve->detail.quality.sag_pass_scale
          << " rigid=" << curve->detail.quality.sag_rigidity_scale;
      trace.push_back({DecisionTraceTopic::kSagProfile, "SagProfileRule", sag.str()});
    }
    if (const Span* span = spans().find(span_id); span != nullptr) {
      if (state_.has_span_endpoint_socket_override(span_id, true) || state_.has_span_endpoint_socket_override(span_id, false)) {
        std::ostringstream socket_summary;
        socket_summary << "socketA=" << state_.resolve_span_endpoint_socket_id(*span, true)
                       << " socketB=" << state_.resolve_span_endpoint_socket_id(*span, false);
        trace.push_back({DecisionTraceTopic::kOverrideResolution, "AttachmentSocketOverride", socket_summary.str()});
      }
    }
    return trace;
  }

  if (ref.kind == EntityKind::kJunction) {
    const auto junction = FindJunctionByNode(state_, static_cast<ObjectId>(ref.stable_id));
    if (junction.has_value()) {
      std::ostringstream summary;
      summary << "incidentCount=" << junction->incidents.size();
      for (const JunctionIncident& incident : junction->incidents) {
        summary << " [" << incident.neighbor_node_id << " order=" << incident.order
                << " primary=" << BoolText(incident.primary) << "]";
      }
      trace.push_back({DecisionTraceTopic::kFlowClassification, "JunctionPrimaryOrder", summary.str()});
    }
  }
  return trace;
}

} // namespace wire::core
