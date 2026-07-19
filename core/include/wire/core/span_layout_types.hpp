#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "wire/core/detail_curve.hpp"
#include "wire/core/endpoint_resolution.hpp"
#include "wire/core/id.hpp"
#include "wire/core/types.hpp"
#include "wire/core/variation.hpp"
#include "wire/core/workflow_types.hpp"

namespace wire::core {

// Derived span-layout boundary types consumed by geometry and inspection.
enum class LayoutOriginKind : std::uint8_t {
  kMainSupport = 0,
  kBranchSupport = 1,
  kAerialBranch = 2,
  kPlacementConstraint = 3,
  kFallback = 4,
};

struct LoweredSupportGroupKey {
  ObjectId owner_pole_id = kInvalidObjectId;
  int support_group_id = -1;

  [[nodiscard]] bool operator==(const LoweredSupportGroupKey& other) const {
    return owner_pole_id == other.owner_pole_id && support_group_id == other.support_group_id;
  }
};

[[nodiscard]] inline LoweredSupportGroupKey LoweredSupportGroupKeyFromDecision(
    const EndpointContinuityDecision& decision) {
  return {decision.owner_pole_id, decision.support_group_id};
}

struct LoweredSupportGroupKeyHash {
  [[nodiscard]] std::size_t operator()(const LoweredSupportGroupKey& key) const {
    const std::size_t h1 = std::hash<ObjectId>{}(key.owner_pole_id);
    const std::size_t h2 = std::hash<int>{}(key.support_group_id);
    return h1 ^ (h2 + 0x9e3779b9 + (h1 << 6) + (h1 >> 2));
  }
};

struct LayoutSemantic {
  ObjectId owner_pole_id = kInvalidObjectId;
  JunctionRelationKind relation_kind = JunctionRelationKind::kNone;
  ContinuityCategoryClass continuity_class = ContinuityCategoryClass::kPointLike;
  bool in_through_pair = false;
  ObjectId support_pair_peer_low = kInvalidObjectId;
  ObjectId support_pair_peer_high = kInvalidObjectId;
  int support_group_id = -1;
  bool lower_required = false;
  bool lowering_blocked_by_policy = false;
  SideAssignmentRuleKind side_assignment_rule = SideAssignmentRuleKind::kPoleLocal;
  SupportOrientationRuleKind support_orientation_rule = SupportOrientationRuleKind::kRadial;
  SupportOrientationBasisKind support_orientation_basis = SupportOrientationBasisKind::kRadial;
  bool has_side_axis = false;
  Vec3d side_axis{};
  double chosen_side_sign = 0.0;

  LayoutSemantic() = default;
  LayoutSemantic(const EndpointContinuityDecision& source);
  LayoutSemantic& operator=(const EndpointContinuityDecision& source);
};

[[nodiscard]] inline LayoutSemantic MakeLayoutSemantic(
    const EndpointContinuityDecision& source, ObjectId owner_pole_id = kInvalidObjectId) {
  LayoutSemantic decision{};
  decision.owner_pole_id = (owner_pole_id == kInvalidObjectId) ? source.owner_pole_id : owner_pole_id;
  decision.relation_kind = source.relation_kind;
  decision.continuity_class = source.continuity_class;
  decision.in_through_pair = source.in_through_pair;
  decision.support_pair_peer_low = source.support_pair_peer_low;
  decision.support_pair_peer_high = source.support_pair_peer_high;
  decision.support_group_id = source.support_group_id;
  decision.lower_required = source.lower_required;
  decision.lowering_blocked_by_policy = source.lowering_blocked_by_policy;
  decision.side_assignment_rule = source.side_assignment_rule;
  decision.support_orientation_rule = source.support_orientation_rule;
  decision.support_orientation_basis = source.support_orientation_basis;
  decision.has_side_axis = source.has_side_axis;
  decision.side_axis = source.side_axis;
  decision.chosen_side_sign = source.chosen_side_sign;
  return decision;
}

inline LayoutSemantic::LayoutSemantic(const EndpointContinuityDecision& source)
    : LayoutSemantic(MakeLayoutSemantic(source)) {}

inline LayoutSemantic& LayoutSemantic::operator=(const EndpointContinuityDecision& source) {
  *this = MakeLayoutSemantic(source);
  return *this;
}

[[nodiscard]] inline LoweredSupportGroupKey LoweredSupportGroupKeyFromDecision(
    const LayoutSemantic& decision) {
  return {decision.owner_pole_id, decision.support_group_id};
}

[[nodiscard]] inline bool HasAuthoritativeSupportPair(const LayoutSemantic& decision) {
  return HasAuthoritativeSupportPair(decision.support_pair_peer_low, decision.support_pair_peer_high);
}

[[nodiscard]] inline bool UsesAuthoritativeGroupedLoweredSupport(const LayoutSemantic& decision) {
  return decision.owner_pole_id != kInvalidObjectId && decision.lower_required && !decision.lowering_blocked_by_policy &&
         decision.support_group_id >= 0;
}

inline void CopyLayoutSemantic(LayoutSemantic& dst, const LayoutSemantic& src) {
  dst.owner_pole_id = src.owner_pole_id;
  dst.relation_kind = src.relation_kind;
  dst.continuity_class = src.continuity_class;
  dst.in_through_pair = src.in_through_pair;
  dst.support_pair_peer_low = src.support_pair_peer_low;
  dst.support_pair_peer_high = src.support_pair_peer_high;
  dst.support_group_id = src.support_group_id;
  dst.lower_required = src.lower_required;
  dst.lowering_blocked_by_policy = src.lowering_blocked_by_policy;
  dst.side_assignment_rule = src.side_assignment_rule;
  dst.support_orientation_rule = src.support_orientation_rule;
  dst.support_orientation_basis = src.support_orientation_basis;
  dst.has_side_axis = src.has_side_axis;
  dst.side_axis = src.side_axis;
  dst.chosen_side_sign = src.chosen_side_sign;
}

struct SourceEdgeProjectionRef {
  ObjectId source_edge_id = kInvalidObjectId;
  ObjectId source_edge_bundle_id = kInvalidObjectId;
  ObjectId from_node_id = kInvalidObjectId;
  BundleTemplateId bundle_template_id = kInvalidBundleTemplateId;
  std::size_t lane_index = 0;
  double t = 0.0;

  [[nodiscard]] bool valid() const noexcept {
    return source_edge_id != kInvalidObjectId && source_edge_bundle_id != kInvalidObjectId &&
           from_node_id != kInvalidObjectId;
  }
};

struct LayoutEndpoint : LayoutSemantic {
  ObjectId endpoint_node_id = kInvalidObjectId;
  ObjectId port_id = kInvalidObjectId;
  ObjectId jumper_peer_port_id = kInvalidObjectId;
  SourceEdgeProjectionRef source_projection{};
  EndpointAttachmentRequest attachment_request{};
  std::optional<int> resolved_socket_id{};
  BackboneFlowKind flow_kind = BackboneFlowKind::kMain;
  LayoutOriginKind origin = LayoutOriginKind::kFallback;
  LayoutEndpointSourceKind endpoint_source = LayoutEndpointSourceKind::kFallback;
  PortPlacementSourceKind port_source = PortPlacementSourceKind::kUnknown;
  SlotSide side = SlotSide::kCenter;
  CurveEndpointMode endpoint_mode = CurveEndpointMode::kDirectThrough;
  Vec3d support_world{};
  Vec3d endpoint_world{};
  Vec3d departure_dir{};
  Vec3d endpoint_offset{};
  double local_departure_length_m = 0.0;
  double automatic_branch_down_offset_m = 0.0;
  double branch_down_offset_m = 0.0;
  double automatic_endpoint_offset_z_m = 0.0;
  double endpoint_offset_z_m = 0.0;
  bool default_lower_required = false;
  bool same_level_feasible = true;
  bool unresolved_same_level_conflict = false;
  SameLevelFeasibilityReason same_level_reason = SameLevelFeasibilityReason::kNone;
  double projected_spacing_topview_m = -1.0;
  double required_clearance_m = 0.0;
  bool solver_used_same_level_constraint = false;
  bool used_special_case_ports = false;
  OrderDecisionPolicyKind order_decision_policy = OrderDecisionPolicyKind::kFixedOrder;
  OrderDecisionChoiceKind order_decision_choice = OrderDecisionChoiceKind::kNormal;
  OrderDecisionChoiceReason order_decision_choice_reason = OrderDecisionChoiceReason::kFixedOrder;
  LateralSideChoiceKind chosen_side = LateralSideChoiceKind::kCenter;
  bool used_junction_pair_side_assignment = false;
  HierarchicalVariationSample down_offset_variation{};
};

struct SupportGroupDecision : LayoutSemantic {
  SlotSide side = SlotSide::kCenter;
  LayoutOriginKind origin = LayoutOriginKind::kFallback;
  OrderDecisionPolicyKind order_decision_policy = OrderDecisionPolicyKind::kFixedOrder;
  OrderDecisionChoiceKind order_decision_choice = OrderDecisionChoiceKind::kNormal;
  OrderDecisionChoiceReason order_decision_choice_reason = OrderDecisionChoiceReason::kFixedOrder;
  LateralSideChoiceKind chosen_side = LateralSideChoiceKind::kCenter;
  bool used_junction_pair_side_assignment = false;
};

struct EndpointLayoutRule {
  ObjectId endpoint_node_id = kInvalidObjectId;
  ObjectId port_id = kInvalidObjectId;
  ObjectId jumper_peer_port_id = kInvalidObjectId;
  SourceEdgeProjectionRef source_projection{};
  LayoutSemantic semantic{};
  EndpointAttachmentRequest attachment_request{};
  std::optional<int> resolved_socket_id{};
  BackboneFlowKind flow_kind = BackboneFlowKind::kMain;
  LayoutOriginKind origin = LayoutOriginKind::kFallback;
  LayoutEndpointSourceKind endpoint_source = LayoutEndpointSourceKind::kFallback;
  PortPlacementSourceKind port_source = PortPlacementSourceKind::kUnknown;
  SlotSide side = SlotSide::kCenter;
  CurveEndpointMode endpoint_mode = CurveEndpointMode::kDirectThrough;
  double automatic_branch_down_offset_m = 0.0;
  double branch_down_offset_m = 0.0;
  double automatic_endpoint_offset_z_m = 0.0;
  double endpoint_offset_z_m = 0.0;
  bool default_lower_required = false;
  bool same_level_feasible = true;
  bool unresolved_same_level_conflict = false;
  SameLevelFeasibilityReason same_level_reason = SameLevelFeasibilityReason::kNone;
  double projected_spacing_topview_m = -1.0;
  double required_clearance_m = 0.0;
  bool solver_used_same_level_constraint = false;
  bool used_special_case_ports = false;
  OrderDecisionPolicyKind order_decision_policy = OrderDecisionPolicyKind::kFixedOrder;
  OrderDecisionChoiceKind order_decision_choice = OrderDecisionChoiceKind::kNormal;
  OrderDecisionChoiceReason order_decision_choice_reason = OrderDecisionChoiceReason::kFixedOrder;
  LateralSideChoiceKind chosen_side = LateralSideChoiceKind::kCenter;
  bool used_junction_pair_side_assignment = false;
  HierarchicalVariationSample down_offset_variation{};
};

inline void ApplyEndpointLayoutRule(LayoutEndpoint& dst, const EndpointLayoutRule& rule,
                                    const Vec3d& port_world_position) {
  CopyLayoutSemantic(dst, rule.semantic);
  dst.endpoint_node_id = rule.endpoint_node_id;
  dst.port_id = rule.port_id;
  dst.jumper_peer_port_id = rule.jumper_peer_port_id;
  dst.source_projection = rule.source_projection;
  dst.attachment_request = rule.attachment_request;
  dst.resolved_socket_id = rule.resolved_socket_id;
  dst.flow_kind = rule.flow_kind;
  dst.origin = rule.origin;
  dst.endpoint_source = rule.endpoint_source;
  dst.port_source = rule.port_source;
  dst.side = rule.side;
  dst.endpoint_mode = rule.endpoint_mode;
  dst.automatic_branch_down_offset_m = rule.automatic_branch_down_offset_m;
  dst.branch_down_offset_m = rule.branch_down_offset_m;
  dst.automatic_endpoint_offset_z_m = rule.automatic_endpoint_offset_z_m;
  dst.endpoint_offset_z_m = rule.endpoint_offset_z_m;
  dst.default_lower_required = rule.default_lower_required;
  dst.same_level_feasible = rule.same_level_feasible;
  dst.unresolved_same_level_conflict = rule.unresolved_same_level_conflict;
  dst.same_level_reason = rule.same_level_reason;
  dst.projected_spacing_topview_m = rule.projected_spacing_topview_m;
  dst.required_clearance_m = rule.required_clearance_m;
  dst.solver_used_same_level_constraint = rule.solver_used_same_level_constraint;
  dst.used_special_case_ports = rule.used_special_case_ports;
  dst.order_decision_policy = rule.order_decision_policy;
  dst.order_decision_choice = rule.order_decision_choice;
  dst.order_decision_choice_reason = rule.order_decision_choice_reason;
  dst.chosen_side = rule.chosen_side;
  dst.used_junction_pair_side_assignment = rule.used_junction_pair_side_assignment;
  dst.down_offset_variation = rule.down_offset_variation;
  dst.support_world = port_world_position;
  dst.endpoint_world = port_world_position;
  if (rule.default_lower_required || rule.semantic.lower_required) {
    const double endpoint_offset = rule.endpoint_offset_z_m != 0.0 ? rule.endpoint_offset_z_m
                                                                   : rule.automatic_endpoint_offset_z_m;
    dst.endpoint_offset_z_m = endpoint_offset;
    dst.automatic_endpoint_offset_z_m = rule.automatic_endpoint_offset_z_m;
    dst.branch_down_offset_m = rule.branch_down_offset_m != 0.0 ? rule.branch_down_offset_m
                                                                : std::max(0.0, -endpoint_offset);
    dst.automatic_branch_down_offset_m = rule.automatic_branch_down_offset_m != 0.0
                                             ? rule.automatic_branch_down_offset_m
                                             : std::max(0.0, -rule.automatic_endpoint_offset_z_m);
  }
  dst.departure_dir = {1.0, 0.0, 0.0};
}

struct SpanLayoutRule {
  ObjectId span_id = kInvalidObjectId;
  BackboneFlowKind flow_kind = BackboneFlowKind::kMain;
  CurvePassMode pass_mode = CurvePassMode::kPassThrough;
  std::uint64_t variation_flow_key = 0;
  BackboneLoweringKind lowering_kind = BackboneLoweringKind::kNone;
  EndpointLayoutRule start{};
  EndpointLayoutRule end{};
  std::unordered_map<LoweredSupportGroupKey, SupportGroupDecision, LoweredSupportGroupKeyHash> support_group_rules{};
};

struct SpanLayoutRules {
  std::vector<SpanLayoutRule> spans{};
};

struct SpanLayoutRulesView {
  const SpanLayoutRule* rule = nullptr;

  [[nodiscard]] bool has_rule() const { return rule != nullptr; }
};

struct SpanLayoutEntry;

struct SpanLayoutView {
  const SpanLayoutEntry* entry = nullptr;

  [[nodiscard]] bool has_layout() const { return entry != nullptr; }
};

struct SpanLayoutState {
  bool has_rules = false;
  bool has_layout = false;
};

struct LoweredSupportGroupPlacement {
  // Placement output. Semantic decision lives in SupportGroupDecision.
  SupportGroupingRuleKind grouping_rule = SupportGroupingRuleKind::kDecisionGroup;
  int grouped_port_count = 1;
  double down_offset_m = 0.0;
  Vec3d mount_world{};
  Vec3d tip_world{};
  std::vector<Vec3d> attachment_worlds{};
  HierarchicalVariationSample down_offset_variation{};
};

struct SpanLayoutEntry {
  ObjectId span_id = kInvalidObjectId;
  BackboneFlowKind flow_kind = BackboneFlowKind::kMain;
  CurvePassMode pass_mode = CurvePassMode::kPassThrough;
  CurveProfileHint detail_curve_profile_hint = CurveProfileHint::kAuto;
  double basis_length_m = 0.0;
  double effective_sag_ratio = 0.0;
  CableContinuityPolicyHint continuity_preference = CableContinuityPolicyHint::kAuto;
  double bend_stiffness_hint = 1.0;
  double min_bend_radius_hint_m = 0.0;
  std::uint64_t variation_flow_key = 0;
  HierarchicalVariationSample sag_variation{};
  BackboneLoweringKind lowering_kind = BackboneLoweringKind::kNone;
  LayoutEndpoint start{};
  LayoutEndpoint end{};
  std::vector<LoweredSupportGroupKey> lowered_support_group_keys{};
  std::uint64_t source_version = 0;
};

struct SpanLayoutCacheRecord {
  std::optional<SpanLayoutEntry> layout{};
  std::optional<SpanLayoutRule> rule{};

  [[nodiscard]] bool has_layout() const { return layout.has_value(); }
  [[nodiscard]] bool has_rule() const { return rule.has_value(); }
  [[nodiscard]] const SpanLayoutEntry* span_layout() const {
    return layout.has_value() ? &*layout : nullptr;
  }
  [[nodiscard]] SpanLayoutEntry* span_layout() {
    return layout.has_value() ? &*layout : nullptr;
  }
  [[nodiscard]] const SpanLayoutRule* span_layout_rule() const {
    return rule.has_value() ? &*rule : nullptr;
  }

  void store_layout(SpanLayoutEntry layout_value) { layout = std::move(layout_value); }
  void clear_layout() { layout.reset(); }
  void store_rule(SpanLayoutRule rule_value) { rule = std::move(rule_value); }
};

struct SupportGroupDecisionCache {
  std::unordered_map<LoweredSupportGroupKey, SupportGroupDecision, LoweredSupportGroupKeyHash> by_key{};
};

struct SupportGroupPlacementCache {
  std::unordered_map<LoweredSupportGroupKey, LoweredSupportGroupPlacement, LoweredSupportGroupKeyHash> by_key{};
};

struct SupportGroupCache {
  SupportGroupDecisionCache decision{};
  SupportGroupPlacementCache placement{};
};

struct SpanLayoutCache {
  std::unordered_map<ObjectId, SpanLayoutCacheRecord> records_by_span{};
  SupportGroupCache support_groups{};

  [[nodiscard]] SpanLayoutRulesView rules_view(ObjectId span_id) const {
    const auto it = records_by_span.find(span_id);
    if (it == records_by_span.end()) {
      return {};
    }
    return {it->second.span_layout_rule()};
  }

  [[nodiscard]] SpanLayoutView layout_view(ObjectId span_id) const {
    const auto it = records_by_span.find(span_id);
    if (it == records_by_span.end()) {
      return {};
    }
    return {it->second.span_layout()};
  }

  [[nodiscard]] SpanLayoutState layout_state(ObjectId span_id) const {
    const auto it = records_by_span.find(span_id);
    if (it == records_by_span.end()) {
      return {};
    }
    const SpanLayoutCacheRecord& record = it->second;
    return {record.span_layout_rule() != nullptr, record.span_layout() != nullptr};
  }

  template <typename Fn>
  void for_each_layout_record(Fn&& visitor) const {
    for (const auto& [span_id, record] : records_by_span) {
      if (const SpanLayoutEntry* layout = record.span_layout(); layout != nullptr) {
        visitor(span_id, record, *layout);
      }
    }
  }

  template <typename Fn>
  void for_each_layout_record(Fn&& visitor) {
    for (auto& [span_id, record] : records_by_span) {
      if (SpanLayoutEntry* layout = record.span_layout(); layout != nullptr) {
        visitor(span_id, record, *layout);
      }
    }
  }

  void erase_record_if_empty(ObjectId span_id) {
    const auto it = records_by_span.find(span_id);
    if (it == records_by_span.end()) {
      return;
    }
    if (!it->second.has_layout() && !it->second.has_rule()) {
      records_by_span.erase(it);
    }
  }

  void store_layout(SpanLayoutEntry layout) {
    const ObjectId span_id = layout.span_id;
    SpanLayoutCacheRecord& record = records_by_span[span_id];
    record.store_layout(std::move(layout));
  }

  void store_rules(const SpanLayoutRules& rules) {
    for (const SpanLayoutRule& rule : rules.spans) {
      if (rule.span_id == kInvalidObjectId) {
        continue;
      }
      records_by_span[rule.span_id].store_rule(rule);
    }
  }

  void clear_layout(ObjectId span_id) {
    const auto it = records_by_span.find(span_id);
    if (it == records_by_span.end()) {
      return;
    }
    it->second.clear_layout();
    erase_record_if_empty(span_id);
  }
};

} // namespace wire::core
