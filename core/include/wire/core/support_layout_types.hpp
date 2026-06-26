#pragma once

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

// Materialization-stage boundary types. Decision authors fill these from
// workflow_types, and downstream geometry / inspection consumes them.
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

struct JunctionPairAuthority {
  ObjectId pair_peer_low = kInvalidObjectId;
  ObjectId pair_peer_high = kInvalidObjectId;
  SupportOrientationBasisKind orientation_basis = SupportOrientationBasisKind::kRadial;
  Vec3d pair_axis{};
  bool has_pair_axis = false;
  int height_rank = -1;
};

struct ResolvedSupportAuthority {
  JunctionPairAuthority pair{};
  Vec3d signed_support_axis{};
  bool has_signed_support_axis = false;
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

struct LayoutEndpoint : LayoutSemantic {
  ObjectId endpoint_node_id = kInvalidObjectId;
  ObjectId port_id = kInvalidObjectId;
  ResolvedSupportAuthority support_authority{};
  EndpointAttachmentRequest attachment_request{};
  std::optional<int> resolved_socket_id{};
  BackboneFlowKind flow_kind = BackboneFlowKind::kMain;
  LayoutOriginKind origin = LayoutOriginKind::kFallback;
  LayoutEndpointSourceKind endpoint_source = LayoutEndpointSourceKind::kFallback;
  PortPlacementSourceKind port_source = PortPlacementSourceKind::kUnknown;
  SlotSide side = SlotSide::kCenter;
  CurveEndpointMode endpoint_mode = CurveEndpointMode::kDirectThrough;
  bool has_visual_arm_geometry = false;
  Vec3d visual_arm_mount_world{};
  Vec3d visual_arm_tip_world{};
  Vec3d visual_insulator_base_world{};
  Vec3d support_world{};
  Vec3d endpoint_world{};
  Vec3d departure_dir{};
  Vec3d endpoint_offset{};
  double local_departure_length_m = 0.0;
  double automatic_branch_down_offset_m = 0.0;
  double branch_down_offset_m = 0.0;
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

struct SupportLayoutDecisionSeedEndpoint : LayoutSemantic {
  ObjectId endpoint_node_id = kInvalidObjectId;
  ObjectId port_id = kInvalidObjectId;
  ResolvedSupportAuthority support_authority{};
  EndpointAttachmentRequest attachment_request{};
  std::optional<int> resolved_socket_id{};
  BackboneFlowKind flow_kind = BackboneFlowKind::kMain;
  LayoutOriginKind origin = LayoutOriginKind::kFallback;
  LayoutEndpointSourceKind endpoint_source = LayoutEndpointSourceKind::kFallback;
  PortPlacementSourceKind port_source = PortPlacementSourceKind::kUnknown;
  SlotSide side = SlotSide::kCenter;
  CurveEndpointMode endpoint_mode = CurveEndpointMode::kDirectThrough;
  bool has_visual_arm_geometry = false;
  Vec3d visual_arm_mount_world{};
  Vec3d visual_arm_tip_world{};
  Vec3d visual_insulator_base_world{};
  double automatic_branch_down_offset_m = 0.0;
  double branch_down_offset_m = 0.0;
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
  ResolvedSupportAuthority support_authority{};
  SlotSide side = SlotSide::kCenter;
  LayoutOriginKind origin = LayoutOriginKind::kFallback;
  OrderDecisionPolicyKind order_decision_policy = OrderDecisionPolicyKind::kFixedOrder;
  OrderDecisionChoiceKind order_decision_choice = OrderDecisionChoiceKind::kNormal;
  OrderDecisionChoiceReason order_decision_choice_reason = OrderDecisionChoiceReason::kFixedOrder;
  LateralSideChoiceKind chosen_side = LateralSideChoiceKind::kCenter;
  bool used_junction_pair_side_assignment = false;
};

struct SpanSupportLayoutDecisionSeed {
  ObjectId span_id = kInvalidObjectId;
  BackboneFlowKind flow_kind = BackboneFlowKind::kMain;
  CurvePassMode pass_mode = CurvePassMode::kPassThrough;
  std::uint64_t variation_flow_key = 0;
  BackboneLoweringKind lowering_kind = BackboneLoweringKind::kNone;
  SupportLayoutDecisionSeedEndpoint start{};
  SupportLayoutDecisionSeedEndpoint end{};
  std::unordered_map<LoweredSupportGroupKey, SupportGroupDecision, LoweredSupportGroupKeyHash> support_group_decisions{};
};

struct EndpointLayoutRule {
  ObjectId endpoint_node_id = kInvalidObjectId;
  ObjectId port_id = kInvalidObjectId;
  LayoutSemantic semantic{};
  ResolvedSupportAuthority support_authority{};
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
  bool input_required = false;
};

struct LoweredSupportGroupPlacement {
  // Geometry/materialization only. Semantic authority lives in SupportGroupDecision.
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

struct SpanSupportLayoutAuthorityRecord {
  bool required = false;
  std::optional<SpanSupportLayoutDecisionSeed> seed{};

  [[nodiscard]] bool has_seed() const { return seed.has_value(); }
  [[nodiscard]] const SpanSupportLayoutDecisionSeed* value() const {
    return seed.has_value() ? &*seed : nullptr;
  }
  [[nodiscard]] SpanSupportLayoutDecisionSeed* value() {
    return seed.has_value() ? &*seed : nullptr;
  }
};

struct SpanSupportLayoutProjectionRecord {
  std::optional<SpanLayoutEntry> layout{};

  [[nodiscard]] bool has_layout() const { return layout.has_value(); }
  [[nodiscard]] const SpanLayoutEntry* value() const {
    return layout.has_value() ? &*layout : nullptr;
  }
  [[nodiscard]] SpanLayoutEntry* value() {
    return layout.has_value() ? &*layout : nullptr;
  }
};

struct SupportLayoutCacheRecord {
  SpanSupportLayoutAuthorityRecord authority{};
  SpanSupportLayoutProjectionRecord projection{};
  std::optional<SpanLayoutRule> saved_rule{};

  [[nodiscard]] bool requires_authority() const { return authority.required; }
  [[nodiscard]] bool has_authority() const { return authority.has_seed(); }
  [[nodiscard]] bool has_projection() const { return projection.has_layout(); }
  [[nodiscard]] bool has_rule() const { return saved_rule.has_value(); }
  [[nodiscard]] const SpanSupportLayoutDecisionSeed* authority_seed() const { return authority.value(); }
  [[nodiscard]] SpanSupportLayoutDecisionSeed* authority_seed() { return authority.value(); }
  [[nodiscard]] const SpanLayoutEntry* projected_layout() const { return projection.value(); }
  [[nodiscard]] SpanLayoutEntry* projected_layout() { return projection.value(); }
  [[nodiscard]] const SpanLayoutRule* span_layout_rule() const {
    return saved_rule.has_value() ? &*saved_rule : nullptr;
  }

  void require_authority(bool required_value = true) { authority.required = required_value; }
  void store_authority(SpanSupportLayoutDecisionSeed seed_value) {
    authority.required = true;
    authority.seed = std::move(seed_value);
  }
  void clear_authority() { authority.seed.reset(); }
  void store_projection(SpanLayoutEntry layout_value) { projection.layout = std::move(layout_value); }
  void clear_projection() { projection.layout.reset(); }
  void store_rule(SpanLayoutRule rule_value) { saved_rule = std::move(rule_value); }
};

struct SpanSupportLayoutAuthorityView {
  const SpanSupportLayoutDecisionSeed* seed = nullptr;
  bool required = false;

  [[nodiscard]] bool has_authority() const { return seed != nullptr; }
};

struct SupportGroupAuthorityCache {
  std::unordered_map<LoweredSupportGroupKey, SupportGroupDecision, LoweredSupportGroupKeyHash> by_key{};
};

struct SupportGroupPlacementCache {
  std::unordered_map<LoweredSupportGroupKey, LoweredSupportGroupPlacement, LoweredSupportGroupKeyHash> by_key{};
};

struct SupportGroupCacheContract {
  SupportGroupAuthorityCache authority{};
  SupportGroupPlacementCache placement{};
};

struct SpanLayoutCache {
  std::unordered_map<ObjectId, SupportLayoutCacheRecord> records_by_span{};
  // Derived support-group cache. Authority is rebuilt from span decision seeds.
  // Placement is rebuilt from authority + observation. Neither is an authoring source.
  SupportGroupCacheContract support_groups{};

  [[nodiscard]] SpanSupportLayoutAuthorityView authority_view(ObjectId span_id) const {
    const auto it = records_by_span.find(span_id);
    if (it == records_by_span.end()) {
      return {};
    }
    const SupportLayoutCacheRecord& record = it->second;
    return {record.authority_seed(), record.requires_authority()};
  }

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
    return {it->second.projected_layout()};
  }

  [[nodiscard]] SpanLayoutState layout_state(ObjectId span_id) const {
    const auto it = records_by_span.find(span_id);
    if (it == records_by_span.end()) {
      return {};
    }
    const SupportLayoutCacheRecord& record = it->second;
    return {record.span_layout_rule() != nullptr, record.projected_layout() != nullptr, record.requires_authority()};
  }

  [[nodiscard]] std::vector<ObjectId> ordered_authority_span_ids() const {
    std::vector<ObjectId> span_ids{};
    span_ids.reserve(records_by_span.size());
    for (const auto& [span_id, record] : records_by_span) {
      if (record.has_authority()) {
        span_ids.push_back(span_id);
      }
    }
    std::sort(span_ids.begin(), span_ids.end());
    return span_ids;
  }

  template <typename Fn>
  void for_each_authority_seed(Fn&& visitor) const {
    for (ObjectId span_id : ordered_authority_span_ids()) {
      const SpanSupportLayoutAuthorityView authority = authority_view(span_id);
      if (!authority.has_authority()) {
        continue;
      }
      visitor(span_id, authority, *authority.seed);
    }
  }

  template <typename Fn>
  void for_each_projected_record(Fn&& visitor) const {
    for (const auto& [span_id, record] : records_by_span) {
      if (const SpanLayoutEntry* layout = record.projected_layout(); layout != nullptr) {
        visitor(span_id, record, *layout);
      }
    }
  }

  template <typename Fn>
  void for_each_projected_record(Fn&& visitor) {
    for (auto& [span_id, record] : records_by_span) {
      if (SpanLayoutEntry* layout = record.projected_layout(); layout != nullptr) {
        visitor(span_id, record, *layout);
      }
    }
  }

  void require_authority(ObjectId span_id) {
    records_by_span[span_id].require_authority();
  }

  void erase_record_if_empty(ObjectId span_id) {
    const auto it = records_by_span.find(span_id);
    if (it == records_by_span.end()) {
      return;
    }
    if (!it->second.requires_authority() && !it->second.has_authority() && !it->second.has_projection() &&
        !it->second.has_rule()) {
      records_by_span.erase(it);
    }
  }

  void store_layout(SpanLayoutEntry layout) {
    const ObjectId span_id = layout.span_id;
    SupportLayoutCacheRecord& record = records_by_span[span_id];
    record.store_projection(std::move(layout));
  }

  void store_seed(SpanSupportLayoutDecisionSeed seed) {
    const ObjectId span_id = seed.span_id;
    SupportLayoutCacheRecord& record = records_by_span[span_id];
    record.store_authority(std::move(seed));
  }

  void store_rules(const SpanLayoutRules& rules) {
    for (const SpanLayoutRule& rule : rules.spans) {
      if (rule.span_id == kInvalidObjectId) {
        continue;
      }
      records_by_span[rule.span_id].store_rule(rule);
    }
  }

  void clear_seed(ObjectId span_id) {
    const auto it = records_by_span.find(span_id);
    if (it == records_by_span.end()) {
      return;
    }
    it->second.clear_authority();
    erase_record_if_empty(span_id);
  }

  void clear_layout(ObjectId span_id) {
    const auto it = records_by_span.find(span_id);
    if (it == records_by_span.end()) {
      return;
    }
    it->second.clear_projection();
    erase_record_if_empty(span_id);
  }
};

using SupportLayoutCache = SpanLayoutCache;

} // namespace wire::core
