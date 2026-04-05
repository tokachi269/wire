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
enum class SupportLayoutOriginKind : std::uint8_t {
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

struct SupportLayoutSemanticDecision {
  ObjectId owner_pole_id = kInvalidObjectId;
  JunctionRelationKind relation_kind = JunctionRelationKind::kNone;
  ContinuityCategoryClass continuity_class = ContinuityCategoryClass::kPointLike;
  bool in_through_pair = false;
  ObjectId support_pair_peer_low = kInvalidObjectId;
  ObjectId support_pair_peer_high = kInvalidObjectId;
  int support_group_id = -1;
  bool lower_required = false;
  bool lowering_blocked_by_policy = false;
  OrderDecisionPolicyKind order_decision_policy = OrderDecisionPolicyKind::kFixedOrder;
  OrderDecisionChoiceKind order_decision_choice = OrderDecisionChoiceKind::kNormal;
  OrderDecisionChoiceReason order_decision_choice_reason = OrderDecisionChoiceReason::kFixedOrder;
  LateralSideChoiceKind chosen_side = LateralSideChoiceKind::kCenter;
  SideAssignmentRuleKind side_assignment_rule = SideAssignmentRuleKind::kPoleLocal;
  SupportOrientationRuleKind support_orientation_rule = SupportOrientationRuleKind::kRadial;
  SupportOrientationBasisKind support_orientation_basis = SupportOrientationBasisKind::kRadial;
  bool used_junction_pair_side_assignment = false;
  bool has_side_axis = false;
  Vec3d side_axis{};
  double chosen_side_sign = 0.0;

  SupportLayoutSemanticDecision() = default;
  SupportLayoutSemanticDecision(const EndpointContinuityDecision& source);
  SupportLayoutSemanticDecision& operator=(const EndpointContinuityDecision& source);
};

[[nodiscard]] inline SupportLayoutSemanticDecision MakeSupportLayoutSemanticDecision(
    const EndpointContinuityDecision& source, ObjectId owner_pole_id = kInvalidObjectId) {
  SupportLayoutSemanticDecision decision{};
  decision.owner_pole_id = (owner_pole_id == kInvalidObjectId) ? source.owner_pole_id : owner_pole_id;
  decision.relation_kind = source.relation_kind;
  decision.continuity_class = source.continuity_class;
  decision.in_through_pair = source.in_through_pair;
  decision.support_pair_peer_low = source.support_pair_peer_low;
  decision.support_pair_peer_high = source.support_pair_peer_high;
  decision.support_group_id = source.support_group_id;
  decision.lower_required = source.lower_required;
  decision.lowering_blocked_by_policy = source.lowering_blocked_by_policy;
  decision.order_decision_policy = source.order_decision_policy;
  decision.order_decision_choice = source.order_decision_choice;
  decision.order_decision_choice_reason = source.order_decision_choice_reason;
  decision.chosen_side = source.chosen_side;
  decision.side_assignment_rule = source.side_assignment_rule;
  decision.support_orientation_rule = source.support_orientation_rule;
  decision.support_orientation_basis = source.support_orientation_basis;
  decision.used_junction_pair_side_assignment = source.used_junction_pair_side_assignment;
  decision.has_side_axis = source.has_side_axis;
  decision.side_axis = source.side_axis;
  decision.chosen_side_sign = source.chosen_side_sign;
  return decision;
}

inline SupportLayoutSemanticDecision::SupportLayoutSemanticDecision(const EndpointContinuityDecision& source)
    : SupportLayoutSemanticDecision(MakeSupportLayoutSemanticDecision(source)) {}

inline SupportLayoutSemanticDecision&
SupportLayoutSemanticDecision::operator=(const EndpointContinuityDecision& source) {
  *this = MakeSupportLayoutSemanticDecision(source);
  return *this;
}

[[nodiscard]] inline LoweredSupportGroupKey LoweredSupportGroupKeyFromDecision(
    const SupportLayoutSemanticDecision& decision) {
  return {decision.owner_pole_id, decision.support_group_id};
}

[[nodiscard]] inline bool HasAuthoritativeSupportPair(const SupportLayoutSemanticDecision& decision) {
  return HasAuthoritativeSupportPair(decision.support_pair_peer_low, decision.support_pair_peer_high);
}

[[nodiscard]] inline bool UsesAuthoritativeGroupedLoweredSupport(const SupportLayoutSemanticDecision& decision) {
  return decision.owner_pole_id != kInvalidObjectId && decision.lower_required && !decision.lowering_blocked_by_policy &&
         decision.support_group_id >= 0;
}

struct SupportLayoutEndpoint : SupportLayoutSemanticDecision {
  ObjectId endpoint_node_id = kInvalidObjectId;
  ObjectId port_id = kInvalidObjectId;
  ResolvedSupportAuthority support_authority{};
  EndpointAttachmentRequest attachment_request{};
  std::optional<int> resolved_socket_id{};
  BackboneFlowKind flow_kind = BackboneFlowKind::kMain;
  SupportLayoutOriginKind origin = SupportLayoutOriginKind::kFallback;
  SupportLayoutEndpointSourceKind endpoint_source = SupportLayoutEndpointSourceKind::kFallback;
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
  HierarchicalVariationSample down_offset_variation{};
};

struct SupportLayoutDecisionSeedEndpoint : SupportLayoutSemanticDecision {
  ObjectId endpoint_node_id = kInvalidObjectId;
  ObjectId port_id = kInvalidObjectId;
  ResolvedSupportAuthority support_authority{};
  EndpointAttachmentRequest attachment_request{};
  std::optional<int> resolved_socket_id{};
  BackboneFlowKind flow_kind = BackboneFlowKind::kMain;
  SupportLayoutOriginKind origin = SupportLayoutOriginKind::kFallback;
  SupportLayoutEndpointSourceKind endpoint_source = SupportLayoutEndpointSourceKind::kFallback;
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
  HierarchicalVariationSample down_offset_variation{};
};

struct SupportGroupDecision : SupportLayoutSemanticDecision {
  ResolvedSupportAuthority support_authority{};
  ConnectionCategory category = ConnectionCategory::kLowVoltage;
  SlotSide side = SlotSide::kCenter;
  SupportLayoutOriginKind origin = SupportLayoutOriginKind::kFallback;
  double down_offset_m = 0.0;
  Vec3d support_world{};
  HierarchicalVariationSample down_offset_variation{};
  int grouped_port_count = 0;
  std::vector<Vec3d> attachment_worlds{};
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

struct SpanSupportLayoutEntry {
  ObjectId span_id = kInvalidObjectId;
  bool requires_decision_seed = false;
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
  SupportLayoutEndpoint start{};
  SupportLayoutEndpoint end{};
  std::vector<LoweredSupportGroupKey> lowered_support_group_keys{};
  std::unordered_map<LoweredSupportGroupKey, SupportGroupDecision, LoweredSupportGroupKeyHash> support_group_decisions{};
  std::uint64_t source_version = 0;
};

struct SupportLayoutCache {
  std::unordered_map<ObjectId, SpanSupportLayoutDecisionSeed> decision_seeds_by_span{};
  std::unordered_set<ObjectId> decision_required_span_ids{};
  std::unordered_map<ObjectId, SpanSupportLayoutEntry> by_span{};
  std::unordered_map<LoweredSupportGroupKey, SupportGroupDecision, LoweredSupportGroupKeyHash> support_group_decisions{};
  std::unordered_map<LoweredSupportGroupKey, LoweredSupportGroupPlacement, LoweredSupportGroupKeyHash>
      lowered_support_groups{};
};

} // namespace wire::core
