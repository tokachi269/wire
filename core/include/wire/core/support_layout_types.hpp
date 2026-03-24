#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <unordered_map>
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

struct SupportLayoutEndpoint {
  ObjectId endpoint_node_id = kInvalidObjectId;
  ObjectId owner_pole_id = kInvalidObjectId;
  ObjectId port_id = kInvalidObjectId;
  EndpointContinuityDecision decision{};
  EndpointAttachmentRequest attachment_request{};
  std::optional<int> resolved_socket_id{};
  BackboneFlowKind flow_kind = BackboneFlowKind::kMain;
  SupportLayoutOriginKind origin = SupportLayoutOriginKind::kFallback;
  SupportLayoutEndpointSourceKind endpoint_source = SupportLayoutEndpointSourceKind::kFallback;
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
  HierarchicalVariationSample down_offset_variation{};
};

struct SupportLayoutDecisionSeedEndpoint {
  ObjectId endpoint_node_id = kInvalidObjectId;
  ObjectId owner_pole_id = kInvalidObjectId;
  ObjectId port_id = kInvalidObjectId;
  EndpointContinuityDecision decision{};
  EndpointAttachmentRequest attachment_request{};
  std::optional<int> resolved_socket_id{};
  BackboneFlowKind flow_kind = BackboneFlowKind::kMain;
  SupportLayoutOriginKind origin = SupportLayoutOriginKind::kFallback;
  SupportLayoutEndpointSourceKind endpoint_source = SupportLayoutEndpointSourceKind::kFallback;
  PortPlacementSourceKind port_source = PortPlacementSourceKind::kUnknown;
  SlotSide side = SlotSide::kCenter;
  CurveEndpointMode endpoint_mode = CurveEndpointMode::kDirectThrough;
  double automatic_branch_down_offset_m = 0.0;
  double branch_down_offset_m = 0.0;
  HierarchicalVariationSample down_offset_variation{};
};

struct SpanSupportLayoutDecisionSeed {
  ObjectId span_id = kInvalidObjectId;
  BackboneFlowKind flow_kind = BackboneFlowKind::kMain;
  CurvePassMode pass_mode = CurvePassMode::kPassThrough;
  std::uint64_t variation_flow_key = 0;
  BackboneLoweringKind lowering_kind = BackboneLoweringKind::kNone;
  SupportLayoutDecisionSeedEndpoint start{};
  SupportLayoutDecisionSeedEndpoint end{};
};

struct SupportGroupDecision {
  EndpointContinuityDecision decision{};
  SlotSide side = SlotSide::kCenter;
  SupportLayoutOriginKind origin = SupportLayoutOriginKind::kFallback;
  double down_offset_m = 0.0;
  Vec3d support_world{};
  HierarchicalVariationSample down_offset_variation{};
  int grouped_port_count = 0;
  std::vector<Vec3d> attachment_worlds{};
};

struct LoweredSupportGroupPlacement {
  EndpointContinuityDecision decision{};
  SlotSide side = SlotSide::kCenter;
  SupportLayoutOriginKind origin = SupportLayoutOriginKind::kFallback;
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
  std::uint64_t source_version = 0;
};

struct SupportLayoutCache {
  std::unordered_map<ObjectId, SpanSupportLayoutDecisionSeed> decision_seeds_by_span{};
  std::unordered_map<ObjectId, SpanSupportLayoutEntry> by_span{};
  std::unordered_map<LoweredSupportGroupKey, SupportGroupDecision, LoweredSupportGroupKeyHash> support_group_decisions{};
  std::unordered_map<LoweredSupportGroupKey, LoweredSupportGroupPlacement, LoweredSupportGroupKeyHash>
      lowered_support_groups{};
};

} // namespace wire::core
