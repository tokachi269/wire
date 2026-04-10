#pragma once

#include "wire/core/core_state.hpp"
#include "backbone_pole_orientation_policy.hpp"
#include "backbone_prepare.hpp"

#include <functional>
#include <limits>
#include <string>
#include <unordered_map>

namespace wire::core::generation::detail {

struct BackboneBundlePlan {
  BundleKind template_id = BundleKind::kLowVoltage;
  ConnectionCategory category = ConnectionCategory::kLowVoltage;
  SpanLayer layer = SpanLayer::kUnknown;
  int count = 1;
  double spacing_m = 0.2;
  ContinuityCategoryClass continuity_class = ContinuityCategoryClass::kPointLike;
  OrderDecisionPolicyKind order_decision_policy = OrderDecisionPolicyKind::kFixedOrder;
  bool allow_mirror = true;
  bool preserve_conductor_identity = false;
  bool allow_midair_node = true;
  bool allow_midair_branch = true;
  bool enable_branch_down_offset = false;
};

struct EdgeFlowInfo {
  BackboneFlowKind kind = BackboneFlowKind::kMain;
  BackboneFlowDecisionRule rule = BackboneFlowDecisionRule::kDefaultMain;
  bool underpass_at_cross = false;
};

struct BackboneDecisionPhaseOutput {
  std::vector<EdgeFlowInfo> edge_flow_by_segment{};
  std::vector<JunctionRelation> junction_relations_in_path_order{};
  std::unordered_map<ObjectId, JunctionRelation> junction_relations_by_node{};
};

struct BackboneFeasibilityPhaseOutput {
  std::unordered_map<ObjectId, JunctionFeasibility> feasibility_by_node{};
};

struct BackboneBundleGapSegment {
  std::size_t segment_index = 0;
  int missing_count = 0;
};

struct BackboneBundleGapAnalysis {
  BackboneBundlePlan bundle_plan{};
  int missing_total = 0;
  std::size_t first_missing_segment = std::numeric_limits<std::size_t>::max();
  std::vector<BackboneBundleGapSegment> missing_segments{};

  [[nodiscard]] bool requires_allocation() const {
    return missing_total > 0;
  }

  [[nodiscard]] bool has_generation_start() const {
    return first_missing_segment != std::numeric_limits<std::size_t>::max();
  }
};

struct BackboneBundleAllocation {
  BackboneBundlePlan bundle_plan{};
  ObjectId bundle_id = kInvalidObjectId;
  ChangeSet change_set{};
};

struct BackboneSpanGenerationRunPlan {
  std::size_t segment_start_index = 0;
  std::size_t segment_end_index = 0;
  std::vector<ObjectId> ordered_support_node_ids{};
  std::uint64_t variation_flow_key = 0;
  BackboneLoweringPolicy lowering_policy{};
};

struct BackboneGroupedSpanGenerationPhaseOutput {
  ChangeSet change_set{};
  std::vector<ObjectId> span_ids{};
  std::vector<SegmentLaneAssignment> lane_assignments{};
  std::vector<BackboneEdgeOrientation> edge_orientations{};
  std::unordered_map<ObjectId, JunctionRelation> junction_relations_by_node{};
};

struct BackboneSupportLayoutSeedAuthorityPhaseOutput {
  std::vector<SpanSupportLayoutDecisionSeed> support_layout_seeds{};
};

struct BackboneGeneratedSpanMetadataPhaseOutput {
  ChangeSet change_set{};
  std::vector<ObjectId> generated_span_ids{};
};

struct BackboneSpanMaterializationPhaseOutput {
  ChangeSet change_set{};
  std::vector<ObjectId> generated_span_ids{};
  std::vector<SegmentLaneAssignment> lane_assignments{};
  std::vector<BackboneEdgeOrientation> edge_orientations{};
  std::vector<SpanSupportLayoutDecisionSeed> support_layout_seeds{};
  std::unordered_map<ObjectId, JunctionRelation> junction_relations_by_node{};
};

struct BackboneMaterializationPhaseOutput {
  ChangeSet change_set{};
  ObjectId primary_bundle_id = kInvalidObjectId;
  std::vector<ObjectId> bundle_ids{};
  std::vector<ObjectId> generated_span_ids{};
  std::vector<SegmentLaneAssignment> lane_assignments{};
  std::vector<BackboneEdgeOrientation> edge_orientations{};
  std::unordered_map<ObjectId, JunctionRelation> junction_relations_by_node{};
};

struct BackboneGenerationRequestPlan {
  BackboneSpec request{};
  NodeSpecByIndex node_spec_by_index{};
  NodeBundleModeByPoint node_bundle_mode_by_point{};
  std::vector<BackboneBundlePlan> bundle_plans{};
  std::vector<BackboneBundlePlan> active_bundle_plans{};
  std::vector<Vec3d> guide_points{};
  PathDirectionEvaluationDebug direction_debug{};
  std::vector<SupportNodeCandidate> candidates{};
  bool request_has_non_pole_points = false;
  bool request_has_source_edge_branch_points = false;
};

enum class BackboneSupportResolutionKind {
  kReusePole,
  kReuseSupportNode,
  kCreatePole,
  kCreateVirtualSupport,
};

struct BackboneSupportResolution {
  BackboneSupportResolutionKind kind = BackboneSupportResolutionKind::kReusePole;
  int candidate_index = -1;
  ObjectId planned_node_id = kInvalidObjectId;
  ObjectId existing_node_id = kInvalidObjectId;
  SupportKind support_kind = SupportKind::kPole;
  PlacementMode placement_mode = PlacementMode::kAuto;
  Transformd authored_transform{};
  PoleContextInfo authored_context{};
};

struct PlannedPoleCreation {
  ObjectId planned_pole_id = kInvalidObjectId;
  Transformd transform{};
  PlacementMode placement_mode = PlacementMode::kAuto;
  int candidate_index = -1;
  PoleContextInfo context{};
  PoleTypeId pole_type_id = kInvalidPoleTypeId;
  PoleKind pole_kind = PoleKind::kConcrete;
  double height_m = 10.0;
  std::string name{"PathPole"};
};

struct BackboneSupportChainPlan {
  std::uint64_t session_id = 0;
  std::vector<BackboneSupportResolution> resolutions{};
  std::vector<ObjectId> ordered_pole_ids{};
  std::vector<ObjectId> ordered_support_node_ids{};
  std::unordered_map<ObjectId, SupportNode> support_node_by_id{};
  std::vector<PlannedPoleCreation> pole_creations{};
  std::vector<ObjectId> generated_pole_ids{};
};

struct BackboneCommittedSupportChain {
  std::uint64_t session_id = 0;
  ChangeSet change_set{};
  std::vector<ObjectId> generated_pole_ids{};
  std::vector<ObjectId> ordered_support_node_ids{};
  std::unordered_map<ObjectId, SupportNode> support_node_by_id{};
  std::unordered_map<ObjectId, ObjectId> committed_node_id_by_planned_node_id{};
};

struct BackboneTopologyPlan {
  BackboneResult existing_network_backbone{};
  BackboneResult generation_backbone{};
  std::unordered_map<ObjectId, std::uint64_t> existing_prioritized_session_by_node{};
  std::unordered_map<ObjectId, std::unordered_map<ObjectId, std::uint64_t>> existing_incident_session_by_node{};
  std::unordered_map<ObjectId, std::vector<ObjectId>> route_neighbors_by_node{};
  std::unordered_map<ObjectId, Vec3d> existing_node_position_by_id{};
  BackboneDecisionPhaseOutput decision_phase{};
};

struct BackboneOrientationPlan {
  std::unordered_map<ObjectId, BackbonePlannedPoleOrientation> planned_pole_orientations{};
};

struct BackboneCommittedTopologyState {
  BackboneResult generation_backbone{};
  std::unordered_map<ObjectId, std::vector<ObjectId>> route_neighbors_by_node{};
  std::unordered_map<ObjectId, Vec3d> node_position_by_id{};
  std::unordered_map<ObjectId, std::uint64_t> existing_prioritized_session_by_node{};
  std::unordered_map<ObjectId, std::unordered_map<ObjectId, std::uint64_t>> existing_incident_session_by_node{};
};

struct BackboneCommittedGenerationPlan {
  std::uint64_t session_id = 0;
  std::vector<ObjectId> ordered_support_node_ids{};
  std::unordered_map<ObjectId, SupportNode> support_node_by_id{};
  std::unordered_map<ObjectId, ObjectId> committed_node_id_by_planned_node_id{};
  BackboneCommittedTopologyState topology_state{};
  BackboneDecisionPhaseOutput decision_phase{};
  std::unordered_map<ObjectId, BackbonePlannedPoleOrientation> planned_pole_orientations{};
};

struct BackboneGenerationPlan {
  BackboneGenerationRequestPlan request_plan{};
  BackboneSupportChainPlan support_chain_plan{};
  BackboneTopologyPlan topology_plan{};
  BackboneOrientationPlan orientation_plan{};
  ChangeSet change_set{};
  GenerateBundleFromPathResult result{};
};

[[nodiscard]] EditResult<BackboneGenerationRequestPlan> build_backbone_generation_request_plan(
    const CoreState& state, const BackboneSpec& spec);

} // namespace wire::core::generation::detail
