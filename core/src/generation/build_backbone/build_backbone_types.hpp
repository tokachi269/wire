#pragma once

#include "wire/core/core_state.hpp"
#include "../pole_facing/pole_facing_rules.hpp"

#include <functional>
#include <limits>
#include <string>
#include <unordered_map>

namespace wire::core::generation::detail {

struct SupportNodeCandidate {
  Vec3d world{};
  std::size_t segment_index = 0;
  int vertex_index = -1;
  double t = 0.0;
  PlacementMode mode = PlacementMode::kAuto;
  SupportKind support_kind = SupportKind::kPole;
  bool has_tangent_hint = false;
  Vec3d tangent_hint{};
};

using NodeSpecByIndex = std::unordered_map<std::size_t, BackboneInputSpec::NodeSpec>;
using NodeBundleModeByPoint = std::unordered_map<std::size_t, std::unordered_map<BundleKind, BundleNodeMode>>;

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

struct BackboneTopologyPlan {
  BackboneResult existing_network_backbone{};
  BackboneResult generation_backbone{};
  std::unordered_map<ObjectId, std::uint64_t> existing_prioritized_session_by_node{};
  std::unordered_map<ObjectId, std::unordered_map<ObjectId, std::uint64_t>> existing_incident_session_by_node{};
  std::unordered_map<ObjectId, std::vector<ObjectId>> route_neighbors_by_node{};
  std::unordered_map<ObjectId, Vec3d> existing_node_position_by_id{};
};

[[nodiscard]] EditResult<BackboneGenerationRequestPlan> build_backbone_generation_request_plan(
    const CoreState& state, const BackboneSpec& spec);

bool build_backbone_node_maps(const BackboneSpec& request, NodeSpecByIndex* out_node_spec_by_index,
                              NodeBundleModeByPoint* out_node_bundle_mode_by_point, std::string* error);
void build_backbone_guide_points(const BackboneSpec& request, std::vector<Vec3d>* out_guide_points,
                                 PathDirectionEvaluationDebug* out_direction_debug);
bool build_backbone_candidates(const BackboneSpec& request, const std::vector<Vec3d>& guide_points,
                               const NodeSpecByIndex& node_spec_by_index, std::vector<SupportNodeCandidate>* out_candidates,
                               std::string* error);

} // namespace wire::core::generation::detail
