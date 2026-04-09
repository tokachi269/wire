#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "wire/core/debug_types.hpp"
#include "wire/core/detail_curve.hpp"
#include "wire/core/endpoint_resolution.hpp"
#include "wire/core/entities.hpp"
#include "wire/core/id.hpp"
#include "wire/core/types.hpp"
#include "wire/core/workflow_types.hpp"

namespace wire::core {

enum class EntityKind : std::uint8_t {
  kUnknown = 0,
  kPole = 1,
  kJunction = 2,
  kBackboneEdge = 3,
  kSupportNode = 4,
  kSupportLayout = 5,
  kSpan = 6,
  kBundle = 7,
  kDetailCurve = 8,
  kAttachmentEndpoint = 9,
  kTemplate = 10,
  kOverride = 11,
};

enum class EntityRoleKind : std::uint8_t {
  kAuthoritative = 0,
  kOverride = 1,
  kDerived = 2,
  kDetailDerived = 3,
};

enum class PropertyAccessKind : std::uint8_t {
  kReadonly = 0,
  kEditable = 1,
  kOverrideable = 2,
  kHidden = 3,
};

enum class TemplateKind : std::uint8_t {
  kPoleType = 0,
  kCable = 1,
  kBundle = 2,
  kAttachment = 3,
};

enum class OverrideTopicKind : std::uint8_t {
  kPoleForward = 0,
  kPoleYaw = 1,
  kSupportStyle = 2,
  kBranchDownOffset = 3,
  kMirror = 4,
  kFlowClassification = 5,
  kAttachmentSocket = 6,
};

enum class DecisionTraceTopic : std::uint8_t {
  kPoleOrientation = 0,
  kFlowClassification = 1,
  kSupportLayoutSelection = 2,
  kTangentGeneration = 3,
  kContinuitySelection = 4,
  kContinuityDegrade = 5,
  kSagProfile = 6,
  kOverrideResolution = 7,
};

struct EntityRef {
  EntityKind kind = EntityKind::kUnknown;
  std::uint64_t stable_id = 0;

  [[nodiscard]] constexpr bool valid() const noexcept { return kind != EntityKind::kUnknown && stable_id != 0; }
};

struct VariationBreakdownView {
  std::uint64_t flow_key = 0;
  double world_bias = 0.0;
  double flow_bias = 0.0;
  double pole_delta = 0.0;
  double local_jitter = 0.0;
  double final_value = 0.0;
};

struct StyleInspectionView {
  bool has_context = false;
  StyleRouteKey route_key{};
  StyleObjectKey object_key{};
  ResolvedStyleContext resolved{};
};

struct EntityMeta {
  EntityRef ref{};
  std::string display_name{};
  EntityRoleKind role = EntityRoleKind::kDerived;
  bool editable = false;
  bool overrideable = false;
  std::string provenance{};
};

struct RelatedEntityLink {
  std::string label{};
  EntityRef ref{};
};

struct DecisionTraceEntry {
  DecisionTraceTopic topic = DecisionTraceTopic::kPoleOrientation;
  std::string rule{};
  std::string summary{};
};

struct PropertyView {
  std::string name{};
  std::string value{};
  PropertyAccessKind access = PropertyAccessKind::kReadonly;
};

struct OverrideEntryView {
  OverrideTopicKind topic = OverrideTopicKind::kPoleForward;
  std::string name{};
  std::string automatic_value{};
  std::string override_value{};
  std::string resolved_value{};
  bool active = false;
  PropertyAccessKind access = PropertyAccessKind::kOverrideable;
};

struct PoleInspectionView {
  EntityMeta meta{};
  ObjectId pole_id = kInvalidObjectId;
  Vec3d position{};
  double height_m = 0.0;
  Vec3d forward_dir{};
  bool has_forward = false;
  Vec3d tilt_deg{};
  std::optional<double> manual_yaw_override_deg{};
  std::optional<bool> flip_180_override{};
  bool has_final_yaw = false;
  double automatic_yaw_deg = 0.0;
  double final_yaw_deg = 0.0;
  bool placement_override = false;
  bool orientation_override = false;
  PoleForwardRule forward_rule = PoleForwardRule::kFallback;
  PoleSupportAxisRule support_axis_rule = PoleSupportAxisRule::kFallback;
  RowLayoutAxisMode row_layout_axis_mode = RowLayoutAxisMode::kPoleYaw;
  ConnectionCategory row_layout_axis_category = ConnectionCategory::kLowVoltage;
  ObjectId primary_neighbor_id = kInvalidObjectId;
  ObjectId secondary_neighbor_id = kInvalidObjectId;
  Vec3d support_axis_dir{};
  bool has_support_axis = false;
  double layout_yaw_deg = 0.0;
  bool has_layout_yaw = false;
  std::vector<RelatedEntityLink> links{};
};

struct SpanInspectionView {
  EntityMeta meta{};
  ObjectId span_id = kInvalidObjectId;
  ObjectId port_a_id = kInvalidObjectId;
  ObjectId port_b_id = kInvalidObjectId;
  EntityRef bundle_ref{};
  EntityRef start_pole_ref{};
  EntityRef end_pole_ref{};
  EntityRef support_layout_ref{};
  EntityRef detail_curve_ref{};
  BackboneFlowKind flow_kind = BackboneFlowKind::kMain;
  BackboneFlowDecisionRule flow_rule = BackboneFlowDecisionRule::kDefaultMain;
  OrderDecisionPolicyKind order_decision_policy = OrderDecisionPolicyKind::kFixedOrder;
  OrderDecisionChoiceKind order_decision_choice_a = OrderDecisionChoiceKind::kNormal;
  OrderDecisionChoiceKind order_decision_choice_b = OrderDecisionChoiceKind::kNormal;
  OrderDecisionChoiceReason order_decision_choice_reason_a = OrderDecisionChoiceReason::kFixedOrder;
  OrderDecisionChoiceReason order_decision_choice_reason_b = OrderDecisionChoiceReason::kFixedOrder;
  ContinuityCategoryClass continuity_class = ContinuityCategoryClass::kPointLike;
  bool default_lower_required = false;
  CableContinuityPolicyHint requested_continuity = CableContinuityPolicyHint::kAuto;
  DetailCurveContinuityMode adopted_continuity = DetailCurveContinuityMode::kG1;
  DetailCurveContinuityReason continuity_reason = DetailCurveContinuityReason::kAutoBalanced;
  bool degraded_to_g1 = false;
  double sag_amplitude_m = 0.0;
  double curve_length_m = 0.0;
  bool uses_branch_support = false;
  BackboneLoweringKind lowering_kind = BackboneLoweringKind::kNone;
  double branch_down_offset_m = 0.0;
  bool same_level_feasible = true;
  SameLevelFeasibilityReason same_level_reason = SameLevelFeasibilityReason::kNone;
  double projected_spacing_topview_m = -1.0;
  double required_clearance_m = 0.0;
  bool lowering_blocked_by_policy = false;
  bool unresolved_same_level_conflict = false;
  bool solver_used_same_level_constraint = false;
  bool used_special_case_ports = false;
  bool flipped_from_previous = false;
  double turn_angle_deg = 0.0;
  StyleInspectionView style{};
  std::vector<RelatedEntityLink> links{};
};

struct SupportLayoutEndpointView {
  ObjectId endpoint_node_id = kInvalidObjectId;
  ObjectId owner_pole_id = kInvalidObjectId;
  ObjectId port_id = kInvalidObjectId;
  bool authoritative_group_cache_present = false;
  ResolvedSupportAuthority support_authority{};
  EndpointAttachmentRequest attachment_request{};
  std::optional<int> resolved_socket_id{};
  BackboneFlowKind flow_kind = BackboneFlowKind::kMain;
  JunctionRelationKind relation_kind = JunctionRelationKind::kNone;
  ContinuityCategoryClass continuity_class = ContinuityCategoryClass::kPointLike;
  bool in_through_pair = false;
  bool lower_required = false;
  int support_group_id = -1;
  bool default_lower_required = false;
  OrderDecisionPolicyKind order_decision_policy = OrderDecisionPolicyKind::kFixedOrder;
  OrderDecisionChoiceKind order_decision_choice = OrderDecisionChoiceKind::kNormal;
  OrderDecisionChoiceReason order_decision_choice_reason = OrderDecisionChoiceReason::kFixedOrder;
  LateralSideChoiceKind chosen_side = LateralSideChoiceKind::kCenter;
  SlotSide side = SlotSide::kCenter;
  SideAssignmentRuleKind side_assignment_rule = SideAssignmentRuleKind::kPoleLocal;
  SupportOrientationRuleKind support_orientation_rule = SupportOrientationRuleKind::kRadial;
  SupportOrientationBasisKind support_orientation_basis = SupportOrientationBasisKind::kRadial;
  bool used_junction_pair_side_assignment = false;
  bool has_side_axis = false;
  Vec3d side_axis{};
  double chosen_side_sign = 0.0;
  bool has_signed_support_axis = false;
  Vec3d signed_support_axis{};
  int pair_height_rank = -1;
  std::string origin{};
  SupportLayoutEndpointSourceKind endpoint_source = SupportLayoutEndpointSourceKind::kFallback;
  std::string port_source{};
  std::string endpoint_mode{};
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
  bool same_level_feasible = true;
  SameLevelFeasibilityReason same_level_reason = SameLevelFeasibilityReason::kNone;
  double projected_spacing_topview_m = -1.0;
  double required_clearance_m = 0.0;
  bool lowering_blocked_by_policy = false;
  bool unresolved_same_level_conflict = false;
  bool solver_used_same_level_constraint = false;
  bool used_special_case_ports = false;
  VariationBreakdownView down_offset_variation{};
};

struct LoweredSupportGroupInspectionView {
  ObjectId owner_pole_id = kInvalidObjectId;
  ResolvedSupportAuthority support_authority{};
  JunctionRelationKind relation_kind = JunctionRelationKind::kNone;
  ContinuityCategoryClass continuity_class = ContinuityCategoryClass::kPointLike;
  bool in_through_pair = false;
  bool lower_required = false;
  ObjectId pair_peer_low = kInvalidObjectId;
  ObjectId pair_peer_high = kInvalidObjectId;
  SlotSide side = SlotSide::kCenter;
  std::string origin{};
  SupportGroupingRuleKind grouping_rule = SupportGroupingRuleKind::kDecisionGroup;
  int support_group_id = -1;
  int grouped_port_count = 1;
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
  bool has_signed_support_axis = false;
  Vec3d signed_support_axis{};
  int pair_height_rank = -1;
  double down_offset_m = 0.0;
  Vec3d mount_world{};
  Vec3d tip_world{};
  std::vector<Vec3d> attachment_worlds{};
  VariationBreakdownView down_offset_variation{};
};

struct SupportLayoutInspectionView {
  EntityMeta meta{};
  EntityRef source_span{};
  bool has_decision_seed = false;
  bool requires_decision_seed = false;
  bool grouped_authority_cache_complete = true;
  BackboneFlowKind flow_kind = BackboneFlowKind::kMain;
  CurvePassMode pass_mode = CurvePassMode::kPassThrough;
  std::uint64_t variation_flow_key = 0;
  OrderDecisionPolicyKind order_decision_policy = OrderDecisionPolicyKind::kFixedOrder;
  JunctionRelationKind relation_a = JunctionRelationKind::kNone;
  JunctionRelationKind relation_b = JunctionRelationKind::kNone;
  ContinuityCategoryClass continuity_class = ContinuityCategoryClass::kPointLike;
  bool default_lower_required = false;
  bool same_level_feasible = true;
  SameLevelFeasibilityReason same_level_reason = SameLevelFeasibilityReason::kNone;
  double projected_spacing_topview_m = -1.0;
  double required_clearance_m = 0.0;
  bool lowering_blocked_by_policy = false;
  bool unresolved_same_level_conflict = false;
  bool solver_used_same_level_constraint = false;
  bool used_special_case_ports = false;
  BackboneLoweringKind lowering_kind = BackboneLoweringKind::kNone;
  SupportLayoutEndpointView start_endpoint{};
  SupportLayoutEndpointView end_endpoint{};
  std::vector<LoweredSupportGroupInspectionView> lowered_support_groups{};
  std::vector<RelatedEntityLink> links{};
};

struct DetailCurveInspectionView {
  EntityMeta meta{};
  EntityRef source_span{};
  CableContinuityPolicyHint requested_continuity = CableContinuityPolicyHint::kAuto;
  CurveShapePolicyKind shape_policy = CurveShapePolicyKind::kNearStraight;
  DetailCurveContinuityMode adopted_continuity = DetailCurveContinuityMode::kG1;
  DetailCurveContinuityReason continuity_reason = DetailCurveContinuityReason::kAutoBalanced;
  bool attempted_g2 = false;
  bool degraded_to_g1 = false;
  double sag_amplitude_m = 0.0;
  double curve_length_m = 0.0;
  double tangent_scale = 1.0;
  double base_handle_scale = 1.0;
  double policy_handle_scale = 1.0;
  double start_angle_scale = 1.0;
  double end_angle_scale = 1.0;
  double handle_length_start_m = 0.0;
  double handle_length_end_m = 0.0;
  std::array<Vec3d, 4> control_points{};
  std::size_t segment_count = 0;
  std::size_t arc_length_sample_count = 0;
  std::size_t visible_interval_count = 0;
  std::size_t hidden_interval_count = 0;
  std::size_t replacement_path_count = 0;
  std::size_t supplemental_path_count = 0;
  DetailCurveEndpointTangentRule start_tangent_rule = DetailCurveEndpointTangentRule::kFallbackChord;
  DetailCurveEndpointTangentRule end_tangent_rule = DetailCurveEndpointTangentRule::kFallbackChord;
  double start_support_weight = 0.0;
  double end_support_weight = 0.0;
  double start_chord_weight = 1.0;
  double end_chord_weight = 1.0;
  double start_departure_length_m = 0.0;
  double end_departure_length_m = 0.0;
  double start_lateral_ratio_limit = 0.0;
  double end_lateral_ratio_limit = 0.0;
  double lateral_suppression = 0.0;
  SupportLayoutEndpointSourceKind start_endpoint_source = SupportLayoutEndpointSourceKind::kFallback;
  SupportLayoutEndpointSourceKind end_endpoint_source = SupportLayoutEndpointSourceKind::kFallback;
  EndpointAttachmentRequest start_attachment_request{};
  EndpointAttachmentRequest end_attachment_request{};
  std::optional<int> start_resolved_socket_id{};
  std::optional<int> end_resolved_socket_id{};
  double sag_base_ratio = 0.0;
  double sag_length_scale = 1.0;
  double sag_pass_scale = 1.0;
  double sag_rigidity_scale = 1.0;
  VariationBreakdownView sag_variation{};
  StyleInspectionView style{};
  std::vector<RelatedEntityLink> links{};
};

struct JunctionIncidentRelationView {
  ObjectId neighbor_node_id = kInvalidObjectId;
  JunctionRelationKind kind = JunctionRelationKind::kNone;
  double straightness_score = -1.0;
  bool in_route = false;
  bool in_through_pair = false;
  ContinuityCategoryClass continuity_class = ContinuityCategoryClass::kPointLike;
  bool default_lower_required = false;
  bool same_level_feasible = true;
  SameLevelFeasibilityReason infeasible_reason = SameLevelFeasibilityReason::kNone;
  double projected_spacing_topview_m = -1.0;
  double required_clearance_m = 0.0;
};

struct JunctionInspectionView {
  EntityMeta meta{};
  ObjectId node_id = kInvalidObjectId;
  bool has_primary = false;
  bool has_local_relation = false;
  bool through_pair_accepted = false;
  bool is_cross_like = false;
  int route_incident_count = 0;
  ObjectId through_pair_neighbor_a_id = kInvalidObjectId;
  ObjectId through_pair_neighbor_b_id = kInvalidObjectId;
  double through_pair_straightness_score = -1.0;
  std::vector<JunctionIncident> incidents{};
  std::vector<JunctionIncidentRelationView> local_relations{};
  std::vector<RelatedEntityLink> links{};
};

struct TemplateInspectionView {
  EntityMeta meta{};
  TemplateKind template_kind = TemplateKind::kCable;
  std::vector<PropertyView> properties{};
  std::vector<RelatedEntityLink> links{};
};

struct OverrideInspectionView {
  EntityMeta meta{};
  EntityRef target{};
  std::vector<OverrideEntryView> entries{};
  std::vector<RelatedEntityLink> links{};
};

} // namespace wire::core
