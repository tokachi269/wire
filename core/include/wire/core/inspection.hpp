#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "wire/core/detail_curve.hpp"
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
  bool manual_yaw_override = false;
  double manual_yaw_deg = 0.0;
  bool flip_180_override = false;
  bool has_final_yaw = false;
  double automatic_yaw_deg = 0.0;
  double final_yaw_deg = 0.0;
  bool placement_override = false;
  bool orientation_override = false;
  PoleForwardRule forward_rule = PoleForwardRule::kFallback;
  PoleSupportAxisRule support_axis_rule = PoleSupportAxisRule::kFallback;
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
  CableContinuityPolicyHint requested_continuity = CableContinuityPolicyHint::kAuto;
  DetailCurveContinuityMode adopted_continuity = DetailCurveContinuityMode::kG1;
  DetailCurveContinuityReason continuity_reason = DetailCurveContinuityReason::kAutoBalanced;
  bool degraded_to_g1 = false;
  double sag_amplitude_m = 0.0;
  double curve_length_m = 0.0;
  bool uses_branch_support = false;
  double branch_down_offset_m = 0.0;
  bool mirrored = false;
  bool flipped_from_previous = false;
  double turn_angle_deg = 0.0;
  std::vector<RelatedEntityLink> links{};
};

struct SupportLayoutEndpointView {
  ObjectId endpoint_node_id = kInvalidObjectId;
  ObjectId owner_pole_id = kInvalidObjectId;
  ObjectId port_id = kInvalidObjectId;
  ObjectId attachment_id = kInvalidObjectId;
  int socket_id = -1;
  BackboneFlowKind flow_kind = BackboneFlowKind::kMain;
  SlotSide side = SlotSide::kCenter;
  std::string origin{};
  std::string endpoint_source{};
  bool attachment_input_present = false;
  bool socket_override_active = false;
  std::string port_source{};
  std::string endpoint_mode{};
  Vec3d support_world{};
  Vec3d endpoint_world{};
  Vec3d departure_dir{};
  Vec3d endpoint_offset{};
  double local_departure_length_m = 0.0;
  double automatic_branch_down_offset_m = 0.0;
  double branch_down_offset_m = 0.0;
  VariationBreakdownView down_offset_variation{};
};

struct SupportLayoutInspectionView {
  EntityMeta meta{};
  EntityRef source_span{};
  BackboneFlowKind flow_kind = BackboneFlowKind::kMain;
  CurvePassMode pass_mode = CurvePassMode::kPassThrough;
  std::uint64_t variation_flow_key = 0;
  SupportLayoutEndpointView start_endpoint{};
  SupportLayoutEndpointView end_endpoint{};
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
  std::size_t arc_length_sample_count = 0;
  std::size_t visible_interval_count = 0;
  std::size_t hidden_interval_count = 0;
  std::size_t replacement_path_count = 0;
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
  std::string start_endpoint_source{};
  std::string end_endpoint_source{};
  bool start_attachment_input_present = false;
  bool end_attachment_input_present = false;
  int start_socket_id = -1;
  int end_socket_id = -1;
  double sag_base_ratio = 0.0;
  double sag_length_scale = 1.0;
  double sag_pass_scale = 1.0;
  double sag_rigidity_scale = 1.0;
  VariationBreakdownView sag_variation{};
  std::vector<RelatedEntityLink> links{};
};

struct JunctionInspectionView {
  EntityMeta meta{};
  ObjectId node_id = kInvalidObjectId;
  bool has_primary = false;
  std::vector<JunctionIncident> incidents{};
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
