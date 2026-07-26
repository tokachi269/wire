#include "city/wire/support/numeric_tolerances.hpp"
#pragma once

#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "city/wire/entities.hpp"

namespace city::wire {

// Workflow and template input types used by generation and editing commands.
// Workflow input path spec.
struct BackboneInputSpec {
  std::vector<Vec3d> polyline{};
  // Optional per-path-point support-node metadata. Missing indices are treated as Pole.
  struct NodeSpec {
    std::size_t point_index = std::numeric_limits<std::size_t>::max();
    SupportKind support_kind = SupportKind::kPole;
    ObjectId node_id = kInvalidObjectId;
    bool has_tangent_hint = false;
    Vec3d tangent_hint{};
  };
  std::vector<NodeSpec> node_specs{};
};

struct BackboneGenerationConstraints {
  std::vector<Vec3d> avoid_points{};
  double avoid_radius_m = 0.0;
  double lateral_offset_m = 0.0;
};

struct BackbonePolePlacementOptions {
  // Do not force manual poles from guide by default; users pin explicitly.
  bool pin_endpoints = false;
  bool pin_vertices = false;
  // Applies instance tilt before generated ports/bands are materialized.
  bool enable_tilt = false;
  double max_tilt_deg = 0.0;
  // Session-scoped regeneration can restrict pole reuse to the target session.
  bool restrict_reuse_to_session = false;
  std::uint64_t reuse_session_id = 0;
};

struct AttachmentSocketTemplate {
  int id = -1;
  Vec3d local_position{};
  Vec3d tangent_dir{};
  bool has_normal = false;
  Vec3d normal_dir{};
  bool has_binormal = false;
  Vec3d binormal_dir{};
  AttachmentSocketKind kind = AttachmentSocketKind::kGeneric;
};

struct AttachmentInternalPathTemplate {
  enum class ProfileKind : std::uint8_t {
    kExplicitPolyline = 0,
    kStraightCable = 1,
    kCoiledCable = 2,
  };

  int start_socket_id = -1;
  int end_socket_id = -1;
  ProfileKind profile_kind = ProfileKind::kExplicitPolyline;
  std::vector<Vec3d> local_points{};
  double coil_radius_m = 0.0;
  int coil_turn_count = 0;
  int coil_samples_per_turn = 12;
};

struct AttachmentTemplate {
  AttachmentTemplateId id = kInvalidAttachmentTemplateId;
  std::string name{};
  AttachmentKind kind = AttachmentKind::kGeneric;
  AttachmentLineInteractionMode line_interaction_mode = AttachmentLineInteractionMode::kPassThrough;
  std::vector<AttachmentSocketTemplate> sockets{};
  std::vector<AttachmentInternalPathTemplate> internal_paths{};
  std::uint64_t version = 1;
};

enum class ModelFitMode : std::uint8_t {
  kRigid = 0,
  kPoleHeight = 1,
  // Circular-pole radial approximation at the assembly placement height.
  kPoleRadial = 2,
  // Place the part root on the pole surface toward the endpoint fixture root.
  kPoleSurface = 3,
};

struct ModelAssemblySocket {
  std::string name{};
  Vec3d local_position{};
  Vec3d local_direction{1.0, 0.0, 0.0};

  bool operator==(const ModelAssemblySocket& other) const {
    return name == other.name && local_position.x == other.local_position.x &&
           local_position.y == other.local_position.y && local_position.z == other.local_position.z &&
           local_direction.x == other.local_direction.x && local_direction.y == other.local_direction.y &&
           local_direction.z == other.local_direction.z;
  }
};

struct ModelAssemblyPart {
  std::uint32_t part_id = 0;
  std::string model_key{};
  std::uint64_t descriptor_version = 1;
  Transformd local_transform{};
  ModelFitMode fit_mode = ModelFitMode::kRigid;
  std::vector<ModelAssemblySocket> sockets{};

  bool operator==(const ModelAssemblyPart& other) const {
    return part_id == other.part_id && model_key == other.model_key &&
           descriptor_version == other.descriptor_version &&
           local_transform.position.x == other.local_transform.position.x &&
           local_transform.position.y == other.local_transform.position.y &&
           local_transform.position.z == other.local_transform.position.z &&
           local_transform.rotation_euler_deg.x == other.local_transform.rotation_euler_deg.x &&
           local_transform.rotation_euler_deg.y == other.local_transform.rotation_euler_deg.y &&
           local_transform.rotation_euler_deg.z == other.local_transform.rotation_euler_deg.z &&
           local_transform.scale.x == other.local_transform.scale.x &&
           local_transform.scale.y == other.local_transform.scale.y &&
           local_transform.scale.z == other.local_transform.scale.z && fit_mode == other.fit_mode &&
           sockets == other.sockets;
  }
};

struct AssemblySocketRef {
  std::uint32_t part_id = 0;
  std::string socket_name{};

  bool operator==(const AssemblySocketRef&) const = default;
};

struct ModelAssemblyTemplate {
  ModelAssemblyTemplateId id = kInvalidModelAssemblyTemplateId;
  std::vector<ModelAssemblyPart> parts{};
  std::optional<AssemblySocketRef> wire_socket{};
  // Optional row-fixture point from which endpoint fixtures inherit position.
  // Rotation remains owned by the Port PoleFrame/layout yaw.
  std::optional<AssemblySocketRef> endpoint_mount_socket{};
  std::uint64_t version = 1;

  bool operator==(const ModelAssemblyTemplate&) const = default;
};

struct CableSupplementalPathTemplate {
  enum class AnchorMode : std::uint8_t {
    kCurveOffset = 0,
    kPoleBandChord = 1,
  };

  enum class ProfileKind : std::uint8_t {
    kNone = 0,
    kStraightCable = 1,
  };

  AnchorMode anchor_mode = AnchorMode::kCurveOffset;
  ProfileKind profile_kind = ProfileKind::kNone;
  AttachmentLineInteractionMode interaction_mode = AttachmentLineInteractionMode::kAddInternalPath;
  int pole_band_id = 0;
  double endpoint_trim_m = 0.0;
  double lateral_offset_m = 0.0;
  double vertical_offset_m = 0.0;
  double wobble_amplitude_m = 0.0;
  double wobble_wavelength_m = 0.0;
  double wobble_phase_bias = 0.0;
  double endpoint_envelope_ratio = 0.0;
};

enum class BundleCountRuleKind : std::uint8_t {
  kFixed = 0,
  kRange = 1,
};

struct CableTemplate {
  // Single-cable appearance / physical hints. Topology policy is intentionally excluded.
  CableTemplateId id = kInvalidCableTemplateId;
  std::string name{};
  double outer_diameter_m = 0.03;
  double default_grouped_support_fanout_spacing_m = 0.2;
  double bend_stiffness = 1.0;
  double min_bend_radius_m = 0.2;
  CableMaterialStyleKind material_style = CableMaterialStyleKind::kGeneric;
  std::uint32_t color_rgba = 0xFFFFFFFFu;
  double sag_factor = 0.03;
  double slack_factor = 0.0;
  CableContinuityPolicyHint continuity_policy = CableContinuityPolicyHint::kAuto;
  CableAttachmentStyleHint attachment_style = CableAttachmentStyleHint::kAuto;
  AttachmentTemplateId default_endpoint_attachment_template_id = kInvalidAttachmentTemplateId;
  std::vector<CableSupplementalPathTemplate> supplemental_paths{};
  std::uint64_t version = 1;
};

enum class OrderDecisionPolicyKind : std::uint8_t;
enum class RowLayoutAxisMode : std::uint8_t {
  kPoleYaw = 0,
  kSupportAxis = 1,
};

constexpr BundleTemplateId kDefaultHighVoltageBundleTemplateId = 101;
constexpr BundleTemplateId kDefaultLowVoltageBundleTemplateId = 102;
constexpr BundleTemplateId kDefaultDropBundleTemplateId = 103;
constexpr BundleTemplateId kDefaultCommunicationBundleTemplateId = 104;
constexpr BundleTemplateId kDefaultOpticalBundleTemplateId = 105;
constexpr CableTemplateId kDefaultSupportWireCableTemplateId = 6;

[[nodiscard]] constexpr BundleTemplateId DefaultBundleTemplateId(BundleKind kind) noexcept {
  switch (kind) {
  case BundleKind::kHighVoltage:
    return kDefaultHighVoltageBundleTemplateId;
  case BundleKind::kLowVoltage:
    return kDefaultLowVoltageBundleTemplateId;
  case BundleKind::kDrop:
    return kDefaultDropBundleTemplateId;
  case BundleKind::kCommunication:
    return kDefaultCommunicationBundleTemplateId;
  case BundleKind::kOptical:
    return kDefaultOpticalBundleTemplateId;
  case BundleKind::kOpticalWithSupport:
    return kInvalidBundleTemplateId;
  }
  return kInvalidBundleTemplateId;
}

struct SpanVisualAssemblyTemplate {
  bool support_path_enabled = false;
  bool helix_enabled = false;
  double helix_radius_m = 0.0;
  double helix_clearance_m = 0.0;
  double helix_turns_per_meter = 0.0;
  int helix_samples_per_turn = 16;
  double endpoint_trim_m = 0.0;
  double member_wander_ratio = 0.0;
  double member_wander_wavelength_m = 0.0;
  double member_wander_phase_bias = 0.0;
  double member_twist_turns_per_meter = 0.0;
  double member_twist_phase = 0.0;
};

struct BundleTemplate {
  // Bundle/system rule set. Topology policy lives here, not on CableTemplate.
  BundleTemplateId id = kInvalidBundleTemplateId;
  BundleKind kind = BundleKind::kLowVoltage;
  std::string name{};
  ConnectionCategory category = ConnectionCategory::kLowVoltage;
  CableTemplateId cable_template_id = kInvalidCableTemplateId;
  SpanLayer default_layer = SpanLayer::kLowVoltage;
  PoleTypeId related_pole_type_id = kInvalidPoleTypeId;
  // If true, conductor identity/order is treated as strict by higher-level workflow policy.
  bool preserve_conductor_identity = false;
  BundleCountRuleKind count_rule = BundleCountRuleKind::kFixed;
  int fixed_count = 1;
  int min_count = 1;
  int max_count = 1;
  int default_count = 1;
  double default_spacing_m = 0.2;
  double grouped_support_fanout_spacing_m = 0.2;
  bool allow_mirror = true;
  bool allow_midair_node = true;
  bool allow_midair_branch = true;
  bool enable_branch_down_offset = false;
  double branch_endpoint_offset_m = 0.0;
  OrderDecisionPolicyKind order_decision_policy{};
  RowLayoutAxisMode row_layout_axis_mode = RowLayoutAxisMode::kPoleYaw;
  BundleSupportStyleHint support_style = BundleSupportStyleHint::kAuto;
  BundleBranchPolicyHint branch_policy = BundleBranchPolicyHint::kAuto;
  CableContinuityPolicyHint continuity_policy = CableContinuityPolicyHint::kAuto;
  int support_wire_pole_band_id = 0;
  ModelAssemblyTemplateId row_fixture_assembly_id = kInvalidModelAssemblyTemplateId;
  ModelAssemblyTemplateId endpoint_fixture_assembly_id = kInvalidModelAssemblyTemplateId;
  SpanVisualAssemblyTemplate span_visual_assembly{};
  std::vector<CablePopulationRule> population_rules{};
  std::uint64_t version = 1;
};

enum class BundleNodeMode : std::uint8_t {
  kNotPresent = 0,
  kPassThrough = 1,
  // Branch/Terminate are intentionally deferred.
};

struct SupportNodeBundleMode {
  BundleTemplateId bundle_template_id = kInvalidBundleTemplateId;
  BundleNodeMode mode = BundleNodeMode::kNotPresent;
};

struct SupportNode {
  ObjectId node_id = kInvalidObjectId;
  SupportKind support_kind = SupportKind::kPole;
  Vec3d position{};
  ObjectId pole_id = kInvalidObjectId;
  ObjectId saved_backbone_node_id = kInvalidObjectId;
  bool has_source_edge = false;
  ObjectId source_edge_node_a_id = kInvalidObjectId;
  ObjectId source_edge_node_b_id = kInvalidObjectId;
  double source_edge_t = 0.0;
  int path_point_index = -1;
  bool has_tangent_hint = false;
  Vec3d tangent_hint{};
  std::vector<SupportNodeBundleMode> bundle_modes{};
};

enum class PickHitKind : std::uint8_t {
  kEmpty = 0,
  kNode = 1,
  kSegment = 2,
  kGround = 3,
  kExternal = 4,
};

// Viewer-side raycast output passed into core commands.
// core interprets this payload but does not run scene queries.
struct PickResult {
  PickHitKind hit_kind = PickHitKind::kEmpty;
  Vec3d hit_pos_world{};
  ObjectId hit_id = kInvalidObjectId;
  bool has_segment_endpoints = false;
  ObjectId segment_node_a_id = kInvalidObjectId;
  ObjectId segment_node_b_id = kInvalidObjectId;
  Vec3d segment_endpoint_a_world{};
  Vec3d segment_endpoint_b_world{};
};

struct BackboneBundleSpec {
  BundleTemplateId bundle_template_id = kInvalidBundleTemplateId;
  // Stable identity of the request placement entry. Zero means legacy/unspecified.
  std::uint64_t placement_key = 0;
  SpanLayer layer = SpanLayer::kUnknown;
  // Used only for variable-count templates. Fixed templates reject count input.
  int count = 0;
  // Explicit pole-local Bundle placement. Band selection still owns fixture semantics.
  bool placement_explicit = false;
  double height_m = 0.0;
  double lateral_m = 0.0;
  double spacing_m = 0.0;
  // Source Bundle identity for source-edge attachment projection.
  ObjectId source_bundle_id = kInvalidObjectId;
  // Existing Bundle identity used only by saved-scope regeneration.
  ObjectId existing_bundle_id = kInvalidObjectId;
};

struct BackboneSpec {
  BackboneInputSpec path{};
  double interval_m = 0.0;
  PoleTypeId pole_type_id = kInvalidPoleTypeId;
  std::vector<BackboneBundleSpec> bundles{};
  // Optional per-node per-bundle connection mode hints.
  struct NodeBundleModeSpec {
    std::size_t point_index = std::numeric_limits<std::size_t>::max();
    BundleTemplateId bundle_template_id = kInvalidBundleTemplateId;
    BundleNodeMode mode = BundleNodeMode::kNotPresent;
  };
  std::vector<NodeBundleModeSpec> node_bundle_modes{};
  BackboneGenerationConstraints constraints{};
  BackbonePolePlacementOptions pole_placement{};
  PathDirectionMode direction_mode = PathDirectionMode::kAuto;
};

struct JunctionIncident {
  ObjectId neighbor_node_id = kInvalidObjectId;
  int order = -1;
  bool primary = false;
  std::uint64_t source_session_id = 0;
};

struct JunctionInfo {
  ObjectId node_id = kInvalidObjectId;
  std::uint64_t prioritized_session_id = 0;
  std::vector<JunctionIncident> incidents{};
};

enum class BackboneFlowKind : std::uint8_t {
  kMain = 0,
  kBranch = 1,
};

enum class BackboneFlowDecisionRule : std::uint8_t {
  kDefaultMain = 0,
  kJunctionOrderMain = 1,
  kJunctionOrderBranch = 2,
};

// High-level realism input. This stays abstract; core does not own raw domains/road/district datasets.
struct ContextProfile {
  double age = 0.5;
  double clutter = 0.5;
  double regularity = 0.5;
  double service_mix = 0.5;
  std::uint64_t style_seed = 1;
};

// Stable style keys should be derived from route/segment/lane semantics, not transient object ids.
struct StyleRouteKey {
  std::uint64_t family_id = 0;
  BundleTemplateId bundle_template_id = kInvalidBundleTemplateId;
  ConnectionCategory category = ConnectionCategory::kLowVoltage;
  BackboneFlowKind flow_kind = BackboneFlowKind::kMain;
};

struct StyleClusterKey {
  StyleRouteKey route{};
  std::uint32_t cluster_index = 0;
};

enum class StyleObjectKind : std::uint8_t {
  kSpan = 0,
  kEndpoint = 1,
  kAttachment = 2,
  kSupport = 3,
  kPoleAccessory = 4,
};

struct StyleObjectKey {
  StyleRouteKey route{};
  std::uint32_t segment_index = 0;
  std::uint32_t lane_index = 0;
  StyleObjectKind kind = StyleObjectKind::kSpan;
  std::uint32_t ordinal = 0;
  bool is_start_endpoint = false;
};

struct VariationScope {
  std::uint64_t district_seed = 0;
  std::uint64_t route_seed = 0;
  std::uint64_t cluster_seed = 0;
  std::uint64_t object_seed = 0;
};

struct DistrictStyle {
  double age = 0.5;
  double clutter = 0.5;
  double regularity = 0.5;
  double service_mix = 0.5;
  CableMaterialStyleKind cable_family = CableMaterialStyleKind::kGeneric;
  CableAttachmentStyleHint attachment_family = CableAttachmentStyleHint::kAuto;
};

struct RouteStyle {
  StyleRouteKey key{};
  double age_bias = 0.0;
  double clutter_bias = 0.0;
  double regularity_bias = 0.0;
  double service_mix_bias = 0.0;
  double sag_bias = 0.0;
  CableMaterialStyleKind cable_family = CableMaterialStyleKind::kGeneric;
  CableAttachmentStyleHint attachment_family = CableAttachmentStyleHint::kAuto;
};

struct ClusterStyle {
  StyleClusterKey key{};
  double clutter_bias = 0.0;
  double service_mix_bias = 0.0;
  double family_mix = 0.0;
};

struct ObjectVariation {
  StyleObjectKey key{};
  Vec3d local_offset_m{};
  double sag_delta_m = 0.0;
  double attachment_offset_m = 0.0;
  double choice_bias = 0.0;
};

// Downstream-only style surface. This must not reinterpret topology or ownership authority.
struct ResolvedStyleContext {
  ContextProfile profile{};
  VariationScope scope{};
  DistrictStyle district{};
  RouteStyle route{};
  ClusterStyle cluster{};
  ObjectVariation object{};
};

enum class JunctionRelationKind : std::uint8_t {
  kNone = 0,
  kThroughMain = 1,
  kSideBranch = 2,
  kCornerContinuation = 3,
  kCrossUnderpass = 4,
};

enum class ContinuityCategoryClass : std::uint8_t {
  kPointLike = 0,
  kBundleLike = 1,
};

enum class SameLevelFeasibilityReason : std::uint8_t {
  kNone = 0,
  kBundleRule = 1,
  kEnvelopeOverlap = 2,
  kNearNodeClearance = 3,
  kCategoryPolicyDisabled = 4,
};

enum class SideAssignmentRuleKind : std::uint8_t {
  kPoleLocal = 0,
  kChord = 1,
  kThroughPairNormal = 2,
  kBisector = 3,
};

enum class SupportOrientationRuleKind : std::uint8_t {
  kRadial = 0,
  kChord = 1,
  kThroughPairNormal = 2,
  kBisector = 3,
};

enum class OrderDecisionPolicyKind : std::uint8_t {
  kFixedOrder = 0,
  kPermutableHomogeneous = 1,
};

enum class OrderDecisionChoiceKind : std::uint8_t {
  kNormal = 0,
  kReversed = 1,
};

enum class OrderDecisionChoiceReason : std::uint8_t {
  kFixedOrder = 0,
  kCrossingFewer = 1,
  kSpacingBetter = 2,
  kTwistSmaller = 3,
  kKeptDefault = 4,
};

enum class LateralSideChoiceKind : std::uint8_t {
  kCenter = 0,
  kLeft = 1,
  kRight = 2,
};

enum class SupportOrientationBasisKind : std::uint8_t {
  kRadial = 0,
  kChordForward = 1,
  kChordReverse = 2,
  kBisectorForward = 3,
  kBisectorReverse = 4,
  kPairNormalPositive = 5,
  kPairNormalNegative = 6,
};

enum class SupportGroupingRuleKind : std::uint8_t {
  kPerPort = 0,
  kDecisionGroup = 1,
};

struct EndpointContinuityDecision {
  ObjectId owner_pole_id = kInvalidObjectId;
  JunctionRelationKind relation_kind = JunctionRelationKind::kNone;
  ContinuityCategoryClass continuity_class = ContinuityCategoryClass::kPointLike;
  bool in_through_pair = false;
  ObjectId support_pair_peer_low = kInvalidObjectId;
  ObjectId support_pair_peer_high = kInvalidObjectId;
  int support_group_id = -1;
  bool lower_required = false;
  bool default_lower_required = false;
  bool same_level_feasible = true;
  SameLevelFeasibilityReason same_level_reason = SameLevelFeasibilityReason::kNone;
  double projected_spacing_topview_m = -1.0;
  double required_clearance_m = 0.0;
  bool lowering_blocked_by_policy = false;
  bool unresolved_same_level_conflict = false;
  bool solver_used_same_level_constraint = false;
  bool used_special_case_ports = false;
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
};

[[nodiscard]] inline bool HasAuthoritativeSupportPair(const EndpointContinuityDecision& decision) {
  return decision.support_pair_peer_low != kInvalidObjectId && decision.support_pair_peer_high != kInvalidObjectId &&
         decision.support_pair_peer_low != decision.support_pair_peer_high;
}

[[nodiscard]] inline bool HasAuthoritativeSupportPair(ObjectId pair_peer_low, ObjectId pair_peer_high) {
  return pair_peer_low != kInvalidObjectId && pair_peer_high != kInvalidObjectId && pair_peer_low != pair_peer_high;
}

[[nodiscard]] inline bool UsesAuthoritativeGroupedLoweredSupport(const EndpointContinuityDecision& decision) {
  return decision.owner_pole_id != kInvalidObjectId && decision.lower_required && !decision.lowering_blocked_by_policy &&
         decision.support_group_id >= 0;
}

[[nodiscard]] inline LateralSideChoiceKind LateralSideChoiceFromSign(double sign) {
  if (sign > kLengthToleranceM) {
    return LateralSideChoiceKind::kRight;
  }
  if (sign < -kLengthToleranceM) {
    return LateralSideChoiceKind::kLeft;
  }
  return LateralSideChoiceKind::kCenter;
}

[[nodiscard]] inline SupportOrientationBasisKind SupportOrientationBasisFromDecision(
    SupportOrientationRuleKind rule, double chosen_side_sign) {
  if (rule == SupportOrientationRuleKind::kRadial) {
    return SupportOrientationBasisKind::kRadial;
  }
  if (rule == SupportOrientationRuleKind::kThroughPairNormal) {
    return (chosen_side_sign < 0.0) ? SupportOrientationBasisKind::kPairNormalNegative
                                    : SupportOrientationBasisKind::kPairNormalPositive;
  }
  if (rule == SupportOrientationRuleKind::kBisector) {
    return (chosen_side_sign < 0.0) ? SupportOrientationBasisKind::kBisectorReverse
                                    : SupportOrientationBasisKind::kBisectorForward;
  }
  return (chosen_side_sign < 0.0) ? SupportOrientationBasisKind::kChordReverse
                                  : SupportOrientationBasisKind::kChordForward;
}

[[nodiscard]] inline SupportOrientationBasisKind CanonicalSupportOrientationBasis(
    SupportOrientationRuleKind rule) {
  if (rule == SupportOrientationRuleKind::kRadial) {
    return SupportOrientationBasisKind::kRadial;
  }
  if (rule == SupportOrientationRuleKind::kThroughPairNormal) {
    return SupportOrientationBasisKind::kPairNormalPositive;
  }
  if (rule == SupportOrientationRuleKind::kBisector) {
    return SupportOrientationBasisKind::kBisectorForward;
  }
  return SupportOrientationBasisKind::kChordForward;
}

struct ThroughPair {
  ObjectId neighbor_a_id = kInvalidObjectId;
  ObjectId neighbor_b_id = kInvalidObjectId;
  double straightness_score = -1.0;
  bool accepted = false;
};

struct JunctionIncidentRelation {
  ObjectId neighbor_node_id = kInvalidObjectId;
  JunctionRelationKind kind = JunctionRelationKind::kNone;
  double straightness_score = -1.0;
  bool in_route = false;
  bool in_through_pair = false;
  ContinuityCategoryClass continuity_class = ContinuityCategoryClass::kPointLike;
  bool default_lower_required = false;
  bool same_level_feasible = true;
  double projected_spacing_topview_m = -1.0;
  double required_clearance_m = 0.0;
  SameLevelFeasibilityReason infeasible_reason = SameLevelFeasibilityReason::kNone;
};

struct JunctionRelation {
  ObjectId node_id = kInvalidObjectId;
  ObjectId primary_neighbor_id = kInvalidObjectId;
  int route_incident_count = 0;
  bool is_cross_like = false;
  ThroughPair through_pair{};
  std::vector<JunctionIncidentRelation> incidents{};
};

enum class LaneOrientation : std::uint8_t {
  kNormal = 0,
  kReversed = 1,
};

enum class LaneFlipReason : std::uint8_t {
  kNone = 0,
  kAcuteTurn = 1,
};

enum class BackboneLoweringKind : std::uint8_t {
  kNone = 0,
  kBranchSupport = 1,
  kCrossUnderpass = 2,
  kAcuteCorner = 3,
};

struct BackboneLoweringPolicy {
  double offset_m = 0.0;
  bool enable_branch_support = false;
  bool enable_cross_underpass = false;
  bool enable_acute_corner = false;
};

struct BackboneEdgeOrientation {
  ObjectId node_a_id = kInvalidObjectId;
  ObjectId node_b_id = kInvalidObjectId;
  BundleTemplateId bundle_template_id = kInvalidBundleTemplateId;
  std::uint64_t variation_flow_key = 0;
  BackboneFlowKind flow_kind = BackboneFlowKind::kMain;
  BackboneFlowDecisionRule flow_decision_rule = BackboneFlowDecisionRule::kDefaultMain;
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
  OrderDecisionPolicyKind order_decision_policy = OrderDecisionPolicyKind::kFixedOrder;
  OrderDecisionChoiceKind order_decision_choice_a = OrderDecisionChoiceKind::kNormal;
  OrderDecisionChoiceKind order_decision_choice_b = OrderDecisionChoiceKind::kNormal;
  OrderDecisionChoiceReason order_decision_choice_reason_a = OrderDecisionChoiceReason::kFixedOrder;
  OrderDecisionChoiceReason order_decision_choice_reason_b = OrderDecisionChoiceReason::kFixedOrder;
  LaneOrientation orientation = LaneOrientation::kNormal;
  bool uses_branch_support = false;
  BackboneLoweringKind lowering_kind = BackboneLoweringKind::kNone;
  double branch_down_offset_m = 0.0;
  bool flipped_from_previous = false;
  LaneFlipReason flip_reason = LaneFlipReason::kNone;
  double turn_angle_deg = 0.0;
};

// Derived backbone output (no generation-input policy mixed in this type).
struct BackboneResult {
  std::vector<SupportNode> nodes{};
  std::vector<BackboneEdge> edges{};
  std::vector<JunctionInfo> junctions{};
  std::vector<BackboneEdgeOrientation> edge_orientations{};
};

} // namespace city::wire
