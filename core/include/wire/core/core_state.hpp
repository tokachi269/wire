#pragma once

#include <optional>
#include <string>
#include <string_view>
#include <type_traits>
#include <unordered_map>
#include <vector>

#include "wire/core/debug_types.hpp"
#include "wire/core/detail_curve.hpp"
#include "wire/core/endpoint_resolution.hpp"
#include "wire/core/entities.hpp"
#include "wire/core/id.hpp"
#include "wire/core/inspection.hpp"
#include "wire/core/object_store.hpp"
#include "wire/core/types.hpp"
#include "wire/core/variation.hpp"
#include "wire/core/workflow_types.hpp"

namespace wire::core {

class CoreView;
namespace state_internal {
struct OverrideResolutionService;
struct EndpointRefreshService;
struct TemplateMutationService;
}

struct EditState {
  // Entity-layer authoritative stores.
  ObjectStore<Pole> poles;
  ObjectStore<Port> ports;
  ObjectStore<Anchor> anchors;
  ObjectStore<Bundle> bundles;
  ObjectStore<Span> spans;
  ObjectStore<Attachment> attachments;
};

struct AttachmentDebugLineEffect {
  ObjectId attachment_id = kInvalidObjectId;
  AttachmentTemplateId template_id = kInvalidAttachmentTemplateId;
  AttachmentLineInteractionMode interaction_mode = AttachmentLineInteractionMode::kPassThrough;
  double center_length_m = 0.0;
  CurveLengthInterval affected_interval{};
  bool uses_internal_path = false;
};

struct GeometrySettings {
  // Persisted generation settings. Derived caches are rebuilt from these.
  int curve_samples = 8;
  bool sag_enabled = false;
  double sag_factor = 0.03;
  // Pole skin clearance used to avoid center-line overlap with pole body.
  double pole_clearance_m = 0.05;
};

struct CurveCacheEntry {
  DetailCurve detail{};
  std::vector<Vec3d> points{};
  std::uint64_t source_version = 0;
};

struct CurveCache {
  std::unordered_map<ObjectId, CurveCacheEntry> by_span{};
};

struct BoundsCacheEntry {
  AABBd whole{};
  std::vector<AABBd> segments{};
  std::uint64_t source_version = 0;
};

struct BoundsCache {
  std::unordered_map<ObjectId, BoundsCacheEntry> by_span{};
};

enum class VisualPartKind : std::uint8_t {
  kSupportArm = 0,
  kInsulator = 1,
  kFitting = 2,
};

struct VisualPart {
  VisualPartKind kind = VisualPartKind::kSupportArm;
  Vec3d a{};
  Vec3d b{};
  double radius_m = 0.02;
};

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
  bool operator==(const LoweredSupportGroupKey& other) const {
    return owner_pole_id == other.owner_pole_id && support_group_id == other.support_group_id;
  }
};

[[nodiscard]] inline LoweredSupportGroupKey LoweredSupportGroupKeyFromDecision(
    const EndpointContinuityDecision& decision) {
  return {decision.owner_pole_id, decision.support_group_id};
}

struct LoweredSupportGroupKeyHash {
  std::size_t operator()(const LoweredSupportGroupKey& key) const {
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
  JunctionRelationKind relation_kind = JunctionRelationKind::kNone;
  ContinuityCategoryClass continuity_class = ContinuityCategoryClass::kPointLike;
  bool default_lower_required = false;
  SupportLayoutOriginKind origin = SupportLayoutOriginKind::kFallback;
  SupportLayoutEndpointSourceKind endpoint_source = SupportLayoutEndpointSourceKind::kFallback;
  PortPlacementSourceKind port_source = PortPlacementSourceKind::kUnknown;
  OrderDecisionPolicyKind order_decision_policy = OrderDecisionPolicyKind::kFixedOrder;
  OrderDecisionChoiceKind order_decision_choice = OrderDecisionChoiceKind::kNormal;
  OrderDecisionChoiceReason order_decision_choice_reason = OrderDecisionChoiceReason::kFixedOrder;
  SlotSide side = SlotSide::kCenter;
  SideAssignmentRuleKind side_assignment_rule = SideAssignmentRuleKind::kPoleLocal;
  SupportOrientationRuleKind support_orientation_rule = SupportOrientationRuleKind::kRadial;
  bool used_junction_pair_side_assignment = false;
  bool has_side_axis = false;
  Vec3d side_axis{};
  double chosen_side_sign = 0.0;
  CurveEndpointMode endpoint_mode = CurveEndpointMode::kDirectThrough;
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
  HierarchicalVariationSample down_offset_variation{};
};

struct SupportGroupDecision {
  ObjectId owner_pole_id = kInvalidObjectId;
  int support_group_id = -1;
  EndpointContinuityDecision decision{};
  ObjectId pair_peer_low = kInvalidObjectId;
  ObjectId pair_peer_high = kInvalidObjectId;
  SlotSide side = SlotSide::kCenter;
  SupportLayoutOriginKind origin = SupportLayoutOriginKind::kFallback;
  OrderDecisionPolicyKind order_decision_policy = OrderDecisionPolicyKind::kFixedOrder;
  OrderDecisionChoiceKind order_decision_choice = OrderDecisionChoiceKind::kNormal;
  OrderDecisionChoiceReason order_decision_choice_reason = OrderDecisionChoiceReason::kFixedOrder;
  SideAssignmentRuleKind side_assignment_rule = SideAssignmentRuleKind::kPoleLocal;
  SupportOrientationRuleKind support_orientation_rule = SupportOrientationRuleKind::kRadial;
  bool used_junction_pair_side_assignment = false;
  bool has_side_axis = false;
  Vec3d side_axis{};
  double chosen_side_sign = 0.0;
  double down_offset_m = 0.0;
  Vec3d support_world{};
  HierarchicalVariationSample down_offset_variation{};
  int grouped_port_count = 0;
  std::vector<Vec3d> attachment_worlds{};
};

struct LoweredSupportGroupPlacement {
  ObjectId owner_pole_id = kInvalidObjectId;
  EndpointContinuityDecision decision{};
  ObjectId pair_peer_low = kInvalidObjectId;
  ObjectId pair_peer_high = kInvalidObjectId;
  SlotSide side = SlotSide::kCenter;
  SupportLayoutOriginKind origin = SupportLayoutOriginKind::kFallback;
  SupportGroupingRuleKind grouping_rule = SupportGroupingRuleKind::kDecisionGroup;
  int support_group_id = -1;
  int grouped_port_count = 1;
  OrderDecisionPolicyKind order_decision_policy = OrderDecisionPolicyKind::kFixedOrder;
  OrderDecisionChoiceKind order_decision_choice = OrderDecisionChoiceKind::kNormal;
  OrderDecisionChoiceReason order_decision_choice_reason = OrderDecisionChoiceReason::kFixedOrder;
  SideAssignmentRuleKind side_assignment_rule = SideAssignmentRuleKind::kPoleLocal;
  SupportOrientationRuleKind support_orientation_rule = SupportOrientationRuleKind::kRadial;
  bool used_junction_pair_side_assignment = false;
  bool has_side_axis = false;
  Vec3d side_axis{};
  double chosen_side_sign = 0.0;
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
  SupportLayoutEndpoint start{};
  SupportLayoutEndpoint end{};
  std::vector<LoweredSupportGroupKey> lowered_support_group_keys{};
  std::uint64_t source_version = 0;
};

struct SupportLayoutCache {
  std::unordered_map<ObjectId, SpanSupportLayoutEntry> by_span{};
  std::unordered_map<LoweredSupportGroupKey, SupportGroupDecision, LoweredSupportGroupKeyHash> support_group_decisions{};
  std::unordered_map<LoweredSupportGroupKey, LoweredSupportGroupPlacement, LoweredSupportGroupKeyHash> lowered_support_groups{};
};

struct PoleOrientationOverride {
  std::optional<double> manual_yaw_deg{};
  std::optional<bool> flip_180{};
  std::uint64_t version = 1;
};

struct SpanEndpointOverride {
  std::optional<int> socket_a_id{};
  std::optional<int> socket_b_id{};
  std::uint64_t version = 1;
};

struct SpanSupportOverride {
  std::optional<double> branch_down_offset_m{};
  std::uint64_t version = 1;
};

struct OverrideState {
  std::unordered_map<ObjectId, PoleOrientationOverride> pole_orientation_by_pole{};
  std::unordered_map<ObjectId, SpanEndpointOverride> span_endpoint_by_span{};
  std::unordered_map<ObjectId, SpanSupportOverride> span_support_by_span{};
};

struct SpanVisualCacheEntry {
  std::vector<VisualPart> parts{};
  std::uint64_t source_version = 0;
};

struct VisualCache {
  std::unordered_map<ObjectId, SpanVisualCacheEntry> by_span{};
};

struct SpanRenderCacheEntry {
  // Render-only appearance data derived from templates + current span geometry.
  double wire_radius_m = 0.015;
  std::uint32_t color_rgba = 0xFFFFFFFFu;
  CableMaterialStyleKind material_style = CableMaterialStyleKind::kGeneric;
  std::vector<float> arc_length_m_by_point{};
  std::vector<float> arc_length_normalized_by_point{};
  std::vector<float> segment_length_m{};
  std::uint64_t source_version = 0;
};

struct RenderCache {
  std::unordered_map<ObjectId, SpanRenderCacheEntry> by_span{};
};

struct VisualSettings {
  bool enable_support_structures = true;
  bool enable_insulators = true;
  double support_center_threshold_m = 0.03;
  double support_arm_extra_m = 0.20;
  double insulator_radius_m = 0.07;
  double insulator_length_m = 0.16;
};

struct CacheState {
  // Derived cache layer. Not treated as authoritative topology state.
  GeometrySettings geometry_settings{};
  CurveCache curve_cache{};
  BoundsCache bounds_cache{};
  VisualSettings visual_settings{};
  VariationSettings variation_settings{};
  SupportLayoutCache support_layout_cache{};
  VisualCache visual_cache{};
  RenderCache render_cache{};
};

struct TemplateDependencyState {
  // Topology-affecting template edits must not be applied as render-only cache updates.
  std::vector<ObjectId> bundles_requiring_regeneration{};
  std::vector<std::uint64_t> sessions_requiring_regeneration{};
};

inline constexpr double kDefaultCornerThresholdDeg = 70.0;

struct LayoutSettings {
  bool angle_correction_enabled = true;
  double corner_threshold_deg = kDefaultCornerThresholdDeg;
  double min_side_scale = 1.0;
  double max_side_scale = 1.8;
};

struct ConnectionIndex {
  std::unordered_map<ObjectId, std::vector<ObjectId>> spans_by_port;
  std::unordered_map<ObjectId, std::vector<ObjectId>> spans_by_anchor;
};

struct RelationIndex {
  std::unordered_map<ObjectId, std::vector<ObjectId>> ports_by_pole;
  std::unordered_map<ObjectId, std::vector<ObjectId>> anchors_by_pole;
  std::unordered_map<ObjectId, std::vector<ObjectId>> spans_by_bundle;
  std::unordered_map<ObjectId, std::vector<ObjectId>> attachments_by_span;
};

enum class DirtyBits : std::uint32_t {
  kNone = 0,
  kTopology = 1u << 0,
  kGeometry = 1u << 1,
  kBounds = 1u << 2,
  kRender = 1u << 3,
  kRaycast = 1u << 4,
};

inline DirtyBits operator|(DirtyBits a, DirtyBits b) {
  using T = std::underlying_type_t<DirtyBits>;
  return static_cast<DirtyBits>(static_cast<T>(a) | static_cast<T>(b));
}

inline DirtyBits operator&(DirtyBits a, DirtyBits b) {
  using T = std::underlying_type_t<DirtyBits>;
  return static_cast<DirtyBits>(static_cast<T>(a) & static_cast<T>(b));
}

inline DirtyBits operator~(DirtyBits a) {
  using T = std::underlying_type_t<DirtyBits>;
  return static_cast<DirtyBits>(~static_cast<T>(a));
}

inline DirtyBits& operator|=(DirtyBits& a, DirtyBits b) {
  a = a | b;
  return a;
}

inline bool any(DirtyBits bits, DirtyBits flag) { return (bits & flag) != DirtyBits::kNone; }

struct SpanRuntimeState {
  // Derived runtime/version state for incremental recomputation.
  ObjectId span_id = kInvalidObjectId;
  std::uint64_t data_version = 0;
  std::uint64_t geometry_version = 0;
  std::uint64_t bounds_version = 0;
  std::uint64_t render_version = 0;
  std::uint64_t raycast_version = 0;
  std::uint64_t variation_flow_key = 0;
  DirtyBits dirty_bits = DirtyBits::kNone;
};

struct DirtyQueue {
  // Derived work queues. Content is rebuildable from edit operations.
  std::vector<ObjectId> topology_dirty_span_ids;
  std::vector<ObjectId> geometry_dirty_span_ids;
  std::vector<ObjectId> bounds_dirty_span_ids;
  std::vector<ObjectId> render_dirty_span_ids;
  std::vector<ObjectId> raycast_dirty_span_ids;
};

struct RecalcStats {
  std::size_t topology_processed = 0;
  std::size_t geometry_processed = 0;
  std::size_t bounds_processed = 0;
  std::size_t render_processed = 0;
  std::size_t raycast_processed = 0;

  [[nodiscard]] std::size_t total_processed() const {
    return topology_processed + geometry_processed + bounds_processed + render_processed + raycast_processed;
  }
};

struct ChangeSet {
  std::vector<ObjectId> created_ids;
  std::vector<ObjectId> updated_ids;
  std::vector<ObjectId> deleted_ids;
  std::vector<ObjectId> dirty_span_ids;
};

template <typename TValue> struct EditResult {
  bool ok = false;
  TValue value{};
  std::string error{};
  ChangeSet change_set{};
};

enum class ValidationSeverity : std::uint8_t {
  kError = 0,
  kWarning = 1,
};

struct ValidationIssue {
  ValidationSeverity severity = ValidationSeverity::kError;
  std::string code{};
  std::string message{};
  ObjectId object_id = kInvalidObjectId;
};

struct ValidationResult {
  std::vector<ValidationIssue> issues;

  [[nodiscard]] bool has_errors() const;
  [[nodiscard]] bool ok() const { return !has_errors(); }
};

struct CommitOptions {
  bool run_recalc = true;
  bool run_validate_fast = true;
  bool run_validate = false;
};

struct CommitResult {
  RecalcStats recalc_stats{};
  ValidationResult validation{};
};

struct AddConnectionByPoleOptions {
  SpanKind span_kind = SpanKind::kDistribution;
  SpanLayer span_layer = SpanLayer::kUnknown;
  ObjectId bundle_id = kInvalidObjectId;
  BundleKind bundle_template_id = BundleKind::kLowVoltage;
  bool use_bundle_template = false;
  bool auto_create_bundle = true;
  bool allow_generate_port = true;
  ConnectionContext connection_context = ConnectionContext::kTrunkContinue;
  PoleContextKind pole_context_a = PoleContextKind::kStraight;
  PoleContextKind pole_context_b = PoleContextKind::kStraight;
  double corner_angle_deg_a = 0.0;
  double corner_angle_deg_b = 0.0;
  double corner_turn_sign_a = 0.0;
  double corner_turn_sign_b = 0.0;
  ObjectId reference_span_id = kInvalidObjectId;
  std::uint32_t branch_index = 0;
  ObjectId preferred_port_a_id = kInvalidObjectId;
  ObjectId preferred_port_b_id = kInvalidObjectId;
};

struct ResolveBranchPickOptions {
  std::vector<BundleKind> selected_bundle_template_ids{};
  double snap_radius_world = 0.6;
  bool create_midair_node = true;
  bool enforce_midair_template_policy = true;
};

class CoreState {
public:
  CoreState();

  struct AddConnectionByPoleResult {
    ObjectId span_id = kInvalidObjectId;
    ObjectId port_a_id = kInvalidObjectId;
    ObjectId port_b_id = kInvalidObjectId;
  };

  struct AddDropResult {
    ObjectId span_id = kInvalidObjectId;
    ObjectId source_port_id = kInvalidObjectId;
    ObjectId target_port_id = kInvalidObjectId;
    ObjectId split_port_id = kInvalidObjectId;
  };

  struct GenerateSimpleLineResult {
    std::vector<ObjectId> pole_ids{};
    std::vector<ObjectId> span_ids{};
    std::uint64_t generation_session_id = 0;
  };

  struct GenerateBundleFromPathResult {
    ObjectId bundle_id = kInvalidObjectId;
    std::vector<ObjectId> bundle_ids{};
    std::vector<ObjectId> generated_span_ids{};
    std::vector<ObjectId> generated_pole_ids{};
  };

  struct PoleDetailInfo {
    const Pole* pole = nullptr;
    const PoleTypeDefinition* pole_type = nullptr;
    std::vector<const Port*> owned_ports{};
    std::vector<const Anchor*> owned_anchors{};
  };

  EditResult<ObjectId> AddPole(const Transformd& world_transform, double height_m = 10.0, std::string_view name = {},
                               PoleKind kind = PoleKind::kGeneric,
                               PlacementMode placement_mode = PlacementMode::kAuto);
  EditResult<ObjectId> AddPort(ObjectId owner_pole_id, const Vec3d& world_position, PortKind kind = PortKind::kGeneric,
                               PortLayer layer = PortLayer::kUnknown);
  EditResult<ObjectId> AddPort(ObjectId owner_pole_id, const Vec3d& world_position, PortKind kind,
                               PortLayer layer, const Frame3d& direction);
  EditResult<ObjectId> AddAnchor(ObjectId owner_pole_id, const Vec3d& world_position,
                                 AnchorSupportKind support_kind = AnchorSupportKind::kGeneric,
                                 double support_strength = 1.0);
  EditResult<ObjectId> AddBundle(int conductor_count, double phase_spacing_m,
                                 BundleKind kind = BundleKind::kLowVoltage);
  EditResult<ObjectId> AddSpan(ObjectId port_a_id, ObjectId port_b_id, SpanKind kind = SpanKind::kGeneric,
                               SpanLayer layer = SpanLayer::kUnknown, ObjectId bundle_id = kInvalidObjectId,
                               ObjectId anchor_a_id = kInvalidObjectId, ObjectId anchor_b_id = kInvalidObjectId);
  EditResult<ObjectId> AddAttachment(ObjectId span_id, double t, AttachmentKind kind = AttachmentKind::kGeneric,
                                     double display_offset_m = 0.0,
                                     AttachmentTemplateId template_id = kInvalidAttachmentTemplateId);
  EditResult<ObjectId> MovePole(ObjectId pole_id, const Transformd& new_world_transform);
  // Pole tilt is instance-owned. This command updates explicit target poles without touching templates.
  EditResult<bool> ApplyPoleTilt(const std::vector<ObjectId>& pole_ids, double max_tilt_deg);
  EditResult<ObjectId> SetPoleTilt(ObjectId pole_id, double max_tilt_deg);
  EditResult<bool> SetAllPoleTilt(double max_tilt_deg);
  EditResult<ObjectId> MovePort(ObjectId port_id, const Vec3d& new_world_position);
  EditResult<ObjectId> SetPortWorldPositionManual(ObjectId port_id, const Vec3d& new_world_position);
  EditResult<ObjectId> ResetPortPositionToAuto(ObjectId port_id);
  EditResult<ObjectId> MoveAnchor(ObjectId anchor_id, const Vec3d& new_world_position);
  EditResult<ObjectId> DeletePole(ObjectId pole_id);
  EditResult<ObjectId> DeleteSpan(ObjectId span_id);

  struct SplitSpanResult {
    ObjectId old_span_id = kInvalidObjectId;
    ObjectId new_port_id = kInvalidObjectId;
    ObjectId new_span_a_id = kInvalidObjectId;
    ObjectId new_span_b_id = kInvalidObjectId;
  };
  EditResult<SplitSpanResult> SplitSpan(ObjectId span_id, double t);
  EditResult<ObjectId> ApplyPoleType(ObjectId pole_id, PoleTypeId pole_type_id);
  // When template/bundle is supplied in options, connection behavior is derived from that template,
  // and `category` is treated as a fallback label only.
  EditResult<AddConnectionByPoleResult> AddConnectionByPole(ObjectId pole_a_id, ObjectId pole_b_id,
                                                            ConnectionCategory category);
  EditResult<AddConnectionByPoleResult> AddConnectionByPole(ObjectId pole_a_id, ObjectId pole_b_id,
                                                            ConnectionCategory category,
                                                            const AddConnectionByPoleOptions& options);
  EditResult<AddDropResult> AddDropFromPole(ObjectId source_pole_id, const Vec3d& target_world_position,
                                            ConnectionCategory category = ConnectionCategory::kDrop);
  EditResult<AddDropResult> AddDropFromSpan(ObjectId source_span_id, double t, const Vec3d& target_world_position,
                                            ConnectionCategory category = ConnectionCategory::kDrop);
  EditResult<std::vector<ObjectId>> GeneratePolesAlongRoad(const RoadSegment& road, double interval,
                                                           PoleTypeId pole_type_id);
  EditResult<std::vector<ObjectId>> GenerateSpansBetweenPoles(const std::vector<ObjectId>& poles,
                                                              ConnectionCategory category);
  EditResult<GenerateSimpleLineResult> GenerateSimpleLine(const RoadSegment& road, double interval,
                                                          PoleTypeId pole_type_id, ConnectionCategory category);
  EditResult<GenerateSimpleLineResult> GenerateSimpleLineFromPoints(const RoadSegment& road, PoleTypeId pole_type_id,
                                                                    ConnectionCategory category);
  // Canonical path-generation API.
  EditResult<GenerateBundleFromPathResult> GenerateFromBackboneSpec(const BackboneSpec& spec);
  EditResult<GenerateBundleFromPathResult> RegenerateSessionAutoParts(std::uint64_t generation_session_id,
                                                                      const BackboneSpec& spec);
  enum class PickBranchResolutionKind : std::uint8_t {
    kNode = 0,
    kMidair = 1,
  };
  struct ResolveBranchPickResult {
    PickBranchResolutionKind resolution = PickBranchResolutionKind::kNode;
    ObjectId resolved_node_id = kInvalidObjectId;
    SupportKind support_kind = SupportKind::kPole;
    Vec3d position{};
    bool snapped_from_segment_endpoint = false;
  };
  // Interprets a viewer-side pick payload and updates support-node session state when needed.
  EditResult<ResolveBranchPickResult> ResolveBranchPick(const PickResult& pick);
  EditResult<ResolveBranchPickResult> ResolveBranchPick(const PickResult& pick,
                                                        const ResolveBranchPickOptions& options);
  EditResult<ObjectId> SetPolePlacementMode(ObjectId pole_id, PlacementMode mode);
  EditResult<ObjectId> SetPoleFlip180(ObjectId pole_id, bool flip_180);
  EditResult<ObjectId> SetPoleManualYawOverride(ObjectId pole_id, double manual_yaw_deg);
  EditResult<ObjectId> ClearPoleOrientationOverride(ObjectId pole_id);
  EditResult<ObjectId> SetSpanEndpointSocketOverride(ObjectId span_id, bool is_start_endpoint, int socket_id);
  EditResult<ObjectId> ClearSpanEndpointSocketOverride(ObjectId span_id, bool is_start_endpoint);
  EditResult<ObjectId> SetSpanBranchDownOffsetOverride(ObjectId span_id, double branch_down_offset_m);
  EditResult<ObjectId> ClearSpanBranchDownOffsetOverride(ObjectId span_id);
  [[nodiscard]] PoleDetailInfo GetPoleDetail(ObjectId pole_id) const;
  [[nodiscard]] std::vector<ObjectId> GetSpansByBundle(ObjectId bundle_id) const;
  [[nodiscard]] BackboneResult BuildBackboneResult() const;
  [[nodiscard]] std::vector<BackboneEdge> BuildBackboneEdges() const;
  [[nodiscard]] std::vector<ObjectId> FindBackboneRoute(ObjectId start_node_id, ObjectId end_node_id) const;
  EditResult<bool> UpdateGeometrySettings(const GeometrySettings& settings, bool mark_all_spans_dirty = true);
  EditResult<bool> UpdateLayoutSettings(const LayoutSettings& settings);
  EditResult<bool> UpdateVisualSettings(const VisualSettings& settings, bool mark_all_spans_dirty = true);
  EditResult<bool> UpdateVariationSettings(const VariationSettings& settings, bool mark_all_spans_dirty = true);
  EditResult<bool> UpdateCableTemplate(const CableTemplate& cable_template);
  EditResult<bool> UpdateCableTemplate(const CableTemplate& cable_template,
                                       const std::vector<ObjectId>& preferred_visible_span_ids);
  EditResult<bool> UpdateBundleTemplate(const BundleTemplate& bundle_template);
  EditResult<bool> UpdateAttachmentTemplate(const AttachmentTemplate& attachment_template,
                                            bool mark_dependent_spans_dirty = true);
  EditResult<bool> ResetAllSpanReferenceLengths(bool mark_all_spans_dirty = true);
  [[nodiscard]] const CurveCacheEntry* find_curve_cache(ObjectId span_id) const;
  [[nodiscard]] const BoundsCacheEntry* find_bounds_cache(ObjectId span_id) const;
  [[nodiscard]] const SpanSupportLayoutEntry* find_span_support_layout(ObjectId span_id) const;
  [[nodiscard]] const SpanVisualCacheEntry* find_span_visual_cache(ObjectId span_id) const;
  [[nodiscard]] const SpanRenderCacheEntry* find_span_render_cache(ObjectId span_id) const;
  [[nodiscard]] const AttachmentTemplate* find_attachment_template(AttachmentTemplateId attachment_template_id) const;
  [[nodiscard]] ValidationResult ValidateFast() const;

  [[nodiscard]] CommitResult Commit();
  [[nodiscard]] CommitResult Commit(const CommitOptions& options);

  [[nodiscard]] ObjectId next_id() const { return id_generator_.peek(); }
  void clear_path_direction_debug_records() { path_direction_debug_records_.clear(); }
  void clear_port_resolution_debug_records() { port_resolution_debug_records_.clear(); }

  [[nodiscard]] CoreView view() const;

private:
#if defined(WIRE_INTERNAL) || defined(WIRE_TESTING)
  friend struct CoreStateTestHook;
#endif
  friend class CoreView;
  friend struct state_internal::OverrideResolutionService;
  friend struct state_internal::EndpointRefreshService;
  friend struct state_internal::TemplateMutationService;
  void remove_span_from_indexes(const Span& span);
  void add_span_to_index(const Span& span);
  void initialize_span_runtime_state(ObjectId span_id);
  void mark_span_dirty(ObjectId span_id, DirtyBits dirty_bits, bool bump_data_version);
  void add_dirty_queue(ObjectId span_id, DirtyBits dirty_bits);
  void mark_connected_spans_dirty_from_port(ObjectId port_id, DirtyBits dirty_bits, ChangeSet* change_set);
  void mark_connected_spans_dirty_from_anchor(ObjectId anchor_id, DirtyBits dirty_bits, ChangeSet* change_set);
  [[nodiscard]] bool rebuild_span_curve(ObjectId span_id, std::string* error_message);
  [[nodiscard]] bool rebuild_span_bounds(ObjectId span_id, std::string* error_message);
  [[nodiscard]] bool rebuild_span_visual(ObjectId span_id, std::string* error_message);
  [[nodiscard]] SpanSupportLayoutEntry generate_span_support_layout(const Span& span, std::string* error_message) const;
  [[nodiscard]] DetailCurve generate_span_curve(const Span& span, const SpanSupportLayoutEntry& support_layout,
                                                std::string* error_message) const;
  [[nodiscard]] static AABBd build_aabb_from_points(const std::vector<Vec3d>& points);
  [[nodiscard]] static AABBd build_aabb_from_two_points(const Vec3d& a, const Vec3d& b);
  void cache_span_support_layout(SpanSupportLayoutEntry layout);
  void erase_cached_span_support_layout(ObjectId span_id);
  void remove_span_from_caches(ObjectId span_id);
  void rebuild_lowered_support_groups_for_span(ObjectId span_id);
  [[nodiscard]] static std::vector<Vec3d> sample_polyline_points(const std::vector<Vec3d>& polyline, double interval);
  EditResult<std::vector<ObjectId>> generate_poles_from_points(const RoadSegment& road, PoleTypeId pole_type_id,
                                                               const std::vector<Vec3d>& points);
  EditResult<std::vector<ObjectId>>
  generate_grouped_spans_between_support_nodes(const std::vector<ObjectId>& node_ids,
                                               const std::unordered_map<ObjectId, SupportNode>& support_node_by_id,
                                               ObjectId bundle_id, ConnectionCategory category, int conductor_count,
                                               double spacing_m, bool maintain_lane_order, bool allow_lane_mirror,
                                               OrderDecisionPolicyKind order_decision_policy,
                                               BackboneFlowKind flow_kind, const BackboneLoweringPolicy& lowering_policy,
                                               const std::unordered_map<ObjectId, JunctionRelation>* junction_relations_by_node,
                                               std::vector<SegmentLaneAssignment>* out_lane_assignments,
                                               std::vector<BackboneEdgeOrientation>* out_edge_orientations = nullptr,
                                               BundleKind bundle_template_id = BundleKind::kLowVoltage);
  [[nodiscard]] static std::uint64_t hash_path_points(const std::vector<Vec3d>& points);
  [[nodiscard]] double effective_pole_yaw_deg(const Pole& pole) const;
  [[nodiscard]] double effective_pole_layout_yaw_deg(const Pole& pole) const;
  [[nodiscard]] Vec3d to_local_on_pole(const Pole& pole, const Vec3d& world) const;
  [[nodiscard]] SlotSide preferred_side_from_geometry(const Pole& pole, const Pole* peer, double eps) const;
  [[nodiscard]] static double polyline_length(const std::vector<Vec3d>& polyline);
  [[nodiscard]] static PortLayer category_to_port_layer(ConnectionCategory category);
  [[nodiscard]] static SpanLayer category_to_span_layer(ConnectionCategory category);
  [[nodiscard]] static BundleKind category_to_bundle_kind(ConnectionCategory category);
  [[nodiscard]] static PortKind category_to_port_kind(ConnectionCategory category);
  [[nodiscard]] bool has_pole_orientation_override(ObjectId pole_id) const;
  [[nodiscard]] bool has_span_endpoint_socket_override(ObjectId span_id, bool is_start_endpoint) const;
  [[nodiscard]] bool has_span_branch_down_offset_override(ObjectId span_id) const;
  [[nodiscard]] std::optional<double> resolve_pole_manual_yaw_override(const Pole& pole) const;
  [[nodiscard]] std::optional<bool> resolve_pole_flip_180_override(const Pole& pole) const;
  [[nodiscard]] int resolve_span_endpoint_socket_id(const Span& span, bool is_start_endpoint) const;
  [[nodiscard]] double resolve_span_branch_down_offset_m(const Span& span, double automatic_value) const;
  void register_default_pole_types();
  void register_default_cable_templates();
  void register_default_bundle_templates();
  void register_default_attachment_templates();
  [[nodiscard]] const PoleTypeDefinition* find_pole_type(PoleTypeId pole_type_id) const;
  [[nodiscard]] const CableTemplate* find_cable_template(CableTemplateId cable_template_id) const;
  [[nodiscard]] const BundleTemplate* find_bundle_template(BundleKind bundle_template_id) const;
  [[nodiscard]] std::vector<PortPlacementBand> sorted_port_bands(const PoleTypeDefinition& pole_type,
                                                                 ConnectionCategory category) const;
  [[nodiscard]] bool is_port_band_used(ObjectId pole_id, const PortPlacementBand& band) const;

  struct PortResolutionRequest {
    ObjectId pole_id = kInvalidObjectId;
    ObjectId peer_pole_id = kInvalidObjectId;
    ObjectId reference_span_id = kInvalidObjectId;
    ConnectionCategory category = ConnectionCategory::kLowVoltage;
    ConnectionContext connection_context = ConnectionContext::kTrunkContinue;
    PoleContextKind pole_context = PoleContextKind::kStraight;
    double corner_angle_deg = 0.0;
    double corner_turn_sign = 0.0;
    bool allow_generate_port = true;
    bool prefer_template_match = false;
    int preferred_template_layer = 1;
    SlotSide preferred_template_side = SlotSide::kCenter;
    SlotRole preferred_template_role = SlotRole::kNeutral;
    std::uint32_t branch_index = 0;
    // Authoritative endpoint placement decision. Port resolution must not reinterpret
    // continuity/lowering policy through a parallel hint path.
    EndpointContinuityDecision endpoint_decision{};
    std::vector<ObjectId> excluded_port_ids{};
  };

  EditResult<ObjectId> ensure_pole_connection_port(const PortResolutionRequest& request);
  [[nodiscard]] static std::uint8_t deterministic_tiebreak_0_255(ObjectId pole_id, int tiebreak_key,
                                                                 ConnectionCategory category, ConnectionContext context,
                                                                 ObjectId peer_pole_id, ObjectId reference_span_id,
                                                                 std::uint32_t branch_index);
  [[nodiscard]] static bool is_valid_slot_side(SlotSide side);
  [[nodiscard]] static bool is_valid_slot_role(SlotRole role);
  [[nodiscard]] static int inversion_count(const std::vector<double>& values);
  [[nodiscard]] double compute_side_scale(PoleContextKind context, double corner_angle_deg) const;
  [[nodiscard]] static double compute_corner_angle_deg(const Vec3d& prev, const Vec3d& curr, const Vec3d& next);
  [[nodiscard]] static double compute_corner_turn_sign_xy(const Vec3d& prev, const Vec3d& curr, const Vec3d& next);
  [[nodiscard]] PoleContextInfo classify_pole_context_from_path(const std::vector<Vec3d>& points, std::size_t index,
                                                                std::size_t pending_degree) const;
  EditResult<ObjectId> ensure_bundle_for_template(const AddConnectionByPoleOptions& options);
  static void add_unique_id(std::vector<ObjectId>& ids, ObjectId id);
  static void index_add(std::unordered_map<ObjectId, std::vector<ObjectId>>& map, ObjectId key, ObjectId value);
  static void index_remove(std::unordered_map<ObjectId, std::vector<ObjectId>>& map, ObjectId key, ObjectId value);
  static std::string dirty_bits_to_string(DirtyBits bits);
  static void apply_pole_placement_mode(Pole& pole, PlacementMode mode);
  static void apply_port_position_mode(Port& port, PortPositionMode mode, PortPlacementSourceKind source_hint);
  [[nodiscard]] double pole_radius_at_height_m(const Pole& pole, double local_z_m) const;
  [[nodiscard]] Vec3d apply_pole_clearance_to_local(const Pole& pole, const Vec3d& local, SlotSide side) const;
  void finalize_pole_transform_update(ObjectId pole_id, const Pole& old_pole, ChangeSet* change_set);
  std::string next_display_id(std::string_view prefix);
  void refresh_owned_endpoints_from_pole(ObjectId pole_id, ChangeSet* change_set, const Pole* previous_pole = nullptr,
                                         const double* previous_layout_yaw_override = nullptr);
  [[nodiscard]] EditState& edit_state_access() { return edit_state_; }
  [[nodiscard]] const EditState& edit_state_access() const { return edit_state_; }
  [[nodiscard]] ConnectionIndex& connection_index_access() { return connection_index_; }
  [[nodiscard]] const ConnectionIndex& connection_index_access() const { return connection_index_; }
  [[nodiscard]] RelationIndex& relation_index_access() { return relation_index_; }
  [[nodiscard]] const RelationIndex& relation_index_access() const { return relation_index_; }
  [[nodiscard]] std::unordered_map<ObjectId, SpanRuntimeState>& span_runtime_states_access() {
    return span_runtime_states_;
  }
  [[nodiscard]] const std::unordered_map<ObjectId, SpanRuntimeState>& span_runtime_states_access() const {
    return span_runtime_states_;
  }
  [[nodiscard]] DirtyQueue& dirty_queue_access() { return dirty_queue_; }
  [[nodiscard]] const DirtyQueue& dirty_queue_access() const { return dirty_queue_; }
  [[nodiscard]] CacheState& cache_state_access() { return cache_state_; }
  [[nodiscard]] const CacheState& cache_state_access() const { return cache_state_; }
  [[nodiscard]] std::uint64_t& next_generation_session_id_access() { return next_generation_session_id_; }
  [[nodiscard]] std::vector<PathDirectionEvaluationDebug>& path_direction_debug_records_access() {
    return path_direction_debug_records_;
  }

  [[nodiscard]] static bool has_zero_length(const Port& a, const Port& b);
  RecalcStats ProcessDirtyQueues();
  [[nodiscard]] ValidationResult Validate() const;
  [[nodiscard]] const SpanRuntimeState* find_span_runtime_state(ObjectId span_id) const;
  [[nodiscard]] static std::unordered_map<ObjectId, std::vector<ObjectId>>
  make_expected_port_index(const EditState& edit_state);
  [[nodiscard]] static std::unordered_map<ObjectId, std::vector<ObjectId>>
  make_expected_anchor_index(const EditState& edit_state);
  [[nodiscard]] static std::unordered_map<ObjectId, std::vector<ObjectId>>
  make_expected_pole_port_index(const EditState& edit_state);
  [[nodiscard]] static std::unordered_map<ObjectId, std::vector<ObjectId>>
  make_expected_pole_anchor_index(const EditState& edit_state);
  [[nodiscard]] static std::unordered_map<ObjectId, std::vector<ObjectId>>
  make_expected_bundle_span_index(const EditState& edit_state);
  [[nodiscard]] static std::unordered_map<ObjectId, std::vector<ObjectId>>
  make_expected_span_attachment_index(const EditState& edit_state);

  IdGenerator id_generator_{};
  std::uint64_t next_data_version_ = 1;
  std::uint64_t next_generation_session_id_ = 1;
  std::unordered_map<std::string, std::uint64_t> display_id_counters_{};
  // PersistCore entity layer.
  EditState edit_state_{};
  ConnectionIndex connection_index_{};
  RelationIndex relation_index_{};
  std::unordered_map<PoleTypeId, PoleTypeDefinition> pole_types_{};
  std::unordered_map<CableTemplateId, CableTemplate> cable_templates_{};
  std::unordered_map<BundleKind, BundleTemplate> bundle_templates_{};
  std::unordered_map<AttachmentTemplateId, AttachmentTemplate> attachment_templates_{};
  TemplateDependencyState template_dependency_state_{};
  OverrideState override_state_{};
  // Derived cache/runtime layer.
  std::unordered_map<ObjectId, SpanRuntimeState> span_runtime_states_{};
  DirtyQueue dirty_queue_{};
  RecalcStats last_recalc_stats_{};
  CacheState cache_state_{};
  // Persisted/authoritative generation policy layer.
  LayoutSettings layout_settings_{};
  // Session debug layer (non-authoritative, non-persist by policy).
  PathDirectionEvaluationDebug last_path_direction_debug_{};
  std::vector<PathDirectionEvaluationDebug> path_direction_debug_records_{};
  std::unordered_map<ObjectId, PoleOrientationDebugRecord> pole_orientation_debug_records_{};
  std::vector<SupportNode> last_generation_support_nodes_{};
  std::vector<SegmentLaneAssignment> last_generation_lane_assignments_{};
  std::vector<BackboneEdgeOrientation> last_generation_edge_orientations_{};
  std::unordered_map<ObjectId, JunctionRelation> last_generation_junction_relations_{};
  ObjectId next_virtual_support_node_id_ = 0x9000000000000000ull;
  std::vector<PortResolutionDebugRecord> port_resolution_debug_records_{};
};

class CoreView {
public:
  explicit CoreView(const CoreState& state) : state_(state) {}

  [[nodiscard]] const EditState& edit_state() const { return state_.edit_state_; }
  [[nodiscard]] const ObjectStore<Pole>& poles() const { return state_.edit_state_.poles; }
  [[nodiscard]] const ObjectStore<Port>& ports() const { return state_.edit_state_.ports; }
  [[nodiscard]] const ObjectStore<Anchor>& anchors() const { return state_.edit_state_.anchors; }
  [[nodiscard]] const ObjectStore<Bundle>& bundles() const { return state_.edit_state_.bundles; }
  [[nodiscard]] const ObjectStore<Span>& spans() const { return state_.edit_state_.spans; }
  [[nodiscard]] const ObjectStore<Attachment>& attachments() const { return state_.edit_state_.attachments; }

  [[nodiscard]] const ConnectionIndex& connection_index() const { return state_.connection_index_; }
  [[nodiscard]] const RelationIndex& relation_index() const { return state_.relation_index_; }
  [[nodiscard]] const DirtyQueue& dirty_queue() const { return state_.dirty_queue_; }
  [[nodiscard]] const RecalcStats& last_recalc_stats() const { return state_.last_recalc_stats_; }
  [[nodiscard]] const GeometrySettings& geometry_settings() const { return state_.cache_state_.geometry_settings; }
  [[nodiscard]] const VisualSettings& visual_settings() const { return state_.cache_state_.visual_settings; }
  [[nodiscard]] const VariationSettings& variation_settings() const { return state_.cache_state_.variation_settings; }
  [[nodiscard]] const LayoutSettings& layout_settings() const { return state_.layout_settings_; }
  [[nodiscard]] const PathDirectionEvaluationDebug& last_path_direction_debug() const {
    return state_.last_path_direction_debug_;
  }
  [[nodiscard]] const std::vector<PathDirectionEvaluationDebug>& path_direction_debug_records() const {
    return state_.path_direction_debug_records_;
  }
  [[nodiscard]] const std::unordered_map<ObjectId, PoleOrientationDebugRecord>& pole_orientation_debug_records() const {
    return state_.pole_orientation_debug_records_;
  }
  [[nodiscard]] const std::vector<BackboneEdgeOrientation>& last_generation_edge_orientations() const {
    return state_.last_generation_edge_orientations_;
  }
  [[nodiscard]] const CacheState& cache_state() const { return state_.cache_state_; }
  [[nodiscard]] const std::unordered_map<PoleTypeId, PoleTypeDefinition>& pole_types() const {
    return state_.pole_types_;
  }
  [[nodiscard]] int count_port_bands(PoleTypeId pole_type_id, ConnectionCategory category) const {
    const auto it = state_.pole_types_.find(pole_type_id);
    if (it == state_.pole_types_.end()) {
      return 0;
    }
    int count = 0;
    for (const PortPlacementBand& band : it->second.port_bands) {
      if (band.enabled && band.category == category) {
        ++count;
      }
    }
    return count;
  }
  [[nodiscard]] const std::unordered_map<CableTemplateId, CableTemplate>& cable_templates() const {
    return state_.cable_templates_;
  }
  [[nodiscard]] const std::unordered_map<BundleKind, BundleTemplate>& bundle_templates() const {
    return state_.bundle_templates_;
  }
  [[nodiscard]] const std::unordered_map<AttachmentTemplateId, AttachmentTemplate>& attachment_templates() const {
    return state_.attachment_templates_;
  }
  [[nodiscard]] const TemplateDependencyState& template_dependency_state() const {
    return state_.template_dependency_state_;
  }
  [[nodiscard]] const std::vector<PortResolutionDebugRecord>& port_resolution_debug_records() const {
    return state_.port_resolution_debug_records_;
  }
  [[nodiscard]] const std::vector<SegmentLaneAssignment>& last_lane_assignments() const;
  [[nodiscard]] const std::unordered_map<ObjectId, SpanRuntimeState>& span_runtime_states() const {
    return state_.span_runtime_states_;
  }
  [[nodiscard]] const SpanRuntimeState* find_span_runtime_state(ObjectId span_id) const {
    return state_.find_span_runtime_state(span_id);
  }
  [[nodiscard]] const SpanSupportLayoutEntry* find_span_support_layout(ObjectId span_id) const {
    return state_.find_span_support_layout(span_id);
  }
  [[nodiscard]] const SpanVisualCacheEntry* find_span_visual_cache(ObjectId span_id) const {
    // Internal render/debug cache accessor. Prefer inspect_support_layout() for semantic grouped support reads.
    return state_.find_span_visual_cache(span_id);
  }
  [[nodiscard]] const SpanRenderCacheEntry* find_span_render_cache(ObjectId span_id) const {
    return state_.find_span_render_cache(span_id);
  }
  [[nodiscard]] const AttachmentTemplate* find_attachment_template(AttachmentTemplateId attachment_template_id) const {
    return state_.find_attachment_template(attachment_template_id);
  }
  [[nodiscard]] double pole_radius_at_height_m(const Pole& pole, double local_z_m) const {
    return state_.pole_radius_at_height_m(pole, local_z_m);
  }
  [[nodiscard]] std::optional<EntityMeta> describe_entity(EntityRef ref) const;
  [[nodiscard]] std::vector<DecisionTraceEntry> collect_decision_trace(EntityRef ref) const;
  [[nodiscard]] std::optional<PoleInspectionView> inspect_pole(ObjectId pole_id) const;
  [[nodiscard]] std::optional<SpanInspectionView> inspect_span(ObjectId span_id) const;
  [[nodiscard]] std::optional<SupportLayoutInspectionView> inspect_support_layout(ObjectId span_id) const;
  [[nodiscard]] std::optional<DetailCurveInspectionView> inspect_detail_curve(ObjectId span_id) const;
  [[nodiscard]] std::optional<JunctionInspectionView> inspect_junction(ObjectId node_id) const;
  [[nodiscard]] std::optional<TemplateInspectionView> inspect_pole_template(PoleTypeId pole_type_id) const;
  [[nodiscard]] std::optional<TemplateInspectionView> inspect_cable_template(CableTemplateId cable_template_id) const;
  [[nodiscard]] std::optional<TemplateInspectionView> inspect_bundle_template(BundleKind bundle_template_id) const;
  [[nodiscard]] std::optional<TemplateInspectionView>
  inspect_attachment_template(AttachmentTemplateId attachment_template_id) const;
  [[nodiscard]] std::optional<OverrideInspectionView> inspect_overrides(EntityRef target) const;

private:
  const CoreState& state_;
};

inline CoreView CoreState::view() const { return CoreView(*this); }

CoreState make_demo_state();

} // namespace wire::core


