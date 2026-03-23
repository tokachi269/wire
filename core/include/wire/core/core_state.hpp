#pragma once

#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "wire/core/core_authoritative_types.hpp"
#include "wire/core/core_runtime_types.hpp"
#include "wire/core/core_state_api_types.hpp"
#include "wire/core/debug_types.hpp"
#include "wire/core/detail_curve.hpp"
#include "wire/core/entities.hpp"
#include "wire/core/id.hpp"
#include "wire/core/object_store.hpp"
#include "wire/core/support_layout_types.hpp"
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

class CoreState {
public:
  CoreState();

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

CoreState make_demo_state();

} // namespace wire::core


