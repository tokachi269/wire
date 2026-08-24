#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "city/wire/core_state_api_types.hpp"
#include "city/wire/core_state_storage_types.hpp"

namespace city::wire {

class CoreView;
namespace state_internal {
struct OverrideResolutionService;
struct EndpointRefreshService;
struct TemplateMutationService;
}
namespace generation::backbone {
class pipeline;
}

struct PortLayoutYawOverride {
  ConnectionCategory category = ConnectionCategory::kLowVoltage;
  double yaw_deg = 0.0;
};

class CoreState {
public:
  CoreState();

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
                                 BundleTemplateId bundle_template_id = kInvalidBundleTemplateId,
                                 bool placement_explicit = false,
                                 double height_m = 0.0, double lateral_m = 0.0,
                                 double spacing_override_m = 0.0,
                                 std::uint64_t placement_key = 0);
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
  EditResult<ObjectId> ApplyPoleType(ObjectId pole_id, PoleTypeId pole_type_id);
  // Canonical path-generation API.
  EditResult<GenerateBundleFromPathResult> GenerateFromBackboneSpec(const BackboneSpec& spec);
  // Interprets a viewer-side pick payload and creates volatile pending support nodes when needed.
  EditResult<ResolveBranchPickResult> ResolveBranchPick(const PickResult& pick);
  EditResult<ResolveBranchPickResult> ResolveBranchPick(const PickResult& pick,
                                                        const ResolveBranchPickOptions& options);
  // Clears pending support-node drafts created by ResolveBranchPick for the current draw session.
  EditResult<bool> ClearPendingSupportNodes();
  [[nodiscard]] EditResult<DefaultBundlePlacementResult>
  ResolveDefaultBundlePlacement(BundleTemplateId bundle_template_id, PoleTypeId pole_type_id, int count) const;
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
  [[nodiscard]] BackboneResult SavedBackboneResult() const;
  [[nodiscard]] std::vector<BackboneEdge> SavedBackboneEdges() const;
  [[nodiscard]] std::vector<ObjectId> FindSavedBackboneRoute(ObjectId start_node_id, ObjectId end_node_id) const;
  EditResult<bool> DeriveGeneratedSpanOutputs(ObjectId span_id);
  EditResult<bool> UpdateGeometrySettings(const GeometrySettings& settings, bool mark_all_spans_dirty = true);
  EditResult<bool> UpdateLayoutSettings(const LayoutSettings& settings);
  EditResult<bool> UpdateVisualSettings(const VisualSettings& settings, bool mark_all_spans_dirty = true);
  EditResult<bool> UpdateVariationSettings(const VariationSettings& settings, bool mark_all_spans_dirty = true);
  EditResult<bool> UpdateContextProfile(const ContextProfile& profile, bool mark_all_spans_dirty = true);
  EditResult<bool> UpdateCableTemplate(const CableTemplate& cable_template);
  EditResult<bool> UpdateCableTemplate(const CableTemplate& cable_template,
                                       const std::vector<ObjectId>& preferred_visible_span_ids);
  EditResult<bool> UpdatePoleTypeDefinition(const PoleTypeDefinition& pole_type);
  EditResult<bool> UpdateBundleTemplate(const BundleTemplate& bundle_template);
  EditResult<bool> UpdateBackboneBundlePlacement(ObjectId bundle_id, bool placement_explicit,
                                                 double height_m, double lateral_m, double spacing_m);
  // Registers one adapter-built assembly before any template references it.
  // Existing ids are rejected; versioned adapter reload uses UpdateModelAssemblyTemplate.
  EditResult<bool> RegisterModelAssemblyTemplate(const ModelAssemblyTemplate& model_assembly_template);
  EditResult<bool> UpdateModelAssemblyTemplate(const ModelAssemblyTemplate& model_assembly_template);
  EditResult<bool> ApplyBundleRelatedPoleTypeToExistingPoles(BundleTemplateId bundle_template_id);
  EditResult<bool> UpdateAttachmentTemplate(const AttachmentTemplate& attachment_template,
                                            bool mark_dependent_spans_dirty = true);
  EditResult<bool> ResetAllSpanReferenceLengths(bool mark_all_spans_dirty = true);
  [[nodiscard]] EditResult<bool> SerializeAuthoritative(std::string* out) const;
  EditResult<bool> DeserializeAuthoritative(const std::string& text);
  [[nodiscard]] const CurveCacheEntry* find_curve_cache(ObjectId span_id) const;
  [[nodiscard]] const BoundsCacheEntry* find_bounds_cache(ObjectId span_id) const;
  [[nodiscard]] SpanLayoutView span_layout(ObjectId span_id) const;
  [[nodiscard]] SpanLayoutState span_layout_state(ObjectId span_id) const;
  [[nodiscard]] SpanLayoutRulesView span_layout_rules(ObjectId span_id) const;
  [[nodiscard]] const SpanVisualCacheEntry* find_span_visual_cache(ObjectId span_id) const;
  [[nodiscard]] const SpanRenderCacheEntry* find_span_render_cache(ObjectId span_id) const;
  [[nodiscard]] const VisualCurvePartCache& visual_curve_parts() const;
  [[nodiscard]] const VisualModelInstanceCache& visual_model_instances() const;
  [[nodiscard]] const AttachmentTemplate* find_attachment_template(AttachmentTemplateId attachment_template_id) const;
  [[nodiscard]] ValidationResult ValidateFast() const;
  [[nodiscard]] double effective_pole_layout_yaw_deg(const Pole& pole) const;
  [[nodiscard]] double effective_port_layout_yaw_deg(const Pole& pole, ObjectId port_id,
                                                     ConnectionCategory category,
                                                     const PortLayoutYawOverride* row_layout_yaw_override = nullptr) const;

  [[nodiscard]] ObjectId next_id() const { return identity_.id_generator.peek(); }
  void clear_path_direction_debug_records() { debug_.path_direction_debug_records.clear(); }
  void clear_port_resolution_debug_records() { debug_.port_resolution_debug_records.clear(); }

  [[nodiscard]] CoreView view() const;

private:
#if defined(WIRE_INTERNAL) || defined(WIRE_TESTING)
  friend struct CoreStateTestHook;
#endif
  friend class CoreView;
  friend class generation::backbone::pipeline;
  friend struct state_internal::OverrideResolutionService;
  friend struct state_internal::EndpointRefreshService;
  friend struct state_internal::TemplateMutationService;
  void remove_span_from_indexes(const Span& span);
  void add_span_to_index(const Span& span);
  void initialize_span_runtime_state(ObjectId span_id);
  void touch_span(ObjectId span_id, bool bump_data_version);
  void touch_connected_spans_from_port(ObjectId port_id, ChangeSet* change_set);
  void touch_connected_spans_from_anchor(ObjectId anchor_id, ChangeSet* change_set);
  EditResult<bool> derive_generated_span_shape_outputs(ObjectId span_id);
  EditResult<bool> derive_generated_span_draw_outputs(ObjectId span_id);
  [[nodiscard]] EditResult<UpdatePlan> make_update_plan(UpdateRequest request) const;
  EditResult<bool> execute_update_plan(const UpdatePlan& plan);
  [[nodiscard]] std::vector<ObjectId> collect_topology_related_spans_for_ports(const std::vector<ObjectId>& port_ids,
                                                                                ObjectId exclude_span_id) const;
  void touch_topology_related_spans_for_ports(const std::vector<ObjectId>& port_ids, ObjectId exclude_span_id,
                                              ChangeSet* change_set);
  void cache_span_layout(SpanLayoutEntry layout);
  void cache_span_curve(ObjectId span_id, DetailCurve detail);
  void cache_span_bounds(ObjectId span_id, BoundsCacheEntry bounds);
  void cache_span_visual(ObjectId span_id, SpanVisualCacheEntry visual);
  void cache_span_render(ObjectId span_id, SpanRenderCacheEntry render);
  void cache_visual_curve_parts(VisualCurvePartCache visual_curves);
  void cache_visual_model_instances(VisualModelInstanceCache model_instances);
  void cache_support_group(SupportGroupDecision decision, LoweredSupportGroupPlacement placement);
  ObjectId save_backbone_node(ObjectId pole_id, const Vec3d& position,
                              SupportKind support_kind = SupportKind::kPole,
                              ObjectId source_edge_node_a = kInvalidObjectId,
                              ObjectId source_edge_node_b = kInvalidObjectId,
                              double source_edge_t = 0.0,
                              std::vector<SupportNodeBundleMode> bundle_modes = {});
  EditResult<bool> set_span_endpoint_nodes(ObjectId span_id, ObjectId node_a_id, ObjectId node_b_id);
  EditResult<bool> bind_backbone_node_bundle_modes(ObjectId node_id,
                                                   const std::vector<SupportNodeBundleMode>& bundle_modes);
  EditResult<bool> bind_backbone_node_path_point_index(ObjectId node_id, int path_point_index);
  SavedBackboneEdgeRef save_backbone_edge(ObjectId node_a, ObjectId node_b, std::size_t route, std::size_t order,
                                          const Vec3d& dir, double lateral_offset_m);
  ObjectId bind_backbone_bundle(ObjectId edge_id, ObjectId bundle_id, bool edge_forward, std::size_t route,
                                std::size_t order, const Vec3d& dir);
  EditResult<bool> bind_backbone_span(ObjectId edge_bundle_id, std::size_t lane_index, ObjectId span_id);
  EditResult<bool> bind_backbone_port(ObjectId edge_bundle_id, const SavedBackboneRowKey& row_key,
                                      std::size_t lane_index, BundleTemplateId bundle_template_id, PortKind port_kind,
                                      PortLayer port_layer, int placement_band_id, int support_level,
                                      int support_group_id,
                                      double layout_yaw_deg,
                                      ObjectId port_id);
  EditResult<bool> update_backbone_port_binding_frame_exact(
      ObjectId edge_bundle_id, const SavedBackboneRowKey& row_key,
      std::size_t lane_index, double layout_yaw_deg,
      int support_level, int support_group_id, ObjectId port_id,
      const Vec3d& world_position);
  EditResult<bool> reverse_backbone_row_lanes(
      ObjectId edge_bundle_id, const SavedBackboneRowKey& row_key);
  EditResult<bool> bind_backbone_row_continuity(ObjectId node_id,
                                                ObjectId edge_bundle_a,
                                                std::size_t lane_a,
                                                ObjectId edge_bundle_b,
                                                std::size_t lane_b);
  void remove_backbone_row_continuities_for_lanes(const std::vector<ObjectId>& edge_bundle_ids,
                                                  std::size_t first_retired_lane);
  enum class BackboneRegenerateCause : std::uint8_t {
    kBundleCount,
    kBundleTopology,
    kCableDecision,
    kPoleType,
    kSpanOverride,
    kLayoutSettings,
  };
  EditResult<bool> regenerate_backbone_edge_bundles(BundleTemplateId bundle_template_id,
                                                    const BundleTemplate& previous_template,
                                                    const BundleTemplate& next_template,
                                                    ChangeSet* change_set,
                                                    const CableTemplate* cable_template_override = nullptr,
                                                    const std::vector<ObjectId>* scoped_edge_bundle_ids = nullptr,
                                                    const PoleTypeDefinition* pole_type_override = nullptr,
                                                    BackboneRegenerateCause cause = BackboneRegenerateCause::kBundleCount);
  EditResult<bool> regenerate_backbone_span_override(ObjectId span_id, ChangeSet* change_set);
  EditResult<bool> rebuild_loaded_outputs();
  [[nodiscard]] bool authoritative_equals(const CoreState& other) const;
  void cache_span_rules(const SpanLayoutRules& rules);
  void remove_span_from_caches(ObjectId span_id);
  EditResult<bool> merge_cached_span_outputs_from(const CoreState& source,
                                                   ObjectId span_id);
  void merge_cached_support_groups_from(const CoreState& source);
  [[nodiscard]] double effective_pole_yaw_deg(const Pole& pole) const;
  [[nodiscard]] Vec3d to_local_on_pole(const Pole& pole, const Vec3d& world) const;
  [[nodiscard]] SlotSide preferred_side_from_geometry(const Pole& pole, const Pole* peer, double eps) const;
  [[nodiscard]] static double polyline_length(const std::vector<Vec3d>& polyline);
  [[nodiscard]] static PortLayer category_to_port_layer(ConnectionCategory category);
  [[nodiscard]] static SpanLayer category_to_span_layer(ConnectionCategory category);
  [[nodiscard]] static BundleKind category_to_bundle_kind(ConnectionCategory category);
  [[nodiscard]] static PortKind category_to_port_kind(ConnectionCategory category);
  EditResult<bool> ensure_default_endpoint_attachments_for_span(ObjectId span_id);
  EditResult<bool> update_pole_type_and_refresh_instances(const PoleTypeDefinition& pole_type);
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
  [[nodiscard]] const BundleTemplate* find_bundle_template(BundleTemplateId bundle_template_id) const;
  [[nodiscard]] std::vector<PortPlacementBand> sorted_port_bands(const PoleTypeDefinition& pole_type,
                                                                 ConnectionCategory category) const;
  [[nodiscard]] bool is_port_band_used(ObjectId pole_id, const PortPlacementBand& band) const;

  EditResult<ObjectId> ensure_pole_connection_port(const PortResolutionRequest& request);
  [[nodiscard]] static std::uint8_t deterministic_tiebreak_0_255(ObjectId pole_id, int tiebreak_key,
                                                                 ConnectionCategory category, ConnectionContext context,
                                                                 ObjectId peer_pole_id, ObjectId reference_span_id,
                                                                 std::uint32_t branch_index);
  [[nodiscard]] static bool is_valid_slot_side(SlotSide side);
  [[nodiscard]] static bool is_valid_slot_role(SlotRole role);
  [[nodiscard]] double compute_side_scale(PoleContextKind context, double corner_angle_deg) const;
  static void add_unique_id(std::vector<ObjectId>& ids, ObjectId id);
  static void index_add(std::unordered_map<ObjectId, std::vector<ObjectId>>& map, ObjectId key, ObjectId value);
  static void index_remove(std::unordered_map<ObjectId, std::vector<ObjectId>>& map, ObjectId key, ObjectId value);
  static void apply_pole_placement_mode(Pole& pole, PlacementMode mode);
  static void apply_port_position_mode(Port& port, PortPositionMode mode, PortPlacementSourceKind source_hint);
  [[nodiscard]] double pole_radius_at_height_m(const Pole& pole, double local_z_m) const;
  [[nodiscard]] Vec3d apply_pole_clearance_to_local(const Pole& pole, const Vec3d& local, SlotSide side) const;
  EditResult<bool> apply_pole_tilt_from_pull(ObjectId pole_id, double max_tilt_deg, const Vec3d& pull_world_dir,
                                             std::size_t incident_span_count, ChangeSet* change_set);
  void finalize_pole_transform_update(ObjectId pole_id, const Pole& old_pole, ChangeSet* change_set);
  EditResult<bool> refresh_backbone_rows_for_incident_edges(ObjectId pole_id,
                                                             ChangeSet* change_set);
  std::string next_display_id(std::string_view prefix);
  void refresh_owned_endpoints_from_pole(ObjectId pole_id, ChangeSet* change_set, const Pole* previous_pole = nullptr,
                                         const PortLayoutYawOverride* previous_row_layout_yaw_override = nullptr);
  [[nodiscard]] static bool has_zero_length(const Port& a, const Port& b);
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

  CoreStateIdentityStorage identity_{};
  CoreStateAuthoritativeStorage authoritative_{};
  CoreStateRuntimeStorage runtime_{};
  CoreStateSessionStorage session_{};
  CoreStateDebugStorage debug_{};
};

} // namespace city::wire
