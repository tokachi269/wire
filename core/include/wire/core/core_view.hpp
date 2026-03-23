#pragma once

#include <cstdint>
#include <optional>
#include <unordered_map>
#include <vector>

#include "wire/core/core_authoritative_types.hpp"
#include "wire/core/core_runtime_types.hpp"
#include "wire/core/inspection.hpp"
#include "wire/core/object_store.hpp"
#include "wire/core/support_layout_types.hpp"

namespace wire::core {

class CoreState;

class CoreView {
public:
  explicit CoreView(const CoreState& state);

  [[nodiscard]] const EditState& edit_state() const;
  [[nodiscard]] const ObjectStore<Pole>& poles() const;
  [[nodiscard]] const ObjectStore<Port>& ports() const;
  [[nodiscard]] const ObjectStore<Anchor>& anchors() const;
  [[nodiscard]] const ObjectStore<Bundle>& bundles() const;
  [[nodiscard]] const ObjectStore<Span>& spans() const;
  [[nodiscard]] const ObjectStore<Attachment>& attachments() const;

  [[nodiscard]] const ConnectionIndex& connection_index() const;
  [[nodiscard]] const RelationIndex& relation_index() const;
  [[nodiscard]] const DirtyQueue& dirty_queue() const;
  [[nodiscard]] const RecalcStats& last_recalc_stats() const;
  [[nodiscard]] const GeometrySettings& geometry_settings() const;
  [[nodiscard]] const VisualSettings& visual_settings() const;
  [[nodiscard]] const VariationSettings& variation_settings() const;
  [[nodiscard]] const LayoutSettings& layout_settings() const;
  [[nodiscard]] const PathDirectionEvaluationDebug& last_path_direction_debug() const;
  [[nodiscard]] const std::vector<PathDirectionEvaluationDebug>& path_direction_debug_records() const;
  [[nodiscard]] const std::unordered_map<ObjectId, PoleOrientationDebugRecord>& pole_orientation_debug_records() const;
  [[nodiscard]] const std::vector<BackboneEdgeOrientation>& last_generation_edge_orientations() const;
  [[nodiscard]] const CacheState& cache_state() const;
  [[nodiscard]] const std::unordered_map<PoleTypeId, PoleTypeDefinition>& pole_types() const;
  [[nodiscard]] int count_port_bands(PoleTypeId pole_type_id, ConnectionCategory category) const;
  [[nodiscard]] const std::unordered_map<CableTemplateId, CableTemplate>& cable_templates() const;
  [[nodiscard]] const std::unordered_map<BundleKind, BundleTemplate>& bundle_templates() const;
  [[nodiscard]] const std::unordered_map<AttachmentTemplateId, AttachmentTemplate>& attachment_templates() const;
  [[nodiscard]] const TemplateDependencyState& template_dependency_state() const;
  [[nodiscard]] const std::vector<PortResolutionDebugRecord>& port_resolution_debug_records() const;
  [[nodiscard]] const std::vector<SegmentLaneAssignment>& last_lane_assignments() const;
  [[nodiscard]] const std::unordered_map<ObjectId, SpanRuntimeState>& span_runtime_states() const;
  [[nodiscard]] const SpanRuntimeState* find_span_runtime_state(ObjectId span_id) const;
  [[nodiscard]] const SpanSupportLayoutEntry* find_span_support_layout(ObjectId span_id) const;
  [[nodiscard]] const SpanVisualCacheEntry* find_span_visual_cache(ObjectId span_id) const;
  [[nodiscard]] const SpanRenderCacheEntry* find_span_render_cache(ObjectId span_id) const;
  [[nodiscard]] const AttachmentTemplate* find_attachment_template(AttachmentTemplateId attachment_template_id) const;
  [[nodiscard]] double pole_radius_at_height_m(const Pole& pole, double local_z_m) const;
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

} // namespace wire::core
