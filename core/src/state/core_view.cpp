#include "wire/core/core_state.hpp"

#include "wire/core/core_view.hpp"

namespace wire::core {

CoreView::CoreView(const CoreState& state) : state_(state) {}

const EditState& CoreView::edit_state() const { return state_.authoritative_.edit_state; }
const ObjectStore<Pole>& CoreView::poles() const { return state_.authoritative_.edit_state.poles; }
const ObjectStore<Port>& CoreView::ports() const { return state_.authoritative_.edit_state.ports; }
const ObjectStore<Anchor>& CoreView::anchors() const { return state_.authoritative_.edit_state.anchors; }
const ObjectStore<Bundle>& CoreView::bundles() const { return state_.authoritative_.edit_state.bundles; }
const ObjectStore<Span>& CoreView::spans() const { return state_.authoritative_.edit_state.spans; }
const ObjectStore<Attachment>& CoreView::attachments() const { return state_.authoritative_.edit_state.attachments; }

const ConnectionIndex& CoreView::connection_index() const { return state_.runtime_.connection_index; }
const RelationIndex& CoreView::relation_index() const { return state_.runtime_.relation_index; }
const DirtyQueue& CoreView::dirty_queue() const { return state_.runtime_.dirty_queue; }
const RecalcStats& CoreView::last_recalc_stats() const { return state_.runtime_.last_recalc_stats; }
const GeometrySettings& CoreView::geometry_settings() const { return state_.runtime_.cache_state.geometry_settings; }
const VisualSettings& CoreView::visual_settings() const { return state_.runtime_.cache_state.visual_settings; }
const VariationSettings& CoreView::variation_settings() const { return state_.runtime_.cache_state.variation_settings; }
const LayoutSettings& CoreView::layout_settings() const { return state_.authoritative_.layout_settings; }
const PathDirectionEvaluationDebug& CoreView::last_path_direction_debug() const { return state_.debug_.last_path_direction_debug; }
const std::vector<PathDirectionEvaluationDebug>& CoreView::path_direction_debug_records() const {
  return state_.debug_.path_direction_debug_records;
}
const std::unordered_map<ObjectId, PoleOrientationDebugRecord>& CoreView::pole_orientation_debug_records() const {
  return state_.debug_.pole_orientation_debug_records;
}
const std::vector<BackboneEdgeOrientation>& CoreView::last_generation_edge_orientations() const {
  return state_.debug_.last_generation_edge_orientations;
}
const CacheState& CoreView::cache_state() const { return state_.runtime_.cache_state; }
const std::unordered_map<PoleTypeId, PoleTypeDefinition>& CoreView::pole_types() const { return state_.authoritative_.pole_types; }
int CoreView::count_port_bands(PoleTypeId pole_type_id, ConnectionCategory category) const {
  const auto it = state_.authoritative_.pole_types.find(pole_type_id);
  if (it == state_.authoritative_.pole_types.end()) {
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
const std::unordered_map<CableTemplateId, CableTemplate>& CoreView::cable_templates() const {
  return state_.authoritative_.cable_templates;
}
const std::unordered_map<BundleKind, BundleTemplate>& CoreView::bundle_templates() const {
  return state_.authoritative_.bundle_templates;
}
const std::unordered_map<AttachmentTemplateId, AttachmentTemplate>& CoreView::attachment_templates() const {
  return state_.authoritative_.attachment_templates;
}
const TemplateDependencyState& CoreView::template_dependency_state() const { return state_.authoritative_.template_dependency_state; }
const std::vector<PortResolutionDebugRecord>& CoreView::port_resolution_debug_records() const {
  return state_.debug_.port_resolution_debug_records;
}
const std::unordered_map<ObjectId, SpanRuntimeState>& CoreView::span_runtime_states() const {
  return state_.runtime_.span_runtime_states;
}
BackboneResult CoreView::build_backbone_result() const { return state_.BuildBackboneResult(); }
const SpanRuntimeState* CoreView::find_span_runtime_state(ObjectId span_id) const {
  return state_.find_span_runtime_state(span_id);
}
const CurveCacheEntry* CoreView::find_curve_cache(ObjectId span_id) const {
  return state_.find_curve_cache(span_id);
}
const BoundsCacheEntry* CoreView::find_bounds_cache(ObjectId span_id) const {
  return state_.find_bounds_cache(span_id);
}
const SpanSupportLayoutEntry* CoreView::find_span_support_layout(ObjectId span_id) const {
  return state_.find_span_support_layout(span_id);
}
const SpanVisualCacheEntry* CoreView::find_span_visual_cache(ObjectId span_id) const {
  return state_.find_span_visual_cache(span_id);
}
const SpanRenderCacheEntry* CoreView::find_span_render_cache(ObjectId span_id) const {
  return state_.find_span_render_cache(span_id);
}
const AttachmentTemplate* CoreView::find_attachment_template(AttachmentTemplateId attachment_template_id) const {
  return state_.find_attachment_template(attachment_template_id);
}
double CoreView::pole_radius_at_height_m(const Pole& pole, double local_z_m) const {
  return state_.pole_radius_at_height_m(pole, local_z_m);
}

CoreView CoreState::view() const { return CoreView(*this); }

} // namespace wire::core

