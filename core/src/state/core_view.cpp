#include "wire/core/core_state.hpp"

#include "wire/core/core_view.hpp"

namespace wire::core {

CoreView::CoreView(const CoreState& state) : state_(state) {}

const EditState& CoreView::edit_state() const { return state_.edit_state_; }
const ObjectStore<Pole>& CoreView::poles() const { return state_.edit_state_.poles; }
const ObjectStore<Port>& CoreView::ports() const { return state_.edit_state_.ports; }
const ObjectStore<Anchor>& CoreView::anchors() const { return state_.edit_state_.anchors; }
const ObjectStore<Bundle>& CoreView::bundles() const { return state_.edit_state_.bundles; }
const ObjectStore<Span>& CoreView::spans() const { return state_.edit_state_.spans; }
const ObjectStore<Attachment>& CoreView::attachments() const { return state_.edit_state_.attachments; }

const ConnectionIndex& CoreView::connection_index() const { return state_.connection_index_; }
const RelationIndex& CoreView::relation_index() const { return state_.relation_index_; }
const DirtyQueue& CoreView::dirty_queue() const { return state_.dirty_queue_; }
const RecalcStats& CoreView::last_recalc_stats() const { return state_.last_recalc_stats_; }
const GeometrySettings& CoreView::geometry_settings() const { return state_.cache_state_.geometry_settings; }
const VisualSettings& CoreView::visual_settings() const { return state_.cache_state_.visual_settings; }
const VariationSettings& CoreView::variation_settings() const { return state_.cache_state_.variation_settings; }
const LayoutSettings& CoreView::layout_settings() const { return state_.layout_settings_; }
const PathDirectionEvaluationDebug& CoreView::last_path_direction_debug() const { return state_.last_path_direction_debug_; }
const std::vector<PathDirectionEvaluationDebug>& CoreView::path_direction_debug_records() const {
  return state_.path_direction_debug_records_;
}
const std::unordered_map<ObjectId, PoleOrientationDebugRecord>& CoreView::pole_orientation_debug_records() const {
  return state_.pole_orientation_debug_records_;
}
const std::vector<BackboneEdgeOrientation>& CoreView::last_generation_edge_orientations() const {
  return state_.last_generation_edge_orientations_;
}
const CacheState& CoreView::cache_state() const { return state_.cache_state_; }
const std::unordered_map<PoleTypeId, PoleTypeDefinition>& CoreView::pole_types() const { return state_.pole_types_; }
int CoreView::count_port_bands(PoleTypeId pole_type_id, ConnectionCategory category) const {
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
const std::unordered_map<CableTemplateId, CableTemplate>& CoreView::cable_templates() const {
  return state_.cable_templates_;
}
const std::unordered_map<BundleKind, BundleTemplate>& CoreView::bundle_templates() const {
  return state_.bundle_templates_;
}
const std::unordered_map<AttachmentTemplateId, AttachmentTemplate>& CoreView::attachment_templates() const {
  return state_.attachment_templates_;
}
const TemplateDependencyState& CoreView::template_dependency_state() const { return state_.template_dependency_state_; }
const std::vector<PortResolutionDebugRecord>& CoreView::port_resolution_debug_records() const {
  return state_.port_resolution_debug_records_;
}
const std::unordered_map<ObjectId, SpanRuntimeState>& CoreView::span_runtime_states() const {
  return state_.span_runtime_states_;
}
const SpanRuntimeState* CoreView::find_span_runtime_state(ObjectId span_id) const {
  return state_.find_span_runtime_state(span_id);
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
