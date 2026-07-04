#include "wire/core/core_state.hpp"

#include "wire/core/core_view.hpp"
#include "wire/core/coord_utils.hpp"

#include "../generation/support_policy.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace wire::core {

CoreView::CoreView(const CoreState& state) : state_(state) {}

const EditState& CoreView::edit_state() const { return state_.authoritative_.edit_state; }
const ObjectStore<Pole>& CoreView::poles() const { return state_.authoritative_.edit_state.poles; }
const ObjectStore<Port>& CoreView::ports() const { return state_.authoritative_.edit_state.ports; }
const ObjectStore<Anchor>& CoreView::anchors() const { return state_.authoritative_.edit_state.anchors; }
const ObjectStore<Bundle>& CoreView::bundles() const { return state_.authoritative_.edit_state.bundles; }
const ObjectStore<Span>& CoreView::spans() const { return state_.authoritative_.edit_state.spans; }
const ObjectStore<Attachment>& CoreView::attachments() const { return state_.authoritative_.edit_state.attachments; }
const SavedBackboneGraph& CoreView::backbone() const { return state_.authoritative_.backbone; }

const ConnectionIndex& CoreView::connection_index() const { return state_.runtime_.connection_index; }
const RelationIndex& CoreView::relation_index() const { return state_.runtime_.relation_index; }
const BackboneIndex& CoreView::backbone_index() const { return state_.runtime_.backbone_index; }
const SavedBackboneNode* CoreView::backbone_node(ObjectId node_id) const {
  const auto it = std::find_if(state_.authoritative_.backbone.nodes.begin(), state_.authoritative_.backbone.nodes.end(),
                               [&](const SavedBackboneNode& node) { return node.node_id == node_id; });
  return it == state_.authoritative_.backbone.nodes.end() ? nullptr : &*it;
}
const SavedBackboneEdge* CoreView::backbone_edge(ObjectId edge_id) const {
  const auto it = std::find_if(state_.authoritative_.backbone.edges.begin(), state_.authoritative_.backbone.edges.end(),
                               [&](const SavedBackboneEdge& edge) { return edge.edge_id == edge_id; });
  return it == state_.authoritative_.backbone.edges.end() ? nullptr : &*it;
}
const SavedBackboneEdgeBundle* CoreView::backbone_edge_bundle(ObjectId edge_bundle_id) const {
  const auto it = std::find_if(state_.authoritative_.backbone.edge_bundles.begin(),
                               state_.authoritative_.backbone.edge_bundles.end(),
                               [&](const SavedBackboneEdgeBundle& item) {
                                 return item.edge_bundle_id == edge_bundle_id;
                               });
  return it == state_.authoritative_.backbone.edge_bundles.end() ? nullptr : &*it;
}
const SavedBackboneNode* CoreView::backbone_node_for_pole(ObjectId pole_id) const {
  const auto it = state_.runtime_.backbone_index.pole_node.find(pole_id);
  return it == state_.runtime_.backbone_index.pole_node.end() ? nullptr : backbone_node(it->second);
}
const SupportNode* CoreView::pending_support_node(ObjectId node_id) const {
  const auto it = std::find_if(state_.debug_.pending_support_nodes.begin(),
                               state_.debug_.pending_support_nodes.end(),
                               [&](const SupportNode& node) { return node.node_id == node_id; });
  return it == state_.debug_.pending_support_nodes.end() ? nullptr : &*it;
}
const SavedBackbonePortBinding* CoreView::backbone_port_binding_for_port(ObjectId port_id) const {
  const auto it = state_.runtime_.backbone_index.port_bindings_by_port.find(port_id);
  if (it == state_.runtime_.backbone_index.port_bindings_by_port.end() || it->second.empty() ||
      it->second.front() >= state_.authoritative_.backbone.port_bindings.size()) {
    return nullptr;
  }
  return &state_.authoritative_.backbone.port_bindings[it->second.front()];
}
std::vector<const SavedBackbonePortBinding*> CoreView::backbone_port_bindings_for_port(ObjectId port_id) const {
  std::vector<const SavedBackbonePortBinding*> out{};
  const auto it = state_.runtime_.backbone_index.port_bindings_by_port.find(port_id);
  if (it == state_.runtime_.backbone_index.port_bindings_by_port.end()) {
    return out;
  }
  out.reserve(it->second.size());
  for (std::size_t index : it->second) {
    if (index < state_.authoritative_.backbone.port_bindings.size()) {
      out.push_back(&state_.authoritative_.backbone.port_bindings[index]);
    }
  }
  return out;
}
std::vector<const SavedBackbonePortBinding*> CoreView::backbone_port_bindings_for_edge_bundle(
    ObjectId edge_bundle_id) const {
  std::vector<const SavedBackbonePortBinding*> out{};
  const auto it = state_.runtime_.backbone_index.edge_bundle_ports.find(edge_bundle_id);
  if (it == state_.runtime_.backbone_index.edge_bundle_ports.end()) {
    return out;
  }
  out.reserve(it->second.size());
  for (std::size_t index : it->second) {
    if (index < state_.authoritative_.backbone.port_bindings.size()) {
      out.push_back(&state_.authoritative_.backbone.port_bindings[index]);
    }
  }
  return out;
}
std::vector<const SavedBackbonePortBinding*> CoreView::backbone_port_bindings_for_row(
    const SavedBackboneRowKey& row_key, std::size_t lane_index) const {
  std::vector<const SavedBackbonePortBinding*> out{};
  for (const SavedBackbonePortBinding& binding : state_.authoritative_.backbone.port_bindings) {
    if (binding.row_key == row_key && binding.lane_index == lane_index) {
      out.push_back(&binding);
    }
  }
  return out;
}
std::optional<Vec3d> CoreView::backbone_attachment_world(
    ObjectId edge_id, ObjectId from_node_id, BundleKind bundle_template_id, std::size_t lane_index, double t) const {
  const SavedBackboneEdge* edge = backbone_edge(edge_id);
  if (edge == nullptr || (from_node_id != edge->node_a && from_node_id != edge->node_b)) {
    return std::nullopt;
  }
  const auto edge_bundles_it = state_.runtime_.backbone_index.edge_bundles.find(edge_id);
  if (edge_bundles_it == state_.runtime_.backbone_index.edge_bundles.end()) {
    return std::nullopt;
  }

  const SavedBackboneEdgeBundle* matched = nullptr;
  for (ObjectId edge_bundle_id : edge_bundles_it->second) {
    const SavedBackboneEdgeBundle* candidate = backbone_edge_bundle(edge_bundle_id);
    const Bundle* bundle =
        candidate == nullptr ? nullptr : bundles().find(candidate->bundle_id);
    if (bundle == nullptr || bundle->bundle_template_id != bundle_template_id) {
      continue;
    }
    if (matched != nullptr) {
      return std::nullopt;
    }
    matched = candidate;
  }
  if (matched == nullptr) {
    return std::nullopt;
  }

  const SavedBackbonePortBinding* binding_a = nullptr;
  const SavedBackbonePortBinding* binding_b = nullptr;
  for (const SavedBackbonePortBinding* binding :
       backbone_port_bindings_for_edge_bundle(matched->edge_bundle_id)) {
    if (binding == nullptr || binding->lane_index != lane_index) {
      continue;
    }
    const bool at_a = binding->row_key.node_id == edge->node_a;
    const bool at_b = binding->row_key.node_id == edge->node_b;
    if ((!at_a && !at_b) || (at_a && binding_a != nullptr) || (at_b && binding_b != nullptr)) {
      return std::nullopt;
    }
    (at_a ? binding_a : binding_b) = binding;
  }
  if (binding_a == nullptr || binding_b == nullptr) {
    return std::nullopt;
  }
  const auto span_bindings_it =
      state_.runtime_.backbone_index.edge_bundle_span_bindings.find(matched->edge_bundle_id);
  if (span_bindings_it == state_.runtime_.backbone_index.edge_bundle_span_bindings.end()) {
    return std::nullopt;
  }
  const SavedBackboneSpanBinding* span_binding = nullptr;
  for (std::size_t index : span_bindings_it->second) {
    if (index >= state_.authoritative_.backbone.span_bindings.size()) {
      return std::nullopt;
    }
    const SavedBackboneSpanBinding& candidate = state_.authoritative_.backbone.span_bindings[index];
    if (candidate.lane_index != lane_index) {
      continue;
    }
    if (span_binding != nullptr) {
      return std::nullopt;
    }
    span_binding = &candidate;
  }
  if (span_binding == nullptr) {
    return std::nullopt;
  }
  const CurveCacheEntry* curve = find_curve_cache(span_binding->span_id);
  if (curve == nullptr || curve->detail.sample_points.size() < 2) {
    return std::nullopt;
  }

  const double u = from_node_id == edge->node_a ? std::clamp(t, 0.0, 1.0) : 1.0 - std::clamp(t, 0.0, 1.0);
  return curve->detail.EvaluatePosition(u);
}
const GeometrySettings& CoreView::geometry_settings() const { return state_.runtime_.cache_state.geometry_settings; }
const VisualSettings& CoreView::visual_settings() const { return state_.runtime_.cache_state.visual_settings; }
const VariationSettings& CoreView::variation_settings() const { return state_.runtime_.cache_state.variation_settings; }
const ExperimentalCablePopulationConfig& CoreView::experimental_cable_population_config() const {
  return state_.runtime_.cache_state.experimental_cable_population;
}
const ContextProfile& CoreView::context_profile() const { return state_.authoritative_.context_profile; }
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
const UpdateTiming& CoreView::last_update_timing() const { return state_.debug_.last_update_timing; }
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
double CoreView::port_category_base_z_for_pole(const Pole& pole, ConnectionCategory category) const {
  const auto it = state_.authoritative_.pole_types.find(pole.pole_type_id);
  if (it == state_.authoritative_.pole_types.end()) {
    return std::max(0.5, pole.height_m * 0.8);
  }

  const int target_layer = generation::detail::TemplateLayerForCategory(category);
  double best_z = -std::numeric_limits<double>::infinity();
  auto accumulate = [&](const auto& predicate) {
    for (const PortPlacementBand& band : it->second.port_bands) {
      if (!band.enabled || !predicate(band)) {
        continue;
      }
      best_z = std::max(best_z, band.height_max_m);
    }
    return std::isfinite(best_z);
  };

  if (accumulate([&](const PortPlacementBand& band) { return band.layer == target_layer && band.category == category; }) ||
      accumulate([&](const PortPlacementBand& band) { return band.category == category; }) ||
      accumulate([&](const PortPlacementBand& band) { return band.layer == target_layer; })) {
    return best_z;
  }
  return std::max(0.5, pole.height_m * 0.8);
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
const std::vector<PortResolutionDebugRecord>& CoreView::port_resolution_debug_records() const {
  return state_.debug_.port_resolution_debug_records;
}
const std::unordered_map<ObjectId, SpanRuntimeState>& CoreView::span_runtime_states() const {
  return state_.runtime_.span_runtime_states;
}
BackboneResult CoreView::saved_backbone_result() const { return state_.SavedBackboneResult(); }
const SpanRuntimeState* CoreView::find_span_runtime_state(ObjectId span_id) const {
  return state_.find_span_runtime_state(span_id);
}
const CurveCacheEntry* CoreView::find_curve_cache(ObjectId span_id) const {
  return state_.find_curve_cache(span_id);
}
const BoundsCacheEntry* CoreView::find_bounds_cache(ObjectId span_id) const {
  return state_.find_bounds_cache(span_id);
}
SpanLayoutView CoreView::span_layout(ObjectId span_id) const {
  return state_.span_layout(span_id);
}
SpanLayoutState CoreView::span_layout_state(ObjectId span_id) const {
  return state_.span_layout_state(span_id);
}
SpanLayoutRulesView CoreView::span_layout_rules(ObjectId span_id) const {
  return state_.span_layout_rules(span_id);
}
const SpanVisualCacheEntry* CoreView::find_span_visual_cache(ObjectId span_id) const {
  return state_.find_span_visual_cache(span_id);
}
const SpanRenderCacheEntry* CoreView::find_span_render_cache(ObjectId span_id) const {
  return state_.find_span_render_cache(span_id);
}
const VisualCurvePartCache& CoreView::visual_curve_parts() const {
  return state_.visual_curve_parts();
}

namespace {

const SavedBackboneNode* find_saved_node(const SavedBackboneGraph& graph, ObjectId node_id) {
  const auto it = std::find_if(graph.nodes.begin(), graph.nodes.end(),
                               [&](const SavedBackboneNode& node) { return node.node_id == node_id; });
  return it == graph.nodes.end() ? nullptr : &*it;
}

const SavedBackboneEdge* find_saved_edge(const SavedBackboneGraph& graph, ObjectId edge_id) {
  const auto it = std::find_if(graph.edges.begin(), graph.edges.end(),
                               [&](const SavedBackboneEdge& edge) { return edge.edge_id == edge_id; });
  return it == graph.edges.end() ? nullptr : &*it;
}

const SavedBackboneEdgeBundle* find_saved_edge_bundle(const SavedBackboneGraph& graph, ObjectId edge_bundle_id) {
  const auto it = std::find_if(graph.edge_bundles.begin(), graph.edge_bundles.end(),
                               [&](const SavedBackboneEdgeBundle& item) {
                                 return item.edge_bundle_id == edge_bundle_id;
                               });
  return it == graph.edge_bundles.end() ? nullptr : &*it;
}

void add_id(std::vector<ObjectId>* ids, ObjectId id) {
  if (ids == nullptr || id == kInvalidObjectId) {
    return;
  }
  if (std::find(ids->begin(), ids->end(), id) == ids->end()) {
    ids->push_back(id);
  }
}

void add_frontier_node(const SavedBackboneGraph& graph, ObjectId node_id, BackboneFrontier* out) {
  if (node_id == kInvalidObjectId || out == nullptr) {
    return;
  }
  add_id(&out->node_ids, node_id);
  if (const SavedBackboneNode* node = find_saved_node(graph, node_id);
      node != nullptr && node->pole_id != kInvalidObjectId) {
    add_id(&out->pole_ids, node->pole_id);
  }
}

void add_frontier_edge(const SavedBackboneGraph& graph, const BackboneIndex& index, ObjectId edge_id,
                       BackboneFrontier* out) {
  if (edge_id == kInvalidObjectId || out == nullptr) {
    return;
  }
  add_id(&out->edge_ids, edge_id);
  if (const SavedBackboneEdge* edge = find_saved_edge(graph, edge_id)) {
    add_frontier_node(graph, edge->node_a, out);
    add_frontier_node(graph, edge->node_b, out);
  }
  if (const auto bundles = index.edge_bundles.find(edge_id); bundles != index.edge_bundles.end()) {
    for (ObjectId edge_bundle_id : bundles->second) {
      add_id(&out->edge_bundle_ids, edge_bundle_id);
      const SavedBackboneEdgeBundle* edge_bundle = find_saved_edge_bundle(graph, edge_bundle_id);
      if (edge_bundle != nullptr) {
        add_id(&out->bundle_ids, edge_bundle->bundle_id);
      }
      if (const auto spans = index.edge_bundle_spans.find(edge_bundle_id); spans != index.edge_bundle_spans.end()) {
        for (ObjectId span_id : spans->second) {
          add_id(&out->span_ids, span_id);
        }
      }
    }
  }
}

} // namespace

BackboneFrontier CoreView::pole_frontier(ObjectId pole_id) const {
  BackboneFrontier out{};
  out.pole_id = pole_id;
  const BackboneIndex& index = state_.runtime_.backbone_index;
  const auto node_it = index.pole_node.find(pole_id);
  if (node_it == index.pole_node.end()) {
    return out;
  }
  out.node_id = node_it->second;
  add_frontier_node(state_.authoritative_.backbone, out.node_id, &out);
  if (const auto edges = index.node_edges.find(out.node_id); edges != index.node_edges.end()) {
    for (ObjectId edge_id : edges->second) {
      add_frontier_edge(state_.authoritative_.backbone, index, edge_id, &out);
    }
  }
  return out;
}

BackboneFrontier CoreView::span_frontier(ObjectId span_id) const {
  BackboneFrontier out{};
  out.span_id = span_id;
  const BackboneIndex& index = state_.runtime_.backbone_index;
  const auto edge_bundle_it = index.span_edge_bundle.find(span_id);
  if (edge_bundle_it == index.span_edge_bundle.end()) {
    return out;
  }
  out.edge_bundle_id = edge_bundle_it->second;
  add_id(&out.edge_bundle_ids, out.edge_bundle_id);
  const SavedBackboneEdgeBundle* edge_bundle =
      find_saved_edge_bundle(state_.authoritative_.backbone, out.edge_bundle_id);
  if (edge_bundle == nullptr) {
    return out;
  }
  out.edge_id = edge_bundle->edge_id;
  add_id(&out.bundle_ids, edge_bundle->bundle_id);
  add_frontier_edge(state_.authoritative_.backbone, index, out.edge_id, &out);
  return out;
}

const AttachmentTemplate* CoreView::find_attachment_template(AttachmentTemplateId attachment_template_id) const {
  return state_.find_attachment_template(attachment_template_id);
}
double CoreView::pole_radius_at_height_m(const Pole& pole, double local_z_m) const {
  return state_.pole_radius_at_height_m(pole, local_z_m);
}

CoreView CoreState::view() const { return CoreView(*this); }

} // namespace wire::core
