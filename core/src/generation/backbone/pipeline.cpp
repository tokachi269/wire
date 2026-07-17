#include "pipeline.hpp"

#include "../../collection_utils.hpp"
#include "wire/core/core_view.hpp"
#include "wire/core/coord_utils.hpp"

#include "curve_parts.hpp"
#include "derive_span_layout.hpp"
#include "emit_shared.hpp"
#include "model_assembly.hpp"
#include "out.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace wire::core::generation::backbone {

namespace {

using detail::append_unique;

double elapsed_ms_since(std::chrono::steady_clock::time_point started) {
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
}

template <typename Fn>
auto timed(GenerationTiming* timing, double GenerationTiming::*field, Fn&& fn) {
  const auto started = std::chrono::steady_clock::now();
  auto result = fn();
  if (timing != nullptr) {
    timing->*field = elapsed_ms_since(started);
  }
  return result;
}

void write_route_result(EditResult<GenerateBundleFromPathResult>* out, ChangeSet&& change_set, topo&& made) {
  out->change_set = std::move(change_set);
  out->value.generated_pole_ids = made.new_poles;
  out->value.bundle_ids = made.bundles;
  out->value.bundle_id = made.bundles.empty() ? kInvalidObjectId : made.bundles.front();
  out->value.generated_span_ids.reserve(made.spans.size());
  for (const tspan& span : made.spans) {
    out->value.generated_span_ids.push_back(span.id);
  }
}

} // namespace

EditResult<GenerateBundleFromPathResult> pipeline::build(build_input input) {
  const auto total_started = std::chrono::steady_clock::now();
  g_ = std::move(input.made);
  active_bundle_indices_ = std::move(input.active_bundle_indices);
  local_by_input_ = std::move(input.local_by_input);

  EditResult<GenerateBundleFromPathResult> out{};
  GenerationTiming* timing = input.timing == nullptr ? &out.value.timing : input.timing;
  EditResult<route> made = emit_route(timing);
  if (!made.ok) {
    out.error = made.error;
    return out;
  }
  if (!made.value.active) {
    out.ok = true;
    return out;
  }

  for (const tspan& span : made.value.made.spans) {
    const EditResult<bool> attachments = state_.ensure_default_endpoint_attachments_for_span(span.id);
    if (!attachments.ok) {
      out.error = attachments.error;
      return out;
    }
    append_unique(made.value.change_set.created_ids, attachments.change_set.created_ids);
    append_unique(made.value.change_set.updated_ids, attachments.change_set.updated_ids);
    append_unique(made.value.change_set.deleted_ids, attachments.change_set.deleted_ids);
  }

  EditResult<bool> derived = save_derived(made.value, timing);
  if (!derived.ok) {
    out.error = derived.error;
    return out;
  }
  if (input.retire_untouched) retire_untouched(&made.value);
  write_route_result(&out, std::move(made.value.change_set), std::move(made.value.made));
  if (timing != nullptr) {
    timing->total_ms = elapsed_ms_since(total_started);
    state_.debug_.last_generation_timing = *timing;
  }
  out.ok = true;
  return out;
}

pipeline::build_input pipeline::build_input_from_spec() const {
  build_input input{};
  input.made = g_;
  input.active_bundle_indices = active_bundle_indices_;
  input.local_by_input = local_by_input_;
  return input;
}

pipeline::build_input pipeline::build_input_from_saved_scope(
    graph made_graph, std::vector<std::size_t> active_bundle_indices, bool retire_untouched) const {
  build_input input{};
  input.made = std::move(made_graph);
  input.active_bundle_indices = std::move(active_bundle_indices);
  input.retire_untouched = retire_untouched;
  return input;
}

EditResult<pipeline::route> pipeline::emit_route(GenerationTiming* timing) {
  EditResult<route> out{};
  EditResult<pairs> ps = timed(timing, &GenerationTiming::pairs_ms, [&] { return make(g_); });
  if (!ps.ok) {
    out.error = ps.error;
    return out;
  }
  EditResult<bool> duplicates = timed(timing, &GenerationTiming::preflight_ms, [&] { return check(ps.value); });
  if (!duplicates.ok) {
    out.error = duplicates.error;
    return out;
  }
  EditResult<intent> intents = timed(timing, &GenerationTiming::intent_ms, [&] { return make(ps.value); });
  if (!intents.ok) {
    out.error = intents.error;
    return out;
  }
  if (active_bundle_indices_.empty()) {
    out.value.ps = std::move(ps.value);
    out.ok = true;
    return out;
  }
  EditResult<std::vector<PromotionPlanEntry>> promotions =
      timed(timing, &GenerationTiming::preflight_ms, [&] { return plan_promotions(ps.value); });
  if (!promotions.ok) {
    out.error = promotions.error;
    return out;
  }
  promotion_plan_ = std::move(promotions.value);

  EditResult<groups> placement =
      timed(timing, &GenerationTiming::support_groups_ms, [&] { return make(ps.value, intents.value); });
  if (!placement.ok) {
    out.error = placement.error;
    return out;
  }
  EditResult<topo> made = timed(timing, &GenerationTiming::emit_ms, [&] { return emit(ps.value); });
  if (!made.ok) {
    out.error = made.error;
    return out;
  }
  EditResult<bool> graph_saved =
      timed(timing, &GenerationTiming::save_graph_ms, [&] { return save_graph(made.value, ps.value); });
  if (!graph_saved.ok) {
    out.error = graph_saved.error;
    return out;
  }

  out.value.active = true;
  out.value.change_set = std::move(made.change_set);
  out.value.ps = std::move(ps.value);
  out.value.placement = std::move(placement.value);
  out.value.made = std::move(made.value);
  out.ok = true;
  return out;
}

EditResult<bool> pipeline::save_derived(const route& route, GenerationTiming* timing) {
  EditResult<bool> out{};
  rules saved = timed(timing, &GenerationTiming::rules_ms, [&] {
    rules next = make(route.made, route.ps, route.placement);
    return next;
  });
  EditResult<layout> placed = timed(timing, &GenerationTiming::layout_ms, [&] { return make(saved); });
  if (!placed.ok) {
    out.error = placed.error;
    return out;
  }
  EditResult<geom> shaped = timed(timing, &GenerationTiming::geom_ms, [&] { return make(placed.value); });
  if (!shaped.ok) {
    out.error = shaped.error;
    return out;
  }
  draw drawn = timed(timing, &GenerationTiming::draw_ms, [&] { return make(placed.value, shaped.value); });
  save(saved);
  save(std::move(placed.value));
  save(std::move(shaped.value));
  save(std::move(drawn));
  out.ok = true;
  out.value = true;
  return out;
}

namespace {

constexpr double kRowHeightSeparationM = 0.5;
constexpr double kSharpCornerInteriorAngleMaxDeg = 74.0;
constexpr double kRadiansToDegrees = 57.2957795130823208768;

void add(ChangeSet& dst, const ChangeSet& src) {
  auto append = [](std::vector<ObjectId>& a, const std::vector<ObjectId>& b) {
    for (ObjectId id : b) {
      if (std::find(a.begin(), a.end(), id) == a.end()) {
        a.push_back(id);
      }
    }
  };
  append(dst.created_ids, src.created_ids);
  append(dst.updated_ids, src.updated_ids);
  append(dst.deleted_ids, src.deleted_ids);
}

bool contains_id(const std::vector<ObjectId>& ids, ObjectId id) {
  return std::find(ids.begin(), ids.end(), id) != ids.end();
}

void erase_ids(std::vector<ObjectId>& ids, const std::vector<ObjectId>& removed) {
  ids.erase(std::remove_if(ids.begin(), ids.end(),
                           [&](ObjectId id) { return contains_id(removed, id); }),
            ids.end());
}

struct pole_pull {
  Vec3d dir{};
  std::size_t incident_count = 0;
};

pole_pull pull_for_node(const graph& made, std::size_t node_index) {
  pole_pull out{};
  if (node_index >= made.nodes.size()) {
    return out;
  }
  const Vec3d origin = made.nodes[node_index].pos;
  for (const link& edge : made.links) {
    std::size_t other = bad;
    if (edge.a == node_index) {
      other = edge.b;
    } else if (edge.b == node_index) {
      other = edge.a;
    }
    if (other == bad || other >= made.nodes.size()) {
      continue;
    }
    Vec3d dir = made.nodes[other].pos - origin;
    dir.z = 0.0;
    if (!NormalizeXY(&dir)) {
      continue;
    }
    out.dir = out.dir + dir;
    ++out.incident_count;
  }
  return out;
}

ObjectId saved_node_id_for(const CoreState& state, ObjectId node_or_pole_id) {
  if (node_or_pole_id == kInvalidObjectId) {
    return kInvalidObjectId;
  }
  if (const SavedBackboneNode* saved = state.view().backbone_node(node_or_pole_id); saved != nullptr) {
    return saved->node_id;
  }
  if (const SavedBackboneNode* saved = state.view().backbone_node_for_pole(node_or_pole_id); saved != nullptr) {
    return saved->node_id;
  }
  return kInvalidObjectId;
}

const SavedBackboneNode* saved_source_node_for(const CoreState& state, const SupportNode& node) {
  if (!node.has_source_edge) {
    return nullptr;
  }
  const ObjectId source_a = saved_node_id_for(state, node.source_edge_node_a_id);
  const ObjectId source_b = saved_node_id_for(state, node.source_edge_node_b_id);
  if (source_a == kInvalidObjectId || source_b == kInvalidObjectId || source_a == source_b) {
    return nullptr;
  }
  const ObjectId lo = std::min(source_a, source_b);
  const ObjectId hi = std::max(source_a, source_b);
  for (const SavedBackboneNode& saved : state.view().backbone().nodes) {
    if (saved.pole_id != kInvalidObjectId || saved.support_kind != node.support_kind || !saved.has_source_edge) {
      continue;
    }
    if (std::min(saved.source_edge_node_a, saved.source_edge_node_b) != lo ||
        std::max(saved.source_edge_node_a, saved.source_edge_node_b) != hi) {
      continue;
    }
    if (std::abs(saved.source_edge_t - node.source_edge_t) <= 1e-9) {
      return &saved;
    }
  }
  return nullptr;
}

struct spec_view {
  const BackboneBundleSpec* spec = nullptr;
  const BundleTemplate* tmpl = nullptr;
  int count = 0;
  SpanLayer layer = SpanLayer::kUnknown;
  double spacing_m = 0.0;
};

struct port_scope {
  BundleTemplateId bundle = kInvalidBundleTemplateId;
  ObjectId bundle_id = kInvalidObjectId;
  std::uint64_t placement_key = 0;
  PortKind kind = PortKind::kGeneric;
  PortLayer layer = PortLayer::kUnknown;
  int placement_band_id = 0;
};

EditResult<spec_view> view_for(const CoreState& state, const BackboneBundleSpec& spec) {
  EditResult<spec_view> out{};
  const auto tmpl_it = state.view().bundle_templates().find(spec.bundle_template_id);
  if (tmpl_it == state.view().bundle_templates().end()) {
    out.error = "backbone unsupported: bundle template not found";
    return out;
  }
  const BundleTemplate& tmpl = tmpl_it->second;
  if (tmpl.id != spec.bundle_template_id) {
    out.error = "backbone unsupported: bundle template id mismatch";
    return out;
  }
  if (spec.existing_bundle_id != kInvalidObjectId) {
    const Bundle* existing = state.view().bundles().find(spec.existing_bundle_id);
    if (existing == nullptr || existing->bundle_template_id != spec.bundle_template_id) {
      out.error = "backbone unsupported: existing bundle identity is invalid";
      return out;
    }
  }
  const int count = (tmpl.count_rule == BundleCountRuleKind::kFixed)
                        ? tmpl.fixed_count
                        : ((spec.count > 0) ? spec.count : tmpl.default_count);
  if (count <= 0) {
    out.error = "backbone unsupported: bundle count resolved to zero";
    return out;
  }
  if (tmpl.count_rule == BundleCountRuleKind::kFixed && spec.count > 0 && spec.count != tmpl.fixed_count) {
    out.error = "backbone unsupported: fixed bundle count override";
    return out;
  }
  if (tmpl.count_rule == BundleCountRuleKind::kRange && (count < tmpl.min_count || count > tmpl.max_count)) {
    out.error = "backbone unsupported: bundle count is out of range";
    return out;
  }
  const SpanLayer layer = (spec.layer == SpanLayer::kUnknown) ? tmpl.default_layer : spec.layer;
  if (layer == SpanLayer::kUnknown) {
    out.error = "backbone unsupported: bundle layer is unknown";
    return out;
  }
  if (!std::isfinite(spec.height_m) || !std::isfinite(spec.lateral_m) ||
      !std::isfinite(spec.spacing_m) || spec.spacing_m < 0.0) {
    out.error = "backbone unsupported: bundle placement is invalid";
    return out;
  }
  const double spacing_m = spec.spacing_m > 0.0 ? spec.spacing_m : tmpl.default_spacing_m;
  if (!std::isfinite(spacing_m) || spacing_m <= 0.0) {
    out.error = "backbone unsupported: bundle spacing is invalid";
    return out;
  }
  out.value = spec_view{&spec, &tmpl, count, layer, spacing_m};
  out.ok = true;
  return out;
}

EditResult<PoleTypeId> pole_type_for(const CoreState& state, const BackboneSpec& spec) {
  EditResult<PoleTypeId> out{};
  if (state.view().pole_types().find(spec.pole_type_id) != state.view().pole_types().end()) {
    out.value = spec.pole_type_id;
    out.ok = true;
    return out;
  }
  if (spec.pole_type_id != kInvalidPoleTypeId) {
    out.error = "backbone unsupported: pole type not found";
    return out;
  }
  PoleTypeId resolved = kInvalidPoleTypeId;
  for (const BackboneBundleSpec& bundle : spec.bundles) {
    const auto tmpl_it = state.view().bundle_templates().find(bundle.bundle_template_id);
    if (tmpl_it == state.view().bundle_templates().end()) {
      out.error = "backbone unsupported: bundle template not found";
      return out;
    }
    const PoleTypeId candidate = tmpl_it->second.related_pole_type_id;
    if (candidate == kInvalidPoleTypeId || state.view().pole_types().find(candidate) == state.view().pole_types().end()) {
      out.error = "backbone unsupported: pole type missing";
      return out;
    }
    if (resolved == kInvalidPoleTypeId) {
      resolved = candidate;
      continue;
    }
    if (resolved != candidate) {
      out.error = "backbone unsupported: pole type is ambiguous";
      return out;
    }
  }
  if (resolved == kInvalidPoleTypeId) {
    out.error = "backbone unsupported: pole type not found";
    return out;
  }
  out.value = resolved;
  out.ok = true;
  return out;
}

EditResult<std::vector<PortPlacementBand>> bands_for(const CoreState& state, ObjectId pole_id, const spec_view& view) {
  EditResult<std::vector<PortPlacementBand>> out{};
  const Pole* pole = state.view().poles().find(pole_id);
  if (pole == nullptr || pole->pole_type_id == kInvalidPoleTypeId) {
    out.error = "backbone unsupported: pole type missing";
    return out;
  }
  const auto type_it = state.view().pole_types().find(pole->pole_type_id);
  if (type_it == state.view().pole_types().end()) {
    out.error = "backbone unsupported: pole type missing";
    return out;
  }
  return SelectPortPlacementBands(type_it->second, view.tmpl->category, view.layer, view.count);
}

ObjectId saved_edge_for(const CoreState& state, const graph& made, const link& edge) {
  if (edge.a >= made.nodes.size() || edge.b >= made.nodes.size()) {
    return kInvalidObjectId;
  }
  const ObjectId a = made.nodes[edge.a].saved;
  const ObjectId b = made.nodes[edge.b].saved;
  if (a == kInvalidObjectId || b == kInvalidObjectId || a == b) {
    return kInvalidObjectId;
  }
  const BackboneEdgeKey key{std::min(a, b), std::max(a, b)};
  const auto it = state.view().backbone_index().edge_by_nodes.find(key);
  return it == state.view().backbone_index().edge_by_nodes.end() ? kInvalidObjectId : it->second;
}

ObjectId edge_bundle_for(const CoreState& state, const graph& made, const link& edge, ObjectId bundle_id) {
  const ObjectId edge_id = saved_edge_for(state, made, edge);
  if (edge_id == kInvalidObjectId || bundle_id == kInvalidObjectId) {
    return kInvalidObjectId;
  }
  const auto bundles_it = state.view().backbone_index().edge_bundles.find(edge_id);
  if (bundles_it == state.view().backbone_index().edge_bundles.end()) {
    return kInvalidObjectId;
  }
  ObjectId found = kInvalidObjectId;
  for (ObjectId edge_bundle_id : bundles_it->second) {
    const SavedBackboneEdgeBundle* edge_bundle = state.view().backbone_edge_bundle(edge_bundle_id);
    if (edge_bundle == nullptr || edge_bundle->bundle_id != bundle_id) {
      continue;
    }
    if (found != kInvalidObjectId && found != edge_bundle_id) {
      return kInvalidObjectId;
    }
    found = edge_bundle_id;
  }
  return found;
}

ObjectId existing_span_for_lane(const CoreState& state, ObjectId edge_bundle_id, std::size_t lane_index) {
  if (edge_bundle_id == kInvalidObjectId) {
    return kInvalidObjectId;
  }
  const auto bindings_it = state.view().backbone_index().edge_bundle_span_bindings.find(edge_bundle_id);
  if (bindings_it == state.view().backbone_index().edge_bundle_span_bindings.end()) {
    return kInvalidObjectId;
  }
  ObjectId found = kInvalidObjectId;
  for (std::size_t binding_index : bindings_it->second) {
    if (binding_index >= state.view().backbone().span_bindings.size()) {
      continue;
    }
    const SavedBackboneSpanBinding& binding = state.view().backbone().span_bindings[binding_index];
    if (binding.lane_index != lane_index) {
      continue;
    }
    if (found != kInvalidObjectId && found != binding.span_id) {
      return kInvalidObjectId;
    }
    found = binding.span_id;
  }
  return found;
}
ObjectId resolve_existing_bundle(const CoreState& state, const graph& made, const BackboneBundleSpec& spec) {
  if (spec.existing_bundle_id != kInvalidObjectId) {
    return spec.existing_bundle_id;
  }
  ObjectId resolved = kInvalidObjectId;
  bool saw_existing_edge = false;
  for (const link& edge : made.links) {
    if (!edge.is_new) {
      continue;
    }
    const ObjectId edge_id = saved_edge_for(state, made, edge);
    if (edge_id == kInvalidObjectId) {
      return kInvalidObjectId;
    }
    saw_existing_edge = true;
    ObjectId edge_bundle_match = kInvalidObjectId;
    const auto bundles_it = state.view().backbone_index().edge_bundles.find(edge_id);
    if (bundles_it == state.view().backbone_index().edge_bundles.end()) {
      return kInvalidObjectId;
    }
    for (ObjectId edge_bundle_id : bundles_it->second) {
      const SavedBackboneEdgeBundle* edge_bundle = state.view().backbone_edge_bundle(edge_bundle_id);
      if (edge_bundle == nullptr) {
        continue;
      }
      const Bundle* bundle = state.view().bundles().find(edge_bundle->bundle_id);
      if (bundle == nullptr || bundle->bundle_template_id != spec.bundle_template_id) {
        continue;
      }
      if (spec.placement_key != 0 && bundle->placement_key != 0 &&
          bundle->placement_key != spec.placement_key) {
        continue;
      }
      if (edge_bundle_match != kInvalidObjectId && edge_bundle_match != bundle->id) {
        return kInvalidObjectId;
      }
      edge_bundle_match = bundle->id;
    }
    if (edge_bundle_match == kInvalidObjectId) {
      return kInvalidObjectId;
    }
    if (resolved != kInvalidObjectId && resolved != edge_bundle_match) {
      return kInvalidObjectId;
    }
    resolved = edge_bundle_match;
  }
  return saw_existing_edge ? resolved : kInvalidObjectId;
}

bool has_saved_open_row_for_edge(const CoreState& state, ObjectId node_id, ObjectId edge_id);

SavedBackboneRowKey key_for(const pairs& ps, const trow& row, const std::vector<ObjectId>& node_id_by_local,
                            const std::vector<SavedBackboneEdgeRef>& edge_by_link) {
  SavedBackboneRowKey key{};
  if (row.node >= node_id_by_local.size() || row.source.id == bad) {
    return key;
  }
  key.node_id = node_id_by_local[row.node];
  key.source_is_open = row.source.is_open;
  auto edge_id = [&](std::size_t link_id) -> ObjectId {
    return link_id < edge_by_link.size() ? edge_by_link[link_id].edge_id : kInvalidObjectId;
  };
  if (row.source.is_open) {
    if (row.source.id >= ps.opens.size()) {
      key.node_id = kInvalidObjectId;
      return key;
    }
    key.source_edge_a = edge_id(ps.opens[row.source.id].link);
    return key;
  }
  if (row.source.id >= ps.joins.size()) {
    key.node_id = kInvalidObjectId;
    return key;
  }
  const ObjectId a = edge_id(ps.joins[row.source.id].left);
  const ObjectId b = edge_id(ps.joins[row.source.id].right);
  key.source_edge_a = std::min(a, b);
  key.source_edge_b = std::max(a, b);
  return key;
}

SavedBackboneEdgeRef ref_for_existing_edge(const CoreState& state, const graph& made, const link& edge) {
  SavedBackboneEdgeRef out{};
  (void)made;
  if (edge.saved == kInvalidObjectId) {
    return out;
  }
  const SavedBackboneEdge* saved = state.view().backbone_edge(edge.saved);
  if (saved == nullptr) {
    return out;
  }
  out.edge_id = saved->edge_id;
  out.node_a = saved->node_a;
  out.node_b = saved->node_b;
  return out;
}

ObjectId source_edge_bundle_for(const CoreState& state, ObjectId edge_id,
                                const BackboneBundleSpec& bundle_spec) {
  if (bundle_spec.source_bundle_id != kInvalidObjectId) {
    const BackboneEdgeBundleKey key{edge_id, bundle_spec.source_bundle_id};
    const auto it = state.view().backbone_index().edge_bundle_by_edge_and_bundle.find(key);
    return it == state.view().backbone_index().edge_bundle_by_edge_and_bundle.end()
               ? kInvalidObjectId
               : it->second;
  }
  const auto edge_bundles_it = state.view().backbone_index().edge_bundles.find(edge_id);
  if (edge_bundles_it == state.view().backbone_index().edge_bundles.end()) {
    return kInvalidObjectId;
  }
  ObjectId matched = kInvalidObjectId;
  for (ObjectId edge_bundle_id : edge_bundles_it->second) {
    const SavedBackboneEdgeBundle* edge_bundle = state.view().backbone_edge_bundle(edge_bundle_id);
    const Bundle* bundle = edge_bundle == nullptr ? nullptr : state.view().bundles().find(edge_bundle->bundle_id);
    if (bundle == nullptr || bundle->bundle_template_id != bundle_spec.bundle_template_id) {
      continue;
    }
    if (bundle_spec.placement_key != 0 && bundle->placement_key != 0 &&
        bundle->placement_key != bundle_spec.placement_key) {
      continue;
    }
    if (matched != kInvalidObjectId) {
      return kInvalidObjectId;
    }
    matched = edge_bundle_id;
  }
  return matched;
}

SourceEdgeProjectionRef source_projection_for(const CoreState& state, const node& source,
                                              const BackboneBundleSpec& bundle_spec, std::size_t lane) {
  SourceEdgeProjectionRef out{};
  const ObjectId source_a = saved_node_id_for(state, source.source_edge_node_a);
  const ObjectId source_b = saved_node_id_for(state, source.source_edge_node_b);
  if (!source.has_source_edge || source_a == kInvalidObjectId || source_b == kInvalidObjectId ||
      source_a == source_b) {
    return out;
  }
  const BackboneEdgeKey key{std::min(source_a, source_b), std::max(source_a, source_b)};
  const auto edge_it = state.view().backbone_index().edge_by_nodes.find(key);
  if (edge_it == state.view().backbone_index().edge_by_nodes.end()) {
    return out;
  }
  out.source_edge_id = edge_it->second;
  out.source_edge_bundle_id = source_edge_bundle_for(state, out.source_edge_id, bundle_spec);
  out.from_node_id = source_a;
  out.bundle_template_id = bundle_spec.bundle_template_id;
  out.lane_index = lane;
  out.t = source.source_edge_t;
  return out;
}

bool same_scope(const CoreState& state, const SavedBackbonePortBinding& binding, port_scope scope) {
  if (binding.bundle_template_id != scope.bundle || binding.port_kind != scope.kind ||
      binding.port_layer != scope.layer) {
    return false;
  }
  if (scope.bundle_id == kInvalidObjectId) {
    if (scope.placement_key == 0) {
      return true;
    }
    const SavedBackboneEdgeBundle* edge_bundle = state.view().backbone_edge_bundle(binding.edge_bundle_id);
    const Bundle* bundle = edge_bundle == nullptr ? nullptr : state.view().bundles().find(edge_bundle->bundle_id);
    return bundle != nullptr &&
           (bundle->placement_key == 0 || bundle->placement_key == scope.placement_key);
  }
  const SavedBackboneEdgeBundle* edge_bundle = state.view().backbone_edge_bundle(binding.edge_bundle_id);
  return edge_bundle != nullptr && edge_bundle->bundle_id == scope.bundle_id;
}

bool is_open_row_for_edge(const SavedBackboneRowKey& row_key, ObjectId node_id, ObjectId edge_id) {
  return row_key.node_id == node_id && row_key.source_is_open && row_key.source_edge_a == edge_id &&
         row_key.source_edge_b == kInvalidObjectId;
}

bool has_saved_open_row_for_edge(const CoreState& state, ObjectId node_id, ObjectId edge_id) {
  if (node_id == kInvalidObjectId || edge_id == kInvalidObjectId) {
    return false;
  }
  for (const SavedBackbonePortBinding& binding : state.view().backbone().port_bindings) {
    if (is_open_row_for_edge(binding.row_key, node_id, edge_id)) {
      return true;
    }
  }
  return false;
}

bool has_saved_open_row_at_node(const CoreState& state, ObjectId node_id) {
  if (node_id == kInvalidObjectId) {
    return false;
  }
  for (const SavedBackbonePortBinding& binding : state.view().backbone().port_bindings) {
    if (binding.row_key.node_id == node_id && binding.row_key.source_is_open) {
      return true;
    }
  }
  return false;
}

bool has_saved_pair_row_for_edge(const CoreState& state, ObjectId node_id, ObjectId edge_id) {
  if (node_id == kInvalidObjectId || edge_id == kInvalidObjectId) {
    return false;
  }
  for (const SavedBackbonePortBinding& binding : state.view().backbone().port_bindings) {
    if (binding.row_key.node_id != node_id || binding.row_key.source_is_open) {
      continue;
    }
    if (binding.row_key.source_edge_a == edge_id || binding.row_key.source_edge_b == edge_id) {
      return true;
    }
  }
  return false;
}

bool has_saved_pair_row_for_edges(const CoreState& state, ObjectId node_id, ObjectId edge_a, ObjectId edge_b) {
  if (node_id == kInvalidObjectId || edge_a == kInvalidObjectId || edge_b == kInvalidObjectId) {
    return false;
  }
  const ObjectId lo = std::min(edge_a, edge_b);
  const ObjectId hi = std::max(edge_a, edge_b);
  for (const SavedBackbonePortBinding& binding : state.view().backbone().port_bindings) {
    if (binding.row_key.node_id == node_id && !binding.row_key.source_is_open &&
        binding.row_key.source_edge_a == lo && binding.row_key.source_edge_b == hi) {
      return true;
    }
  }
  return false;
}

bool has_saved_pair_row_at_node(const CoreState& state, ObjectId node_id) {
  if (node_id == kInvalidObjectId) {
    return false;
  }
  for (const SavedBackbonePortBinding& binding : state.view().backbone().port_bindings) {
    if (binding.row_key.node_id == node_id && !binding.row_key.source_is_open) {
      return true;
    }
  }
  return false;
}

bool makes_pole(const node& item) {
  return item.support == SupportKind::kPole && item.is_new;
}

bool needs_pole_type(const graph& made) {
  return std::any_of(made.nodes.begin(), made.nodes.end(), makes_pole);
}

bool has_band(const PoleTypeDefinition& pole_type, const spec_view& spec) {
  return SelectPortPlacementBands(pole_type, spec.tmpl->category, spec.layer, spec.count).ok;
}

EditResult<bool> check_port_bands(const CoreState& state, const graph& made, const BackboneSpec& spec,
                                  const std::vector<std::size_t>& active_bundle_indices) {
  for (const node& item : made.nodes) {
    if (!item.on_route) {
      continue;
    }
    if (item.support == SupportKind::kMidair || item.support == SupportKind::kExternal ||
        item.support == SupportKind::kGround) {
      continue;
    }
    PoleTypeId pole_type_id = kInvalidPoleTypeId;
    if (item.is_new) {
      EditResult<PoleTypeId> resolved_type = pole_type_for(state, spec);
      if (!resolved_type.ok) {
        EditResult<bool> failed{};
        failed.error = resolved_type.error;
        return failed;
      }
      pole_type_id = resolved_type.value;
    } else {
      const ObjectId pole_id = item.pole;
      if (pole_id == kInvalidObjectId) {
        EditResult<bool> failed{};
        failed.error = "backbone unsupported: existing pole is missing";
        return failed;
      }
      const Pole* pole = state.view().poles().find(pole_id);
      if (pole == nullptr) {
        EditResult<bool> failed{};
        failed.error = "backbone unsupported: existing pole is missing";
        return failed;
      }
      pole_type_id = pole->pole_type_id;
    }
    const auto type_it = state.view().pole_types().find(pole_type_id);
    if (type_it == state.view().pole_types().end()) {
      EditResult<bool> failed{};
      failed.error = "backbone unsupported: pole type missing";
      return failed;
    }
    for (const std::size_t bundle_index : active_bundle_indices) {
      if (bundle_index >= spec.bundles.size()) {
        EditResult<bool> failed{};
        failed.error = "backbone unsupported: active bundle index is invalid";
        return failed;
      }
      const BackboneBundleSpec& bundle = spec.bundles[bundle_index];
      EditResult<spec_view> checked = view_for(state, bundle);
      if (!checked.ok) {
        EditResult<bool> failed{};
        failed.error = checked.error;
        return failed;
      }
      if (!has_band(type_it->second, checked.value)) {
        EditResult<bool> failed{};
        failed.error = "backbone unsupported: port band missing";
        return failed;
      }
    }
  }
  EditResult<bool> out{};
  out.value = true;
  out.ok = true;
  return out;
}

EditResult<ObjectId> resolve_port_binding(const CoreState& state, ObjectId pole_id, const SavedBackboneRowKey& row_key,
                                          std::size_t lane_index, port_scope scope) {
  EditResult<ObjectId> out{};
  out.value = kInvalidObjectId;
  out.ok = true;
  if (pole_id == kInvalidObjectId || row_key.node_id == kInvalidObjectId ||
      row_key.source_edge_a == kInvalidObjectId) {
    return out;
  }
  ObjectId found = kInvalidObjectId;
  for (const SavedBackbonePortBinding* binding : state.view().backbone_port_bindings_for_row(row_key, lane_index)) {
    if (binding == nullptr) {
      continue;
    }
    if (!same_scope(state, *binding, scope)) {
      continue;
    }
    const Port* port = state.view().ports().find(binding->port_id);
    if (port == nullptr || port->owner_pole_id != pole_id || port->kind != scope.kind || port->layer != scope.layer) {
      continue;
    }
    if (found != kInvalidObjectId && found != port->id) {
      out.error = "backbone unsupported: ambiguous backbone port binding";
      out.ok = false;
      return out;
    }
    found = port->id;
  }
  out.value = found;
  return out;
}

std::vector<Vec3d> row_height_offsets(const pairs& ps) {
  std::vector<Vec3d> offsets(ps.rows.size(), Vec3d{});
  std::vector<bool> active_rows(ps.rows.size(), false);
  for (const link& edge : ps.links) {
    if (!edge.is_new) {
      continue;
    }
    if (edge.arow < active_rows.size()) {
      active_rows[edge.arow] = true;
    }
    if (edge.brow < active_rows.size()) {
      active_rows[edge.brow] = true;
    }
  }
  std::unordered_map<std::size_t, std::vector<std::size_t>> rows_by_node{};
  for (const row& r : ps.rows) {
    rows_by_node[r.node].push_back(r.id);
  }
  for (auto& item : rows_by_node) {
    std::vector<std::size_t>& rows = item.second;
    std::sort(rows.begin(), rows.end(), [&](std::size_t lhs, std::size_t rhs) {
      const bool lhs_active = lhs < active_rows.size() && active_rows[lhs];
      const bool rhs_active = rhs < active_rows.size() && active_rows[rhs];
      if (lhs_active != rhs_active) {
        return lhs_active;
      }
      return lhs < rhs;
    });
    const double center = (static_cast<double>(rows.size()) - 1.0) * 0.5;
    for (std::size_t order = 0; order < rows.size(); ++order) {
      const std::size_t row_id = rows[order];
      if (row_id >= ps.rows.size()) {
        continue;
      }
      const double amount = (static_cast<double>(order) - center) * kRowHeightSeparationM;
      offsets[row_id] = Vec3d{0.0, 0.0, amount};
    }
  }
  return offsets;
}

bool route_clear_of_avoid_points(const graph& made, const std::vector<Vec3d>& points, double radius) {
  if (points.empty() || radius <= 0.0) {
    return true;
  }
  const double radius2 = radius * radius;
  const auto same_point = [](const Vec3d& lhs, const Vec3d& rhs) {
    return DistanceSquared(lhs, rhs) <= 1e-18;
  };
  for (const Vec3d& point : points) {
    if (!made.nodes.empty() && (same_point(point, made.nodes.front().pos) || same_point(point, made.nodes.back().pos))) {
      continue;
    }
    const auto node_it = std::find_if(made.nodes.begin(), made.nodes.end(), [&](const node& n) {
      return (!n.is_new || n.explicit_support) && same_point(point, n.pos);
    });
    if (node_it != made.nodes.end()) {
      continue;
    }
    for (const link& edge : made.links) {
      if (!edge.is_new || edge.a >= made.nodes.size() || edge.b >= made.nodes.size()) {
        continue;
      }
      if (DistanceSquaredToSegment(point, made.nodes[edge.a].pos, made.nodes[edge.b].pos) <= radius2) {
        return false;
      }
    }
  }
  return true;
}

struct segment_insert {
  double t = 0.0;
  Vec3d p{};
  SupportKind support = SupportKind::kPole;
  bool detour = false;
};

bool ownerless_support(SupportKind kind) {
  return kind == SupportKind::kMidair || kind == SupportKind::kExternal || kind == SupportKind::kGround;
}

EditResult<std::size_t> avoid_detours_for_segment(const Vec3d& a, const Vec3d& b, const std::vector<Vec3d>& points,
                                                  double radius, SupportKind inserted_support,
                                                  std::vector<segment_insert>* inserts) {
  EditResult<std::size_t> out{};
  out.value = 0;
  if (points.empty() || radius <= 0.0) {
    out.ok = true;
    return out;
  }
  Vec3d ab = b - a;
  const double len2 = LengthSquared(ab);
  if (len2 <= 1e-12) {
    out.error = "backbone unsupported: avoid routing source segment is zero length";
    return out;
  }
  Vec3d dir = ab;
  if (!NormalizeXY(&dir)) {
    out.error = "backbone unsupported: avoid routing source segment is zero length";
    return out;
  }
  const Vec3d detour_axis = ComputeLateralAxis(dir);
  constexpr double kAvoidClearanceM = 0.5;
  for (const Vec3d& p : points) {
    const double t = Dot(p - a, ab) / len2;
    if (t <= 1e-6 || t >= 1.0 - 1e-6 || DistanceSquaredToSegment(p, a, b) > radius * radius) {
      continue;
    }
    const Vec3d closest = {a.x + ab.x * t, a.y + ab.y * t, a.z + ab.z * t};
    const double len_m = Length(ab);
    const double before_m = t * len_m;
    const double after_m = (1.0 - t) * len_m;
    if (radius >= before_m || radius >= after_m) {
      out.error = "backbone unsupported: avoid radius reaches a fixed route endpoint";
      return out;
    }
    double clearance = radius + kAvoidClearanceM;
    clearance = std::min(clearance, std::min(before_m, after_m) - 1e-6);
    auto required_detour = [&](double along_m) {
      const double denom2 = along_m * along_m - clearance * clearance;
      return denom2 > 1e-12 ? (clearance * along_m) / std::sqrt(denom2) : along_m + radius + kAvoidClearanceM;
    };
    const double detour = std::max(required_detour(before_m), required_detour(after_m));
    if (inserts != nullptr) {
      inserts->push_back(
          {t, closest + Vec3d{detour_axis.x * detour, detour_axis.y * detour, 0.0}, inserted_support, true});
    }
    ++out.value;
  }
  out.ok = true;
  return out;
}

bool detour_internal_vertex(Vec3d prev, Vec3d current, Vec3d next, const std::vector<Vec3d>& points, double radius,
                            Vec3d* out) {
  if (points.empty() || radius <= 0.0) {
    return false;
  }
  const double radius2 = radius * radius;
  bool needs_detour = false;
  for (const Vec3d& point : points) {
    if (DistanceSquared(point, current) <= radius2) {
      needs_detour = true;
      break;
    }
  }
  if (!needs_detour) {
    return false;
  }
  Vec3d chord = next - prev;
  if (!NormalizeXY(&chord)) {
    chord = current - prev;
    if (!NormalizeXY(&chord)) {
      chord = next - current;
      if (!NormalizeXY(&chord)) {
        return false;
      }
    }
  }
  constexpr double kAvoidClearanceM = 0.5;
  const Vec3d axis = ComputeLateralAxis(chord);
  *out = current + Vec3d{axis.x * (radius + kAvoidClearanceM), axis.y * (radius + kAvoidClearanceM), 0.0};
  return true;
}

EditResult<bool> unsupported(std::string reason) {
  EditResult<bool> out{};
  out.error = "backbone unsupported: " + std::move(reason);
  return out;
}

EditResult<pairs> unsupported_pairs(std::string reason) {
  EditResult<pairs> out{};
  out.error = "backbone unsupported: " + std::move(reason);
  return out;
}

bool has_current_route_pair_at(const graph& made, std::size_t node_id) {
  const link* incoming = nullptr;
  const link* outgoing = nullptr;
  for (const link& edge : made.links) {
    if (!edge.is_new) {
      continue;
    }
    if (edge.b == node_id) {
      if (incoming != nullptr) {
        return false;
      }
      incoming = &edge;
    }
    if (edge.a == node_id) {
      if (outgoing != nullptr) {
        return false;
      }
      outgoing = &edge;
    }
  }
  return incoming != nullptr && outgoing != nullptr && incoming->route == outgoing->route &&
         incoming->order + 1 == outgoing->order;
}

std::size_t current_route_incident_count_at(const graph& made, std::size_t node_id) {
  std::size_t count = 0;
  for (const link& edge : made.links) {
    if (!edge.is_new) {
      continue;
    }
    if (edge.a == node_id || edge.b == node_id) {
      ++count;
    }
  }
  return count;
}

std::size_t context_incident_count_at(const graph& made, std::size_t node_id) {
  std::size_t count = 0;
  for (const link& edge : made.links) {
    if (edge.is_new) {
      continue;
    }
    if (edge.a == node_id || edge.b == node_id) {
      ++count;
    }
  }
  return count;
}

std::size_t add_open(pairs* out, std::size_t node_id, std::size_t link_id, const Vec3d& axis) {
  const std::size_t open_id = out->opens.size();
  out->opens.push_back(open{open_id, node_id, link_id, axis});
  const std::size_t row_id = out->rows.size();
  out->rows.push_back(row{row_id, node_id, src{true, open_id}, axis});
  return row_id;
}

std::size_t add_pair(pairs* out, std::size_t node_id, std::size_t left, std::size_t right, const Vec3d& axis) {
  const std::size_t pair_id = out->joins.size();
  out->joins.push_back(pair{pair_id, node_id, left, right, axis});
  const std::size_t row_id = out->rows.size();
  out->rows.push_back(row{row_id, node_id, src{false, pair_id}, axis});
  return row_id;
}

void flip_open_row_axis(pairs* out, std::size_t row_id) {
  if (out == nullptr || row_id >= out->rows.size()) {
    return;
  }
  row& item = out->rows[row_id];
  if (!item.source.is_open) {
    return;
  }
  item.axis = ScaleVec(item.axis, -1.0);
  if (item.source.id < out->opens.size()) {
    out->opens[item.source.id].axis = item.axis;
  }
}

void align_open_row_axis_to_pair(pairs* out, std::size_t open_row_id, std::size_t pair_row_id,
                                 const Vec3d& edge_lateral_axis) {
  if (out == nullptr || open_row_id >= out->rows.size() || pair_row_id >= out->rows.size()) {
    return;
  }
  const row& open_row = out->rows[open_row_id];
  const row& pair_row = out->rows[pair_row_id];
  if (!open_row.source.is_open || pair_row.source.is_open) {
    return;
  }
  const double open_dot = Dot(open_row.axis, edge_lateral_axis);
  const double pair_dot = Dot(pair_row.axis, edge_lateral_axis);
  if (std::abs(open_dot) <= 1e-9 || std::abs(pair_dot) <= 1e-9 || open_dot * pair_dot >= 0.0) {
    return;
  }
  flip_open_row_axis(out, open_row_id);
}

void align_terminal_open_rows_to_pairs(pairs* out) {
  if (out == nullptr) {
    return;
  }
  for (const link& edge : out->links) {
    if (edge.arow >= out->rows.size() || edge.brow >= out->rows.size()) {
      continue;
    }
    const Vec3d edge_lateral_axis = ComputeLateralAxis(edge.dir);
    if (out->rows[edge.arow].source.is_open && !out->rows[edge.brow].source.is_open) {
      align_open_row_axis_to_pair(out, edge.arow, edge.brow, edge_lateral_axis);
    } else if (!out->rows[edge.arow].source.is_open && out->rows[edge.brow].source.is_open) {
      align_open_row_axis_to_pair(out, edge.brow, edge.arow, edge_lateral_axis);
    }
  }
}

double interior_angle_deg(const link& incoming, const link& outgoing) {
  const double dot = std::clamp(Dot(ScaleVec(incoming.dir, -1.0), outgoing.dir), -1.0, 1.0);
  return std::acos(dot) * kRadiansToDegrees;
}

Vec3d away_from_node(const link& edge, std::size_t node_id) {
  if (edge.a == node_id) {
    return edge.dir;
  }
  if (edge.b == node_id) {
    return ScaleVec(edge.dir, -1.0);
  }
  return {};
}

std::uint64_t canonical_edge_order_key(const link& edge) {
  if (edge.saved != kInvalidObjectId) {
    return static_cast<std::uint64_t>(edge.saved);
  }
  return (std::uint64_t{1} << 63) + static_cast<std::uint64_t>(edge.id);
}

Vec3d canonical_pair_axis(const link& a, const link& b, std::size_t node_id) {
  const bool a_low = canonical_edge_order_key(a) < canonical_edge_order_key(b);
  const Vec3d low = away_from_node(a_low ? a : b, node_id);
  const Vec3d high = away_from_node(a_low ? b : a, node_id);
  return high - low;
}

bool is_promoted_open_continuation(const CoreState& state, const node& n, const link& a, const link& b) {
  if (!n.on_route || a.route == b.route || (a.route != 0 && b.route != 0) || a.is_new == b.is_new) {
    return false;
  }
  const link& context = a.is_new ? b : a;
  return context.saved != kInvalidObjectId && n.saved != kInvalidObjectId &&
         has_saved_open_row_for_edge(state, n.saved, context.saved) &&
         interior_angle_deg(a, b) > 1e-6;
}

bool is_saved_pair_continuation(const CoreState& state, const node& n, const link& a, const link& b) {
  return n.saved != kInvalidObjectId && a.saved != kInvalidObjectId && b.saved != kInvalidObjectId &&
         has_saved_pair_row_for_edges(state, n.saved, a.saved, b.saved);
}

bool moved_more_than_epsilon(const Vec3d& lhs, const Vec3d& rhs) {
  const Vec3d d = lhs - rhs;
  return d.x * d.x + d.y * d.y + d.z * d.z > 1e-18;
}

} // namespace

void pipeline::retire_untouched(route* route) {
  if (route == nullptr) {
    return;
  }
  auto add_id = [](std::vector<ObjectId>& ids, ObjectId id) {
    if (id != kInvalidObjectId && !contains_id(ids, id)) {
      ids.push_back(id);
    }
  };
  for (const tspan& span : route->made.spans) {
    add_id(route->touched_span_ids, span.id);
    if (span.arow < route->made.rows.size() && span.bundle < route->made.rows[span.arow].ports.size() &&
        span.lane < route->made.rows[span.arow].ports[span.bundle].size()) {
      add_id(route->touched_port_ids, route->made.rows[span.arow].ports[span.bundle][span.lane]);
    }
    if (span.brow < route->made.rows.size() && span.bundle < route->made.rows[span.brow].ports.size() &&
        span.lane < route->made.rows[span.brow].ports[span.bundle].size()) {
      add_id(route->touched_port_ids, route->made.rows[span.brow].ports[span.bundle][span.lane]);
    }
  }
  for (const link& edge : route->ps.links) {
    if (!edge.is_new || edge.saved == kInvalidObjectId) {
      continue;
    }
    for (ObjectId bundle_id : route->made.bundles) {
      add_id(route->scope_edge_bundle_ids, edge_bundle_for(state_, g_, edge, bundle_id));
    }
  }
  if (route->scope_edge_bundle_ids.empty()) {
    return;
  }

  std::vector<ObjectId> retired_spans{};
  std::vector<ObjectId> retired_ports{};
  const SavedBackboneGraph& saved = state_.view().backbone();
  for (ObjectId edge_bundle_id : route->scope_edge_bundle_ids) {
    const auto span_binding_it = state_.runtime_.backbone_index.edge_bundle_span_bindings.find(edge_bundle_id);
    if (span_binding_it != state_.runtime_.backbone_index.edge_bundle_span_bindings.end()) {
      for (std::size_t binding_index : span_binding_it->second) {
        if (binding_index >= saved.span_bindings.size()) {
          continue;
        }
        ObjectId span_id = saved.span_bindings[binding_index].span_id;
        if (!contains_id(route->touched_span_ids, span_id)) {
          add_id(retired_spans, span_id);
        }
      }
    }
    const auto port_binding_it = state_.runtime_.backbone_index.edge_bundle_ports.find(edge_bundle_id);
    if (port_binding_it != state_.runtime_.backbone_index.edge_bundle_ports.end()) {
      for (std::size_t binding_index : port_binding_it->second) {
        if (binding_index >= saved.port_bindings.size()) {
          continue;
        }
        ObjectId port_id = saved.port_bindings[binding_index].port_id;
        if (!contains_id(route->touched_port_ids, port_id)) {
          add_id(retired_ports, port_id);
        }
      }
    }
  }
  if (retired_spans.empty() && retired_ports.empty()) {
    return;
  }

  for (ObjectId span_id : retired_spans) {
    const Span* span = state_.authoritative_.edit_state.spans.find(span_id);
    if (span == nullptr) {
      continue;
    }
    const Span copy = *span;
    std::vector<ObjectId> attachment_ids{};
    if (const auto attachments_it = state_.runtime_.relation_index.attachments_by_span.find(span_id);
        attachments_it != state_.runtime_.relation_index.attachments_by_span.end()) {
      attachment_ids = attachments_it->second;
    }
    state_.remove_span_from_indexes(copy);
    state_.authoritative_.edit_state.spans.remove(span_id);
    state_.runtime_.span_runtime_states.erase(span_id);
    state_.remove_span_from_caches(span_id);
    for (ObjectId attachment_id : attachment_ids) {
      if (state_.authoritative_.edit_state.attachments.remove(attachment_id)) {
        CoreState::add_unique_id(route->change_set.deleted_ids, attachment_id);
      }
    }
    state_.runtime_.relation_index.attachments_by_span.erase(span_id);
    CoreState::add_unique_id(route->change_set.deleted_ids, span_id);
  }
  for (ObjectId port_id : retired_ports) {
    const Port* port = state_.authoritative_.edit_state.ports.find(port_id);
    if (port == nullptr) {
      continue;
    }
    CoreState::index_remove(state_.runtime_.relation_index.ports_by_pole, port->owner_pole_id, port_id);
    state_.runtime_.connection_index.spans_by_port.erase(port_id);
    state_.authoritative_.edit_state.ports.remove(port_id);
    CoreState::add_unique_id(route->change_set.deleted_ids, port_id);
  }

  SavedBackboneGraph& graph = state_.authoritative_.backbone;
  for (SavedBackboneEdgeBundle& item : graph.edge_bundles) {
    if (contains_id(route->scope_edge_bundle_ids, item.edge_bundle_id)) {
      erase_ids(item.span_ids, retired_spans);
    }
  }
  graph.span_bindings.erase(
      std::remove_if(graph.span_bindings.begin(), graph.span_bindings.end(),
                     [&](const SavedBackboneSpanBinding& binding) {
                       return contains_id(retired_spans, binding.span_id);
                     }),
      graph.span_bindings.end());
  graph.port_bindings.erase(
      std::remove_if(graph.port_bindings.begin(), graph.port_bindings.end(),
                     [&](const SavedBackbonePortBinding& binding) {
                       return contains_id(retired_ports, binding.port_id);
                     }),
      graph.port_bindings.end());

  BackboneIndex rebuilt{};
  for (const SavedBackboneNode& node : graph.nodes) {
    if (node.pole_id != kInvalidObjectId) {
      rebuilt.pole_node[node.pole_id] = node.node_id;
    }
  }
  for (const SavedBackboneEdge& item : graph.edges) {
    CoreState::index_add(rebuilt.node_edges, item.node_a, item.edge_id);
    CoreState::index_add(rebuilt.node_edges, item.node_b, item.edge_id);
    const BackboneEdgeKey key{std::min(item.node_a, item.node_b), std::max(item.node_a, item.node_b)};
    rebuilt.edge_by_nodes[key] = item.edge_id;
  }
  for (std::size_t position = 0; position < graph.edge_bundles.size(); ++position) {
    const SavedBackboneEdgeBundle& item = graph.edge_bundles[position];
    CoreState::index_add(rebuilt.edge_bundles, item.edge_id, item.edge_bundle_id);
    rebuilt.edge_bundle_by_edge_and_bundle[{item.edge_id, item.bundle_id}] = item.edge_bundle_id;
    rebuilt.edge_bundle_positions[item.edge_bundle_id] = position;
    CoreState::index_add(rebuilt.bundle_edge, item.bundle_id, item.edge_id);
    for (ObjectId span_id : item.span_ids) {
      CoreState::index_add(rebuilt.edge_bundle_spans, item.edge_bundle_id, span_id);
      rebuilt.span_edge_bundle[span_id] = item.edge_bundle_id;
    }
  }
  for (std::size_t i = 0; i < graph.span_bindings.size(); ++i) {
    const SavedBackboneSpanBinding& binding = graph.span_bindings[i];
    rebuilt.edge_bundle_span_bindings[binding.edge_bundle_id].push_back(i);
    rebuilt.span_bindings_by_span[binding.span_id].push_back(i);
  }
  for (std::size_t i = 0; i < graph.port_bindings.size(); ++i) {
    const SavedBackbonePortBinding& binding = graph.port_bindings[i];
    rebuilt.edge_bundle_ports[binding.edge_bundle_id].push_back(i);
    rebuilt.port_bindings_by_port[binding.port_id].push_back(i);
  }
  state_.runtime_.backbone_index = std::move(rebuilt);
}

std::size_t pipeline::local(std::size_t input_point) const {
  return input_point < local_by_input_.size() ? local_by_input_[input_point] : bad;
}

EditResult<bool> pipeline::prepare() {
  EditResult<bool> out{};
  g_ = {};
  active_bundle_indices_.clear();
  local_by_input_.clear();
  const std::vector<Vec3d>& input = spec_.path.polyline;
  std::vector<Vec3d> guide = input;
  std::unordered_map<std::size_t, const BackboneInputSpec::NodeSpec*> spec_by_point{};
  spec_by_point.reserve(spec_.path.node_specs.size());
  for (const BackboneInputSpec::NodeSpec& spec : spec_.path.node_specs) {
    if (spec.point_index >= input.size()) {
      out.error = "backbone unsupported: node spec point index is out of range";
      return out;
    }
    if (spec.support_kind != SupportKind::kPole && spec.support_kind != SupportKind::kMidair &&
        spec.support_kind != SupportKind::kExternal && spec.support_kind != SupportKind::kGround) {
      out.error = "backbone unsupported: node specs require pole, midair, external, or ground support";
      return out;
    }
    if (spec.has_tangent_hint) {
      Vec3d tangent = spec.tangent_hint;
      if (!NormalizeXY(&tangent)) {
        out.error = "backbone unsupported: node spec tangent hint is zero";
        return out;
      }
    }
    if (!spec_by_point.emplace(spec.point_index, &spec).second) {
      out.error = "backbone unsupported: duplicate node spec";
      return out;
    }
  }
  if (spec_.direction_mode == PathDirectionMode::kReverse) {
    std::reverse(guide.begin(), guide.end());
    std::unordered_map<std::size_t, const BackboneInputSpec::NodeSpec*> reversed{};
    reversed.reserve(spec_by_point.size());
    for (const auto& item : spec_by_point) {
      reversed.emplace(guide.size() - 1 - item.first, item.second);
    }
    spec_by_point.swap(reversed);
  }
  std::vector<Vec3d> pts{};
  std::vector<std::size_t> guide_by_local{};
  std::vector<SupportKind> support_by_local{};
  pts.reserve(guide.size());
  guide_by_local.reserve(guide.size());
  support_by_local.reserve(guide.size());
  auto push_point = [&](const Vec3d& p, std::size_t guide_index, SupportKind support = SupportKind::kPole) {
    pts.push_back(p);
    guide_by_local.push_back(guide_index);
    support_by_local.push_back(support);
  };
  auto inserted_support_for_segment = [&](std::size_t a, std::size_t b) {
    const auto spec_a = spec_by_point.find(a);
    const auto spec_b = spec_by_point.find(b);
    if (spec_a == spec_by_point.end() || spec_b == spec_by_point.end()) {
      return SupportKind::kPole;
    }
    const SupportKind support = spec_a->second->support_kind;
    return support == spec_b->second->support_kind && ownerless_support(support) ? support : SupportKind::kPole;
  };
  if (!guide.empty()) {
    push_point(guide.front(), 0);
    for (std::size_t i = 0; i + 1 < guide.size(); ++i) {
      const Vec3d a = guide[i];
      Vec3d b = guide[i + 1];
      if (i + 2 < guide.size() && spec_by_point.find(i + 1) == spec_by_point.end()) {
        Vec3d detoured{};
        if (detour_internal_vertex(guide[i], guide[i + 1], guide[i + 2], spec_.constraints.avoid_points,
                                   spec_.constraints.avoid_radius_m, &detoured)) {
          b = detoured;
          guide[i + 1] = detoured;
        }
      }
      const SupportKind inserted_support = inserted_support_for_segment(i, i + 1);
      const Vec3d seg = b - a;
      constexpr double kIntervalEps = 1e-9;
      const double len = Length(seg);
      std::vector<segment_insert> inserts{};
      if (spec_.interval_m > 0.0 && len > kIntervalEps) {
        for (std::size_t step = 1;; ++step) {
          const double dist = spec_.interval_m * static_cast<double>(step);
          if (dist >= len - kIntervalEps) {
            break;
          }
          const double t = std::clamp(dist / len, 0.0, 1.0);
          inserts.push_back({t, {a.x + seg.x * t, a.y + seg.y * t, a.z + seg.z * t}, inserted_support, false});
        }
      }
      EditResult<std::size_t> avoid =
          avoid_detours_for_segment(a, b, spec_.constraints.avoid_points, spec_.constraints.avoid_radius_m,
                                    inserted_support, &inserts);
      if (!avoid.ok) {
        out.error = avoid.error;
        return out;
      }
      std::sort(inserts.begin(), inserts.end(), [](const segment_insert& lhs, const segment_insert& rhs) {
        if (std::abs(lhs.t - rhs.t) > 1e-9) {
          return lhs.t < rhs.t;
        }
        const auto len2 = [](const Vec3d& p) { return p.x * p.x + p.y * p.y + p.z * p.z; };
        return len2(lhs.p) < len2(rhs.p);
      });
      std::vector<segment_insert> unique_inserts{};
      unique_inserts.reserve(inserts.size());
      const auto same_point = [](const Vec3d& lhs, const Vec3d& rhs) {
        const Vec3d d = lhs - rhs;
        return d.x * d.x + d.y * d.y + d.z * d.z <= 1e-18;
      };
      for (const segment_insert& item : inserts) {
        if (!unique_inserts.empty() && std::abs(unique_inserts.back().t - item.t) <= 1e-9 &&
            unique_inserts.back().support == item.support) {
          if (item.detour && !unique_inserts.back().detour) {
            unique_inserts.back() = item;
          }
          continue;
        }
        const bool duplicate = !unique_inserts.empty() && unique_inserts.back().support == item.support &&
                               same_point(unique_inserts.back().p, item.p);
        if (duplicate) {
          continue;
        }
        unique_inserts.push_back(item);
      }
      for (const segment_insert& item : unique_inserts) {
        push_point(item.p, bad, item.support);
      }
      push_point(b, i + 1);
    }
  }
  if (pts.size() > 2 && !spec_.pole_placement.pin_vertices) {
    constexpr double kAutoCollapseDistanceM = 1.5;
    constexpr double kAutoCollapseDistanceSq = kAutoCollapseDistanceM * kAutoCollapseDistanceM;
    auto is_auto_generic = [&](std::size_t index) {
      if (index >= guide_by_local.size() || index >= support_by_local.size()) {
        return false;
      }
      const std::size_t guide_index = guide_by_local[index];
      return guide_index != bad && support_by_local[index] == SupportKind::kPole &&
             spec_by_point.find(guide_index) == spec_by_point.end();
    };
    auto is_endpoint = [&](std::size_t index) {
      const std::size_t guide_index = guide_by_local[index];
      return guide_index == 0 || guide_index + 1 == guide.size();
    };
    std::vector<bool> drop(pts.size(), false);
    for (std::size_t i = 0; i + 1 < pts.size(); ++i) {
      if (drop[i] || !is_auto_generic(i) || !is_auto_generic(i + 1)) {
        continue;
      }
      const Vec3d d = pts[i + 1] - pts[i];
      const double dist_sq = d.x * d.x + d.y * d.y + d.z * d.z;
      if (dist_sq <= 1e-18 || dist_sq >= kAutoCollapseDistanceSq) {
        continue;
      }
      const bool a_endpoint = is_endpoint(i);
      const bool b_endpoint = is_endpoint(i + 1);
      if (a_endpoint && b_endpoint) {
        continue;
      }
      drop[a_endpoint ? i + 1 : i] = true;
    }
    if (std::any_of(drop.begin(), drop.end(), [](bool item) { return item; })) {
      std::vector<Vec3d> kept_pts{};
      std::vector<std::size_t> kept_guide{};
      std::vector<SupportKind> kept_support{};
      kept_pts.reserve(pts.size());
      kept_guide.reserve(guide_by_local.size());
      kept_support.reserve(support_by_local.size());
      for (std::size_t i = 0; i < pts.size(); ++i) {
        if (drop[i]) {
          continue;
        }
        kept_pts.push_back(pts[i]);
        kept_guide.push_back(guide_by_local[i]);
        kept_support.push_back(support_by_local[i]);
      }
      pts = std::move(kept_pts);
      guide_by_local = std::move(kept_guide);
      support_by_local = std::move(kept_support);
    }
  }
  std::vector<std::size_t> local_by_guide(guide.size(), bad);
  for (std::size_t i = 0; i < guide_by_local.size(); ++i) {
    if (guide_by_local[i] != bad && guide_by_local[i] < local_by_guide.size()) {
      local_by_guide[guide_by_local[i]] = i;
    }
  }
  local_by_input_.assign(input.size(), bad);
  for (std::size_t i = 0; i < input.size(); ++i) {
    const std::size_t guide_index = (spec_.direction_mode == PathDirectionMode::kReverse) ? input.size() - 1 - i : i;
    local_by_input_[i] = guide_index < local_by_guide.size() ? local_by_guide[guide_index] : bad;
  }
  g_.nodes.reserve(pts.size());
  for (std::size_t i = 0; i < pts.size(); ++i) {
    node n{};
    n.id = i;
    n.pos = pts[i];
    n.support = support_by_local[i];
    n.on_route = true;
    const std::size_t guide_index = guide_by_local[i];
    if (guide_index != bad) {
      const bool endpoint = guide_index == 0 || guide_index + 1 == guide.size();
      n.pinned = spec_.pole_placement.pin_vertices || (spec_.pole_placement.pin_endpoints && endpoint);
    }
    if (guide_index != bad) {
      const auto spec_it = spec_by_point.find(guide_index);
      if (spec_it == spec_by_point.end()) {
        g_.nodes.push_back(n);
        continue;
      }
      if (spec_it->second->has_tangent_hint) {
        Vec3d tangent = spec_it->second->tangent_hint;
        (void)NormalizeXY(&tangent);
        n.has_tangent = true;
        n.tangent = tangent;
      }
      n.explicit_support = true;
      n.support = spec_it->second->support_kind;
      if (n.support == SupportKind::kMidair || n.support == SupportKind::kExternal ||
          n.support == SupportKind::kGround) {
        if (spec_it->second->node_id == kInvalidObjectId) {
          g_.nodes.push_back(n);
          continue;
        }
        const SavedBackboneNode* saved = state_.view().backbone_node(spec_it->second->node_id);
        if (saved != nullptr && saved->pole_id == kInvalidObjectId && saved->support_kind == n.support) {
          n.saved = saved->node_id;
          n.pos = saved->position;
          n.is_new = false;
          n.has_source_edge = saved->has_source_edge;
          n.source_edge_node_a = saved->source_edge_node_a;
          n.source_edge_node_b = saved->source_edge_node_b;
          n.source_edge_t = saved->source_edge_t;
          n.bundle_modes = saved->bundle_modes;
          g_.nodes.push_back(n);
          continue;
        }
        const SupportNode* pending = state_.view().pending_support_node(spec_it->second->node_id);
        if (pending != nullptr && pending->support_kind == n.support) {
          n.pos = pending->position;
          n.bundle_modes = pending->bundle_modes;
          n.has_source_edge = pending->has_source_edge;
          n.source_edge_node_a = pending->source_edge_node_a_id;
          n.source_edge_node_b = pending->source_edge_node_b_id;
          n.source_edge_t = pending->source_edge_t;
          const SavedBackboneNode* resolved = nullptr;
          if (pending->saved_backbone_node_id != kInvalidObjectId) {
            resolved = state_.view().backbone_node(pending->saved_backbone_node_id);
          }
          if (resolved == nullptr) {
            resolved = saved_source_node_for(state_, *pending);
          }
          if (resolved != nullptr && resolved->pole_id == kInvalidObjectId && resolved->support_kind == n.support) {
            n.saved = resolved->node_id;
            n.pos = resolved->position;
            n.is_new = false;
            n.has_source_edge = resolved->has_source_edge;
            n.source_edge_node_a = resolved->source_edge_node_a;
            n.source_edge_node_b = resolved->source_edge_node_b;
            n.source_edge_t = resolved->source_edge_t;
          }
          if (pending->has_tangent_hint) {
            Vec3d tangent = pending->tangent_hint;
            if (NormalizeXY(&tangent)) {
              n.has_tangent = true;
              n.tangent = tangent;
            }
          }
          g_.nodes.push_back(n);
          continue;
        }
        if (saved == nullptr || saved->pole_id != kInvalidObjectId || saved->support_kind != n.support) {
          out.error = (n.support == SupportKind::kMidair) ? "backbone unsupported: saved midair node not found"
                      : (n.support == SupportKind::kExternal)
                          ? "backbone unsupported: saved external node not found"
                          : "backbone unsupported: saved ground node not found";
          return out;
        }
      }
      if (spec_it->second->node_id == kInvalidObjectId) {
        g_.nodes.push_back(n);
        continue;
      }
      const SupportNode* pending_pole = state_.view().pending_support_node(spec_it->second->node_id);
      ObjectId pole_id = spec_it->second->node_id;
      if (pending_pole != nullptr && pending_pole->support_kind == SupportKind::kPole &&
          pending_pole->pole_id != kInvalidObjectId) {
        pole_id = pending_pole->pole_id;
        n.bundle_modes = pending_pole->bundle_modes;
      }
      const Pole* pole = state_.view().poles().find(pole_id);
      if (pole == nullptr) {
        out.error = "backbone unsupported: node spec pole not found";
        return out;
      }
      n.pole = pole->id;
      n.pos = pole->world_transform.position;
      if (pending_pole != nullptr && pending_pole->saved_backbone_node_id != kInvalidObjectId) {
        n.saved = pending_pole->saved_backbone_node_id;
      } else if (const SavedBackboneNode* saved = state_.view().backbone_node_for_pole(pole->id); saved != nullptr) {
        n.saved = saved->node_id;
      } else {
        out.error = "backbone unsupported: saved backbone graph missing for existing pole";
        return out;
      }
      n.is_new = false;
    }
    g_.nodes.push_back(n);
  }
  g_.links.reserve(pts.size() > 0 ? pts.size() - 1 : 0);
  for (std::size_t i = 0; i + 1 < pts.size(); ++i) {
    link edge{};
    edge.id = i;
    edge.a = i;
    edge.b = i + 1;
    edge.route = 0;
    edge.order = i;
    edge.dir = g_.nodes[i + 1].pos - g_.nodes[i].pos;
    edge.is_new = true;
    g_.links.push_back(edge);
  }
  std::unordered_map<ObjectId, std::size_t> local_by_saved{};
  for (const node& n : g_.nodes) {
    if (n.saved != kInvalidObjectId) {
      local_by_saved[n.saved] = n.id;
    }
  }
  auto has_requested_saved_pair = [&](ObjectId a, ObjectId b) {
    if (a == kInvalidObjectId || b == kInvalidObjectId) {
      return false;
    }
    const ObjectId lo = std::min(a, b);
    const ObjectId hi = std::max(a, b);
    for (const link& edge : g_.links) {
      if (!edge.is_new || edge.a >= g_.nodes.size() || edge.b >= g_.nodes.size()) {
        continue;
      }
      const ObjectId ea = g_.nodes[edge.a].saved;
      const ObjectId eb = g_.nodes[edge.b].saved;
      if (ea == kInvalidObjectId || eb == kInvalidObjectId) {
        continue;
      }
      if (std::min(ea, eb) == lo && std::max(ea, eb) == hi) {
        return true;
      }
    }
    return false;
  };
  auto local_for_saved = [&](ObjectId saved_node_id) -> std::size_t {
    if (const auto it = local_by_saved.find(saved_node_id); it != local_by_saved.end()) {
      return it->second;
    }
    const SavedBackboneNode* saved = state_.view().backbone_node(saved_node_id);
    if (saved == nullptr) {
      return bad;
    }
    node n{};
    n.id = g_.nodes.size();
    n.pos = saved->position;
    n.support = saved->support_kind;
    n.pole = saved->pole_id;
    n.saved = saved->node_id;
    n.is_new = false;
    n.has_source_edge = saved->has_source_edge;
    n.source_edge_node_a = saved->source_edge_node_a;
    n.source_edge_node_b = saved->source_edge_node_b;
    n.source_edge_t = saved->source_edge_t;
    n.bundle_modes = saved->bundle_modes;
    g_.nodes.push_back(n);
    local_by_saved[n.saved] = n.id;
    return n.id;
  };
  std::unordered_set<ObjectId> context_edges{};
  const std::size_t route_node_count = pts.size();
  for (std::size_t i = 0; i < route_node_count && i < g_.nodes.size(); ++i) {
    // local_for_saved may append context nodes and reallocate g_.nodes.
    const node n = g_.nodes[i];
    if (n.saved == kInvalidObjectId) {
      continue;
    }
    const auto incident_it = state_.view().backbone_index().node_edges.find(n.saved);
    if (incident_it == state_.view().backbone_index().node_edges.end()) {
      continue;
    }
    for (ObjectId edge_id : incident_it->second) {
      if (context_edges.contains(edge_id)) {
        continue;
      }
      const SavedBackboneEdge* saved = state_.view().backbone_edge(edge_id);
      if (saved == nullptr) {
        continue;
      }
      if (has_requested_saved_pair(saved->node_a, saved->node_b)) {
        continue;
      }
      std::size_t a = local_for_saved(saved->node_a);
      std::size_t b = local_for_saved(saved->node_b);
      if (a == bad || b == bad || a == b) {
        continue;
      }
      bool reversed_context = false;
      const bool has_open_and_pair =
          has_saved_open_row_at_node(state_, n.saved) && has_saved_pair_row_at_node(state_, n.saved);
      const bool preserve_open_direction =
          has_open_and_pair && has_saved_pair_row_for_edge(state_, n.saved, saved->edge_id);
      // A route endpoint with exactly one saved incident edge is a continuation candidate.
      // Orient that context edge toward the route start or away from the route end so pairs
      // can decide continuity without changing saved edge direction or identity.
      if (!preserve_open_direction && ((i == 0 && b != n.id) || (i + 1 == route_node_count && a != n.id))) {
        std::swap(a, b);
        reversed_context = true;
      }
      link edge{};
      edge.id = g_.links.size();
      edge.a = a;
      edge.b = b;
      edge.route = route_node_count + g_.links.size() + 1;
      edge.order = saved->order;
      edge.dir = saved->dir;
      if (reversed_context) {
        edge.dir = ScaleVec(edge.dir, -1.0);
      }
      edge.saved = saved->edge_id;
      edge.is_new = false;
      g_.links.push_back(edge);
      context_edges.insert(edge_id);
    }
  }
  const std::size_t source_context_node_count = g_.nodes.size();
  for (std::size_t node_index = 0; node_index < source_context_node_count; ++node_index) {
    const node n = g_.nodes[node_index];
    const ObjectId source_a = saved_node_id_for(state_, n.source_edge_node_a);
    const ObjectId source_b = saved_node_id_for(state_, n.source_edge_node_b);
    if (!n.has_source_edge || source_a == kInvalidObjectId || source_b == kInvalidObjectId ||
        source_a == source_b) {
      continue;
    }
    const std::size_t a = local_for_saved(source_a);
    const std::size_t b = local_for_saved(source_b);
    if (a == bad || b == bad || n.id >= g_.nodes.size() || a == n.id || b == n.id) {
      out.error = "backbone unsupported: source edge context node is missing";
      return out;
    }
    const BackboneEdgeKey key{std::min(source_a, source_b), std::max(source_a, source_b)};
    const auto edge_it = state_.view().backbone_index().edge_by_nodes.find(key);
    if (edge_it == state_.view().backbone_index().edge_by_nodes.end()) {
      out.error = "backbone unsupported: source edge context is missing";
      return out;
    }
    const ObjectId edge_id = edge_it->second;
    const SavedBackboneEdge* saved = state_.view().backbone_edge(edge_id);
    if (saved == nullptr) {
      out.error = "backbone unsupported: source edge context is missing";
      return out;
    }
    const std::size_t source_route = g_.links.size() + 1;
    auto add_source_context = [&](std::size_t from, std::size_t to, std::size_t order) {
      link edge{};
      edge.id = g_.links.size();
      edge.a = from;
      edge.b = to;
      edge.route = source_route;
      edge.order = order;
      edge.dir = g_.nodes[to].pos - g_.nodes[from].pos;
      edge.saved = edge_id;
      edge.is_new = false;
      g_.links.push_back(edge);
    };
    add_source_context(a, n.id, 0);
    add_source_context(n.id, b, 1);
  }
  active_bundle_indices_.reserve(spec_.bundles.size());
  for (std::size_t bundle_index = 0; bundle_index < spec_.bundles.size(); ++bundle_index) {
    bool allowed = true;
    const BundleTemplateId bundle_template_id = spec_.bundles[bundle_index].bundle_template_id;
    for (const node& n : g_.nodes) {
      if (!n.on_route) {
        continue;
      }
      const auto bundle_mode = std::find_if(n.bundle_modes.begin(), n.bundle_modes.end(),
                                            [&](const SupportNodeBundleMode& mode) {
                                              return mode.bundle_template_id == bundle_template_id;
                                            });
      if (!n.bundle_modes.empty() && bundle_mode == n.bundle_modes.end()) {
        allowed = false;
        break;
      }
      if (bundle_mode != n.bundle_modes.end() && bundle_mode->mode == BundleNodeMode::kNotPresent) {
        allowed = false;
        break;
      }
    }
    if (allowed) {
      active_bundle_indices_.push_back(bundle_index);
    }
  }
  out.value = true;
  out.ok = true;
  return out;
}

EditResult<bool> pipeline::check() const {
  if (g_.nodes.size() < 2 || g_.links.empty()) {
    return unsupported("path needs at least two points");
  }
  if (spec_.bundles.empty()) {
    return unsupported("at least one bundle is required");
  }
  for (const BackboneSpec::NodeBundleModeSpec& mode : spec_.node_bundle_modes) {
    if (mode.point_index >= spec_.path.polyline.size()) {
      return unsupported("node bundle mode point index is out of range");
    }
    const auto bundle_it = std::find_if(spec_.bundles.begin(), spec_.bundles.end(), [&](const BackboneBundleSpec& bundle) {
      return bundle.bundle_template_id == mode.bundle_template_id;
    });
    if (bundle_it == spec_.bundles.end()) {
      return unsupported("node bundle mode references missing bundle");
    }
    if (mode.mode == BundleNodeMode::kPassThrough) {
      const std::size_t local = this->local(mode.point_index);
      if (local == bad || local >= g_.nodes.size()) {
        return unsupported("pass-through node mode point is missing");
      }
      if (has_current_route_pair_at(g_, local)) {
        continue;
      }
      if (g_.nodes[local].saved == kInvalidObjectId) {
        const std::size_t source_context_incidents =
            g_.nodes[local].has_source_edge ? context_incident_count_at(g_, local) : 0U;
        if (source_context_incidents + current_route_incident_count_at(g_, local) < 2) {
          return unsupported("pass-through node mode requires current route pair or saved source context");
        }
        continue;
      }
      const auto incident_it = state_.view().backbone_index().node_edges.find(g_.nodes[local].saved);
      const std::size_t saved_incidents =
          (incident_it == state_.view().backbone_index().node_edges.end()) ? 0U : incident_it->second.size();
      if (saved_incidents + current_route_incident_count_at(g_, local) < 2) {
        return unsupported("pass-through node mode requires current route pair or saved junction context");
      }
      continue;
    }
    if (mode.mode != BundleNodeMode::kNotPresent) {
      return unsupported("node bundle mode is not supported");
    }
  }
  if (!route_clear_of_avoid_points(g_, spec_.constraints.avoid_points, spec_.constraints.avoid_radius_m)) {
    return unsupported("avoid routing cannot satisfy the requested constraints");
  }
  if (needs_pole_type(g_)) {
    EditResult<PoleTypeId> resolved_type = pole_type_for(state_, spec_);
    if (!resolved_type.ok) {
      EditResult<bool> failed{};
      failed.error = resolved_type.error;
      return failed;
    }
  }
  for (const BackboneBundleSpec& spec : spec_.bundles) {
    EditResult<spec_view> checked = view_for(state_, spec);
    if (!checked.ok) {
      EditResult<bool> failed{};
      failed.error = checked.error;
      return failed;
    }
  }
  EditResult<bool> bands = check_port_bands(state_, g_, spec_, active_bundle_indices_);
  if (!bands.ok) {
    return bands;
  }
  EditResult<bool> out{};
  out.value = true;
  out.ok = true;
  return out;
}

EditResult<pairs> pipeline::make(const graph& made) const {
  if (made.nodes.size() < 2 || made.links.empty()) {
    return unsupported_pairs("path needs at least two points");
  }

  EditResult<pairs> out{};
  out.value.links = made.links;
  std::vector<std::vector<std::size_t>> in(made.nodes.size());
  std::vector<std::vector<std::size_t>> out_links(made.nodes.size());
  for (link& edge : out.value.links) {
    if (edge.a >= made.nodes.size() || edge.b >= made.nodes.size()) {
      return unsupported_pairs("link endpoint is out of range");
    }
    if (!NormalizeXY(&edge.dir)) {
      return unsupported_pairs("zero length link");
    }
    out_links[edge.a].push_back(edge.id);
    in[edge.b].push_back(edge.id);
  }

  std::vector<bool> aused(out.value.links.size(), false);
  std::vector<bool> bused(out.value.links.size(), false);
  for (const node& n : made.nodes) {
    std::vector<std::size_t> incoming = in[n.id];
    std::vector<std::size_t> outgoing = out_links[n.id];
    auto link_less = [&](std::size_t lhs, std::size_t rhs) {
      const link& a = out.value.links[lhs];
      const link& b = out.value.links[rhs];
      if (a.route != b.route) {
        return a.route < b.route;
      }
      if (a.order != b.order) {
        return a.order < b.order;
      }
      return a.id < b.id;
    };
    std::sort(incoming.begin(), incoming.end(), link_less);
    std::sort(outgoing.begin(), outgoing.end(), link_less);
    auto is_continuation = [&](const link& a, const link& b) {
      const bool same_route = a.route == b.route && a.order + 1 == b.order;
      const bool saved_pair = is_saved_pair_continuation(state_, n, a, b);
      const bool terminal_extension = is_promoted_open_continuation(state_, n, a, b);
      return same_route || saved_pair || terminal_extension;
    };

    for (std::size_t right : outgoing) {
      int candidates = 0;
      int promoted_candidates = 0;
      for (std::size_t left : incoming) {
        const link& a = out.value.links[left];
        const link& b = out.value.links[right];
        if (is_continuation(a, b)) {
          ++candidates;
          promoted_candidates += is_promoted_open_continuation(state_, n, a, b) ? 1 : 0;
        }
      }
      if (candidates > 1) {
        return unsupported_pairs(promoted_candidates > 1 ? "ambiguous promoted open edge"
                                                         : "route continuity is ambiguous");
      }
    }

    for (std::size_t left : incoming) {
      if (bused[left]) {
        continue;
      }
      std::size_t matched = bad;
      for (std::size_t right : outgoing) {
        if (aused[right]) {
          continue;
        }
        const link& a = out.value.links[left];
        const link& b = out.value.links[right];
        if (!is_continuation(a, b)) {
          continue;
        }
        if (matched != bad) {
          const link& previous = out.value.links[matched];
          const bool promoted = is_promoted_open_continuation(state_, n, a, b) ||
                                is_promoted_open_continuation(state_, n, a, previous);
          return unsupported_pairs(promoted ? "ambiguous promoted open edge"
                                            : "route continuity is ambiguous");
        }
        matched = right;
      }
      if (matched == bad) {
        continue;
      }
      const link& a = out.value.links[left];
      const link& b = out.value.links[matched];
      const double interior_angle = interior_angle_deg(a, b);
      if (interior_angle <= 1e-6) {
        return unsupported_pairs("zero angle route reversal");
      }
      if (interior_angle <= kSharpCornerInteriorAngleMaxDeg + 1e-6) {
        bused[left] = true;
        aused[matched] = true;
        if (out.value.links[left].brow != bad || out.value.links[matched].arow != bad) {
          return unsupported_pairs("incident already used");
        }
        const std::size_t left_row =
            add_open(&out.value, n.id, left, ComputeLateralAxis(out.value.links[left].dir));
        const std::size_t right_row =
            add_open(&out.value, n.id, matched, ComputeLateralAxis(out.value.links[matched].dir));
        out.value.links[left].brow = left_row;
        out.value.links[matched].arow = right_row;
        Vec3d node_forward = ScaleVec(a.dir, -1.0) + b.dir;
        if (!NormalizeXY(&node_forward)) {
          return unsupported_pairs("zero length sharp corner bisector");
        }
        out.value.jumpers.push_back(jumper{n.id, left_row, right_row, interior_angle, node_forward});
        continue;
      }
      const bool canonical_saved_pair = is_saved_pair_continuation(state_, n, a, b) ||
                                        is_promoted_open_continuation(state_, n, a, b);
      Vec3d pair_axis = canonical_saved_pair
                            ? canonical_pair_axis(out.value.links[left], out.value.links[matched], n.id)
                            : out.value.links[left].dir + out.value.links[matched].dir;
      if (!NormalizeXY(&pair_axis)) {
        return unsupported_pairs("zero length pair tangent sum");
      }
      bused[left] = true;
      aused[matched] = true;
      if (out.value.links[left].brow != bad || out.value.links[matched].arow != bad) {
        return unsupported_pairs("incident already used");
      }
      const std::size_t row_id = add_pair(&out.value, n.id, left, matched, ComputeLateralAxis(pair_axis));
      out.value.links[left].brow = row_id;
      out.value.links[matched].arow = row_id;
    }

    for (std::size_t link_id : outgoing) {
      if (aused[link_id]) {
        continue;
      }
      aused[link_id] = true;
      if (out.value.links[link_id].arow != bad) {
        return unsupported_pairs("incident already used");
      }
      out.value.links[link_id].arow =
          add_open(&out.value, n.id, link_id, ComputeLateralAxis(out.value.links[link_id].dir));
    }
    for (std::size_t link_id : incoming) {
      if (bused[link_id]) {
        continue;
      }
      bused[link_id] = true;
      if (out.value.links[link_id].brow != bad) {
        return unsupported_pairs("incident already used");
      }
      out.value.links[link_id].brow =
          add_open(&out.value, n.id, link_id, ComputeLateralAxis(out.value.links[link_id].dir));
    }
  }

  for (const link& edge : out.value.links) {
    if (edge.arow == bad || edge.brow == bad) {
      return unsupported_pairs("link row is unresolved");
    }
  }
  align_terminal_open_rows_to_pairs(&out.value);
  out.ok = true;
  return out;
}

EditResult<bool> pipeline::check(const pairs& ps) const {
  for (const node& source : g_.nodes) {
    if (!source.has_source_edge || !ownerless_support(source.support)) {
      continue;
    }
    for (std::size_t spec_index : active_bundle_indices_) {
      if (spec_index >= spec_.bundles.size()) {
        return unsupported("active bundle index is invalid");
      }
      const BackboneBundleSpec& bundle_spec = spec_.bundles[spec_index];
      EditResult<spec_view> v = view_for(state_, bundle_spec);
      if (!v.ok) {
        EditResult<bool> failed{};
        failed.error = v.error;
        return failed;
      }
      for (int lane = 0; lane < v.value.count; ++lane) {
        const SourceEdgeProjectionRef ref =
            source_projection_for(state_, source, spec_.bundles[spec_index], static_cast<std::size_t>(lane));
        if (source_span_binding_for(state_, ref) == nullptr) {
          return unsupported("source edge attachment is missing");
        }
      }
    }
  }
  std::vector<ObjectId> node_id_by_local(g_.nodes.size(), kInvalidObjectId);
  for (std::size_t i = 0; i < g_.nodes.size(); ++i) {
    node_id_by_local[i] = g_.nodes[i].saved;
    if (g_.nodes[i].saved != kInvalidObjectId && state_.view().backbone_node(g_.nodes[i].saved) == nullptr) {
      return unsupported("saved backbone node is missing");
    }
  }
  std::vector<SavedBackboneEdgeRef> edge_by_link(ps.links.size());
  for (const link& edge : ps.links) {
    if (edge.id >= edge_by_link.size()) {
      continue;
    }
    edge_by_link[edge.id] = ref_for_existing_edge(state_, g_, edge);
    if (edge.saved != kInvalidObjectId && edge_by_link[edge.id].edge_id == kInvalidObjectId) {
      EditResult<bool> failed{};
      failed.error = "backbone graph: context link saved edge missing";
      return failed;
    }
  }
  std::vector<bool> active_rows(ps.rows.size(), false);
  for (const link& edge : ps.links) {
    if (!edge.is_new) {
      continue;
    }
    if (edge.arow < active_rows.size()) {
      active_rows[edge.arow] = true;
    }
    if (edge.brow < active_rows.size()) {
      active_rows[edge.brow] = true;
    }
  }
  for (const row& r : ps.rows) {
    if (r.node >= g_.nodes.size()) {
      return unsupported("row node missing");
    }
    if (r.id >= active_rows.size() || !active_rows[r.id]) {
      continue;
    }
    const bool ownerless = ownerless_support(g_.nodes[r.node].support);
    ObjectId pole_id = kInvalidObjectId;
    const PoleTypeDefinition* new_pole_type = nullptr;
    if (!ownerless && g_.nodes[r.node].is_new) {
      EditResult<PoleTypeId> resolved_type = pole_type_for(state_, spec_);
      if (!resolved_type.ok) {
        EditResult<bool> failed{};
        failed.error = resolved_type.error;
        return failed;
      }
      const auto type_it = state_.view().pole_types().find(resolved_type.value);
      if (type_it == state_.view().pole_types().end()) {
        return unsupported("pole type missing");
      }
      new_pole_type = &type_it->second;
    } else if (!ownerless) {
      pole_id = g_.nodes[r.node].pole;
      if (pole_id == kInvalidObjectId || state_.view().poles().find(pole_id) == nullptr) {
        return unsupported("active row pole missing");
      }
    }
    for (std::size_t bundle_index = 0; bundle_index < active_bundle_indices_.size(); ++bundle_index) {
      const std::size_t spec_index = active_bundle_indices_[bundle_index];
      if (spec_index >= spec_.bundles.size()) {
        return unsupported("active bundle index is invalid");
      }
      const BackboneBundleSpec& bundle_spec = spec_.bundles[spec_index];
      EditResult<spec_view> v = view_for(state_, bundle_spec);
      if (!v.ok) {
        EditResult<bool> failed{};
        failed.error = v.error;
        return failed;
      }
      std::vector<PortPlacementBand> bands(static_cast<std::size_t>(v.value.count));
      if (!ownerless) {
        EditResult<std::vector<PortPlacementBand>> resolved_bands =
            new_pole_type == nullptr
                ? bands_for(state_, pole_id, v.value)
                : SelectPortPlacementBands(*new_pole_type, v.value.tmpl->category, v.value.layer, v.value.count);
        if (!resolved_bands.ok) {
          EditResult<bool> failed{};
          failed.error = resolved_bands.error;
          return failed;
        }
        bands = std::move(resolved_bands.value);
      }
      trow preflight_row{};
      preflight_row.row = r.id;
      preflight_row.node = r.node;
      preflight_row.source = r.source;
      preflight_row.axis = r.axis;
      preflight_row.pole = pole_id;
      const SavedBackboneRowKey row_key = key_for(ps, preflight_row, node_id_by_local, edge_by_link);
      for (int lane = 0; lane < v.value.count; ++lane) {
        const PortPlacementBand& band = bands[static_cast<std::size_t>(lane)];
        const port_scope scope{bundle_spec.bundle_template_id, bundle_spec.existing_bundle_id,
                               bundle_spec.placement_key,
                               PortKindForCategory(v.value.tmpl->category),
                               PortLayerForSpanLayer(v.value.layer), band.band_id};
        if (ownerless && g_.nodes[r.node].has_source_edge) {
          const SourceEdgeProjectionRef ref = source_projection_for(
              state_, g_.nodes[r.node], bundle_spec, static_cast<std::size_t>(lane));
          if (source_span_binding_for(state_, ref) == nullptr) {
            return unsupported("source edge attachment is missing");
          }
        }
        EditResult<ObjectId> resolved =
            resolve_port_binding(state_, pole_id, row_key, static_cast<std::size_t>(lane), scope);
        if (!resolved.ok) {
          EditResult<bool> failed{};
          failed.error = resolved.error;
          return failed;
        }
      }
    }
  }
  for (const link& edge : ps.links) {
    if (!edge.is_new) {
      continue;
    }
    if (edge.saved != kInvalidObjectId) {
      continue;
    }
    const ObjectId edge_id = saved_edge_for(state_, g_, edge);
    if (edge_id == kInvalidObjectId) {
      continue;
    }
    const auto bundles_it = state_.view().backbone_index().edge_bundles.find(edge_id);
    if (bundles_it == state_.view().backbone_index().edge_bundles.end()) {
      continue;
    }
    for (std::size_t spec_index : active_bundle_indices_) {
      const BackboneBundleSpec& spec = spec_.bundles[spec_index];
      for (ObjectId edge_bundle_id : bundles_it->second) {
        const SavedBackboneEdgeBundle* edge_bundle = state_.view().backbone_edge_bundle(edge_bundle_id);
        if (edge_bundle == nullptr) {
          continue;
        }
        const Bundle* bundle = state_.view().bundles().find(edge_bundle->bundle_id);
        if (bundle != nullptr && bundle->bundle_template_id == spec.bundle_template_id) {
          const auto spans_it = state_.view().backbone_index().edge_bundle_span_bindings.find(edge_bundle_id);
          if (spans_it != state_.view().backbone_index().edge_bundle_span_bindings.end()) {
            const SavedBackboneGraph& graph = state_.view().backbone();
            EditResult<spec_view> v = view_for(state_, spec);
            if (!v.ok) {
              EditResult<bool> error{};
              error.error = v.error;
              return error;
            }
            for (std::size_t index : spans_it->second) {
              if (index >= graph.span_bindings.size()) {
                continue;
              }
              const SavedBackboneSpanBinding& binding = graph.span_bindings[index];
              if (binding.lane_index < static_cast<std::size_t>(v.value.count)) {
                return unsupported("duplicate saved span binding");
              }
            }
          }
          return unsupported("duplicate saved edge bundle");
        }
      }
    }
  }
  EditResult<bool> out{};
  out.value = true;
  out.ok = true;
  return out;
}

EditResult<std::vector<pipeline::PromotionPlanEntry>> pipeline::plan_promotions(const pairs& ps) const {
  EditResult<std::vector<PromotionPlanEntry>> out{};

  std::vector<ObjectId> node_id_by_local(g_.nodes.size(), kInvalidObjectId);
  for (std::size_t i = 0; i < g_.nodes.size(); ++i) {
    node_id_by_local[i] = g_.nodes[i].saved;
  }

  auto edge_bundle_for_spec = [&](ObjectId edge_id, const BackboneBundleSpec& spec,
                                  ObjectId* bundle_id) -> EditResult<ObjectId> {
    EditResult<ObjectId> found{};
    found.value = kInvalidObjectId;
    found.ok = true;
    if (bundle_id != nullptr) {
      *bundle_id = kInvalidObjectId;
    }
    const auto bundles_it = state_.view().backbone_index().edge_bundles.find(edge_id);
    if (bundles_it == state_.view().backbone_index().edge_bundles.end()) {
      return found;
    }
    for (ObjectId edge_bundle_id : bundles_it->second) {
      const SavedBackboneEdgeBundle* edge_bundle = state_.view().backbone_edge_bundle(edge_bundle_id);
      const Bundle* bundle = edge_bundle == nullptr ? nullptr : state_.view().bundles().find(edge_bundle->bundle_id);
      if (bundle == nullptr || bundle->bundle_template_id != spec.bundle_template_id) {
        continue;
      }
      if (spec.placement_key != 0 && bundle->placement_key != 0 &&
          bundle->placement_key != spec.placement_key) {
        continue;
      }
      if (found.value != kInvalidObjectId) {
        found.ok = false;
        found.error = "backbone unsupported: ambiguous promoted bundle placement";
        return found;
      }
      found.value = edge_bundle_id;
      if (bundle_id != nullptr) {
        *bundle_id = bundle->id;
      }
    }
    return found;
  };

  auto binding_for = [&](const SavedBackboneRowKey& open_key, ObjectId edge_bundle_id,
                         std::size_t lane_index) -> EditResult<const SavedBackbonePortBinding*> {
    EditResult<const SavedBackbonePortBinding*> found{};
    found.value = nullptr;
    found.ok = true;
    for (const SavedBackbonePortBinding* binding :
         state_.view().backbone_port_bindings_for_row(open_key, lane_index)) {
      if (binding == nullptr || binding->edge_bundle_id != edge_bundle_id) {
        continue;
      }
      if (found.value != nullptr) {
        found.ok = false;
        found.error = "backbone unsupported: ambiguous promoted open row binding";
        return found;
      }
      found.value = binding;
    }
    return found;
  };

  for (const row& r : ps.rows) {
    if (r.source.is_open || r.source.id >= ps.joins.size() || r.node >= node_id_by_local.size()) {
      continue;
    }
    const pair& join = ps.joins[r.source.id];
    const link* left = join.left < ps.links.size() ? &ps.links[join.left] : nullptr;
    const link* right = join.right < ps.links.size() ? &ps.links[join.right] : nullptr;
    if (left == nullptr || right == nullptr || left->is_new == right->is_new) {
      continue;
    }
    const link* existing = left->is_new ? right : left;
    if (existing->saved == kInvalidObjectId ||
        !has_saved_open_row_for_edge(state_, node_id_by_local[r.node], existing->saved)) {
      continue;
    }

    SavedBackboneRowKey open_key{};
    open_key.node_id = node_id_by_local[r.node];
    open_key.source_is_open = true;
    open_key.source_edge_a = existing->saved;

    for (std::size_t spec_index : active_bundle_indices_) {
      if (spec_index >= spec_.bundles.size()) {
        out.error = "backbone unsupported: active bundle index is invalid";
        return out;
      }
      const BackboneBundleSpec& spec = spec_.bundles[spec_index];
      EditResult<spec_view> v = view_for(state_, spec);
      if (!v.ok) {
        out.error = v.error;
        return out;
      }
      ObjectId bundle_id = kInvalidObjectId;
      EditResult<ObjectId> edge_bundle = edge_bundle_for_spec(existing->saved, spec, &bundle_id);
      if (!edge_bundle.ok) {
        out.error = edge_bundle.error;
        return out;
      }
      if (edge_bundle.value == kInvalidObjectId || bundle_id == kInvalidObjectId) {
        continue;
      }
      for (int lane = 0; lane < v.value.count; ++lane) {
        EditResult<const SavedBackbonePortBinding*> binding =
            binding_for(open_key, edge_bundle.value, static_cast<std::size_t>(lane));
        if (!binding.ok) {
          out.error = binding.error;
          return out;
        }
        if (binding.value == nullptr) {
          continue;
        }
        PromotionPlanEntry entry{};
        entry.row = r.id;
        entry.spec_index = spec_index;
        entry.lane_index = static_cast<std::size_t>(lane);
        entry.old_open_row_key = open_key;
        entry.existing_edge_bundle_id = edge_bundle.value;
        entry.existing_bundle_id = bundle_id;
        entry.placement_key = spec.placement_key;
        entry.port_id = binding.value->port_id;
        const auto duplicate = std::find_if(out.value.begin(), out.value.end(), [&](const PromotionPlanEntry& item) {
          return item.row == entry.row && item.spec_index == entry.spec_index &&
                 item.lane_index == entry.lane_index;
        });
        if (duplicate != out.value.end()) {
          out.error = "backbone unsupported: duplicate promoted pair placement";
          return out;
        }
        out.value.push_back(entry);
      }
    }
  }

  out.ok = true;
  return out;
}

EditResult<intent> pipeline::make(const pairs& ps) const {
  EditResult<intent> out{};
  std::vector<const BundleTemplate*> bundle_templates(active_bundle_indices_.size(), nullptr);
  for (std::size_t bundle = 0; bundle < active_bundle_indices_.size(); ++bundle) {
    const std::size_t spec_index = active_bundle_indices_[bundle];
    if (spec_index >= spec_.bundles.size()) {
      out.error = "backbone unsupported: active bundle index is invalid";
      return out;
    }
    const BackboneBundleSpec& bundle_spec = spec_.bundles[spec_index];
    const EditResult<spec_view> v = view_for(state_, bundle_spec);
    if (!v.ok || v.value.tmpl == nullptr) {
      out.error = v.ok ? "backbone unsupported: bundle template not found" : v.error;
      return out;
    }
    bundle_templates[bundle] = v.value.tmpl;
  }
  auto add_row_intent = [&](std::size_t row_id, std::size_t bundle, intent_reason reason) {
    const BundleTemplate* tmpl =
        bundle < bundle_templates.size() ? bundle_templates[bundle] : nullptr;
    const double endpoint_offset = tmpl == nullptr ? 0.0 : tmpl->branch_endpoint_offset_m;
    for (row_intent& item : out.value.rows) {
      if (item.row == row_id && item.bundle == bundle) {
        item.lower_required = true;
        item.endpoint_offset_m = endpoint_offset;
        if (item.reason == intent_reason::none) {
          item.reason = reason;
        }
        return;
      }
    }
    out.value.rows.push_back(
        row_intent{row_id, bundle, CurvePassMode::kPassThrough, true, reason, endpoint_offset});
  };
  for (const BackboneSpec::NodeBundleModeSpec& mode : spec_.node_bundle_modes) {
    if (mode.mode != BundleNodeMode::kPassThrough) {
      continue;
    }
    const std::size_t local = this->local(mode.point_index);
    if (local == bad || local >= g_.nodes.size()) {
      out.error = "backbone unsupported: pass-through node mode point is missing";
      return out;
    }
    const auto bundle_it = std::find_if(spec_.bundles.begin(), spec_.bundles.end(), [&](const BackboneBundleSpec& spec) {
      return spec.bundle_template_id == mode.bundle_template_id;
    });
    if (bundle_it == spec_.bundles.end()) {
      out.error = "backbone unsupported: pass-through bundle is missing";
      return out;
    }
    const std::size_t spec_index = static_cast<std::size_t>(std::distance(spec_.bundles.begin(), bundle_it));
    const auto active_it = std::find(active_bundle_indices_.begin(), active_bundle_indices_.end(), spec_index);
    if (active_it == active_bundle_indices_.end()) {
      out.error = "backbone unsupported: pass-through bundle is inactive";
      return out;
    }
    const std::size_t bundle = static_cast<std::size_t>(std::distance(active_bundle_indices_.begin(), active_it));
    std::vector<std::size_t> matches{};
    for (const link& edge : ps.links) {
      if (!edge.is_new) {
        continue;
      }
      if (edge.a == local && edge.arow != bad) {
        matches.push_back(edge.arow);
      }
      if (edge.b == local && edge.brow != bad) {
        matches.push_back(edge.brow);
      }
    }
    std::sort(matches.begin(), matches.end());
    matches.erase(std::unique(matches.begin(), matches.end()), matches.end());
    if (matches.size() != 1) {
      out.error = "backbone unsupported: pass-through target row is ambiguous";
      return out;
    }
    (void)bundle;
  }

  auto row_links = [&](const row& r) {
    std::vector<std::size_t> links{};
    if (r.source.is_open) {
      if (r.source.id < ps.opens.size()) {
        links.push_back(ps.opens[r.source.id].link);
      }
      return links;
    }
    if (r.source.id < ps.joins.size()) {
      links.push_back(ps.joins[r.source.id].left);
      links.push_back(ps.joins[r.source.id].right);
    }
    return links;
  };
  auto saved_edge_has_bundle = [&](ObjectId edge_id, BundleTemplateId bundle_template_id) {
    const auto bundles_it = state_.view().backbone_index().edge_bundles.find(edge_id);
    if (bundles_it == state_.view().backbone_index().edge_bundles.end()) {
      return false;
    }
    for (ObjectId edge_bundle_id : bundles_it->second) {
      const SavedBackboneEdgeBundle* edge_bundle = state_.view().backbone_edge_bundle(edge_bundle_id);
      const Bundle* bundle =
          edge_bundle == nullptr ? nullptr : state_.view().bundles().find(edge_bundle->bundle_id);
      if (bundle != nullptr && bundle->bundle_template_id == bundle_template_id) {
        return true;
      }
    }
    return false;
  };
  auto row_has_bundle = [&](const row& r, BundleTemplateId bundle_template_id) {
    for (std::size_t link_id : row_links(r)) {
      if (link_id >= ps.links.size()) {
        continue;
      }
      const link& edge = ps.links[link_id];
      if (edge.is_new || (edge.saved != kInvalidObjectId && saved_edge_has_bundle(edge.saved, bundle_template_id))) {
        return true;
      }
    }
    return false;
  };

  std::unordered_map<std::size_t, std::vector<std::size_t>> rows_by_node{};
  for (const row& r : ps.rows) {
    rows_by_node[r.node].push_back(r.id);
  }
  std::vector<bool> active(ps.rows.size(), false);
  for (const link& edge : ps.links) {
    if (!edge.is_new) {
      continue;
    }
    if (edge.arow < active.size()) {
      active[edge.arow] = true;
    }
    if (edge.brow < active.size()) {
      active[edge.brow] = true;
    }
  }
  for (const auto& item : rows_by_node) {
    if (item.second.size() < 2) {
      continue;
    }
    std::vector<std::size_t> active_rows{};
    for (std::size_t row_id : item.second) {
      if (row_id < active.size() && active[row_id]) {
        active_rows.push_back(row_id);
      }
    }
    if (active_rows.size() > 1) {
      const bool resolved_by_jumper = active_rows.size() == 2 &&
          std::any_of(ps.jumpers.begin(), ps.jumpers.end(), [&](const jumper& connection) {
            return (connection.row_a == active_rows[0] && connection.row_b == active_rows[1]) ||
                   (connection.row_a == active_rows[1] && connection.row_b == active_rows[0]);
          });
      if (!resolved_by_jumper) {
        out.error = "backbone unsupported: multiple generated rows conflict at one node";
        return out;
      }
    }
    if (active_rows.empty()) {
      continue;
    }
    const std::size_t selected_row = active_rows.front();
    for (std::size_t bundle = 0; bundle < active_bundle_indices_.size(); ++bundle) {
      const BundleTemplate* tmpl = bundle_templates[bundle];
      if (tmpl == nullptr || !tmpl->enable_branch_down_offset || tmpl->branch_endpoint_offset_m == 0.0) {
        continue;
      }
      bool peer_has_bundle = false;
      for (std::size_t peer_row : item.second) {
        if (peer_row == selected_row || peer_row >= ps.rows.size()) {
          continue;
        }
        peer_has_bundle = peer_has_bundle || row_has_bundle(ps.rows[peer_row], tmpl->id);
      }
      if (peer_has_bundle) {
        add_row_intent(selected_row, bundle, intent_reason::conflicting_rows);
      }
    }
  }
  out.ok = true;
  return out;
}

EditResult<groups> pipeline::make(const pairs& ps, const intent& intents) const {
  EditResult<groups> out{};
  std::unordered_map<std::size_t, std::vector<std::size_t>> rows_by_node{};
  for (const row& r : ps.rows) {
    rows_by_node[r.node].push_back(r.id);
  }
  std::unordered_map<std::size_t, int> order_by_row{};
  for (auto& item : rows_by_node) {
    std::vector<std::size_t>& rows = item.second;
    std::sort(rows.begin(), rows.end());
    for (std::size_t order = 0; order < rows.size(); ++order) {
      order_by_row[rows[order]] = static_cast<int>(order);
    }
  }
  for (const row_intent& item : intents.rows) {
    if (!item.lower_required) {
      continue;
    }
    if (item.row >= ps.rows.size() || item.bundle >= active_bundle_indices_.size()) {
      out.error = "backbone unsupported: support group row is invalid";
      return out;
    }
    group placed{};
    placed.id = out.value.items.size();
    placed.node = ps.rows[item.row].node;
    placed.row_members.push_back(group_member{item.row, item.bundle});
    placed.group_axis = ps.rows[item.row].axis;
    placed.vertical_order = order_by_row.contains(item.row) ? order_by_row[item.row] : 0;
    if (item.endpoint_offset_m == 0.0) {
      out.error = "backbone unsupported: support group offset policy is missing";
      return out;
    }
    placed.endpoint_offset_m = item.endpoint_offset_m;
    out.value.items.push_back(std::move(placed));
  }
  out.ok = true;
  return out;
}

EditResult<topo> pipeline::emit(const pairs& ps) {
  EditResult<topo> out{};
  topo made{};
  auto step = [&](EditResult<bool> result) -> bool {
    if (!result.ok) {
      out.error = result.error;
      return false;
    }
    return true;
  };
  if (!step(emit_poles(&made, ps, &out.change_set)) || !step(emit_bundles(&made, &out.change_set)) ||
      !step(emit_ports(&made, ps, &out.change_set)) || !step(emit_spans(&made, ps, &out.change_set))) {
    return out;
  }
  out.value = std::move(made);
  out.ok = true;
  return out;
}

EditResult<bool> pipeline::emit_poles(topo* made, const pairs& ps, ChangeSet* changes) {
  EditResult<bool> out{};
  if (made == nullptr || changes == nullptr) {
    out.error = "backbone topology: output missing";
    return out;
  }
  made->poles.reserve(g_.nodes.size());
  PoleTypeId pole_type = kInvalidPoleTypeId;
  if (needs_pole_type(g_)) {
    EditResult<PoleTypeId> resolved_type = pole_type_for(state_, spec_);
    if (!resolved_type.ok) {
      out.error = resolved_type.error;
      return out;
    }
    pole_type = resolved_type.value;
  }
  for (std::size_t i = 0; i < g_.nodes.size(); ++i) {
    if (g_.nodes[i].support == SupportKind::kMidair || g_.nodes[i].support == SupportKind::kExternal ||
        g_.nodes[i].support == SupportKind::kGround) {
      made->poles.push_back(kInvalidObjectId);
      continue;
    }
    if (!g_.nodes[i].is_new) {
      made->poles.push_back(g_.nodes[i].pole);
      continue;
    }
    Transformd tf{};
    tf.position = g_.nodes[i].pos;
    std::size_t prev_index = bad;
    std::size_t next_index = bad;
    for (const link& edge : g_.links) {
      if (!edge.is_new) {
        continue;
      }
      if (edge.b == i) {
        prev_index = edge.a;
      }
      if (edge.a == i) {
        next_index = edge.b;
      }
    }
    const Vec3d next = (next_index != bad && next_index < g_.nodes.size()) ? g_.nodes[next_index].pos : g_.nodes[i].pos;
    const Vec3d prev = (prev_index != bad && prev_index < g_.nodes.size()) ? g_.nodes[prev_index].pos : g_.nodes[i].pos;
    Vec3d dir = (next_index != bad) ? (next - g_.nodes[i].pos) : (g_.nodes[i].pos - prev);
    if (g_.nodes[i].has_tangent) {
      dir = g_.nodes[i].tangent;
    } else {
      const auto sharp = std::find_if(ps.jumpers.begin(), ps.jumpers.end(),
                                      [&](const jumper& connection) { return connection.node == i; });
      if (sharp != ps.jumpers.end()) {
        dir = sharp->node_forward;
      }
    }
    tf.rotation_euler_deg.z = YawDegFromXY(dir);
    EditResult<ObjectId> pole = state_.AddPole(tf, 10.0, "backbone-pole", PoleKind::kConcrete,
                                               g_.nodes[i].pinned ? PlacementMode::kManual : PlacementMode::kAuto);
    if (!pole.ok) {
      out.error = pole.error;
      return out;
    }
    add(*changes, pole.change_set);
    if (spec_.pole_placement.enable_tilt && spec_.pole_placement.max_tilt_deg > 0.0) {
      const pole_pull pull = pull_for_node(g_, i);
      EditResult<bool> tilted = state_.apply_pole_tilt_from_pull(pole.value, spec_.pole_placement.max_tilt_deg,
                                                                  pull.dir, pull.incident_count, changes);
      if (!tilted.ok) {
        out.error = tilted.error;
        return out;
      }
    }
    EditResult<ObjectId> typed = state_.ApplyPoleType(pole.value, pole_type);
    if (!typed.ok) {
      out.error = typed.error;
      return out;
    }
    add(*changes, typed.change_set);
    made->poles.push_back(pole.value);
    made->new_poles.push_back(pole.value);
  }

  out.value = true;
  out.ok = true;
  return out;
}

EditResult<bool> pipeline::emit_bundles(topo* made, ChangeSet* changes) {
  EditResult<bool> out{};
  if (made == nullptr || changes == nullptr) {
    out.error = "backbone topology: output missing";
    return out;
  }
  made->bundles.reserve(active_bundle_indices_.size());
  made->bundle_specs.reserve(active_bundle_indices_.size());
  for (std::size_t spec_index : active_bundle_indices_) {
    const BackboneBundleSpec& spec = spec_.bundles[spec_index];
    EditResult<spec_view> v = view_for(state_, spec);
    if (!v.ok) {
      out.error = v.error;
      return out;
    }
    ObjectId promoted_bundle = kInvalidObjectId;
    for (const PromotionPlanEntry& entry : promotion_plan_) {
      if (entry.spec_index != spec_index) {
        continue;
      }
      if (promoted_bundle != kInvalidObjectId && promoted_bundle != entry.existing_bundle_id) {
        out.error = "backbone unsupported: promoted placement maps to multiple bundles";
        return out;
      }
      promoted_bundle = entry.existing_bundle_id;
    }
    if (promoted_bundle != kInvalidObjectId) {
      made->bundles.push_back(promoted_bundle);
      made->bundle_specs.push_back(spec_index);
      continue;
    }
    const ObjectId resolved = resolve_existing_bundle(state_, g_, spec);
    if (resolved != kInvalidObjectId) {
      made->bundles.push_back(resolved);
      made->bundle_specs.push_back(spec_index);
      continue;
    }
    EditResult<ObjectId> bundle = state_.AddBundle(
        v.value.count, v.value.spacing_m, v.value.tmpl->id, spec.placement_explicit,
        spec.height_m, spec.lateral_m, spec.spacing_m, spec.placement_key);
    if (!bundle.ok) {
      out.error = bundle.error;
      return out;
    }
    add(*changes, bundle.change_set);
    made->bundles.push_back(bundle.value);
    made->bundle_specs.push_back(spec_index);
  }

  out.value = true;
  out.ok = true;
  return out;
}

EditResult<bool> pipeline::emit_ports(topo* made, const pairs& ps, ChangeSet* changes) {
  EditResult<bool> out{};
  if (made == nullptr || changes == nullptr) {
    out.error = "backbone topology: output missing";
    return out;
  }

  std::vector<ObjectId> node_id_by_local(g_.nodes.size(), kInvalidObjectId);
  for (std::size_t i = 0; i < g_.nodes.size(); ++i) {
    node_id_by_local[i] = g_.nodes[i].saved;
  }
  std::vector<SavedBackboneEdgeRef> edge_by_link(ps.links.size());
  for (const link& edge : ps.links) {
    if (edge.id < edge_by_link.size()) {
      edge_by_link[edge.id] = ref_for_existing_edge(state_, g_, edge);
    }
  }
  std::vector<bool> active_rows(ps.rows.size(), false);
  for (const link& edge : ps.links) {
    if (!edge.is_new) {
      continue;
    }
    if (edge.arow < active_rows.size()) {
      active_rows[edge.arow] = true;
    }
    if (edge.brow < active_rows.size()) {
      active_rows[edge.brow] = true;
    }
  }
  const std::vector<Vec3d> row_offsets = row_height_offsets(ps);
  made->rows.resize(ps.rows.size());
  for (const row& r : ps.rows) {
    if (r.node >= made->poles.size() || r.node >= g_.nodes.size()) {
      out.error = "backbone topology: row node missing";
      return out;
    }
    trow& tr = made->rows[r.id];
    tr.row = r.id;
    tr.node = r.node;
    tr.source = r.source;
    tr.axis = r.axis;
    tr.pole = made->poles[r.node];
    if (r.id >= active_rows.size() || !active_rows[r.id]) {
      continue;
    }
    const bool ownerless = g_.nodes[r.node].support == SupportKind::kMidair ||
                           g_.nodes[r.node].support == SupportKind::kExternal ||
                           g_.nodes[r.node].support == SupportKind::kGround;
    if (!ownerless && tr.pole == kInvalidObjectId) {
      out.error = "backbone topology: active row pole missing";
      return out;
    }
    tr.ports.resize(made->bundles.size());
    tr.placement_band_ids.resize(made->bundles.size());
    for (std::size_t bundle_index = 0; bundle_index < made->bundles.size(); ++bundle_index) {
      if (bundle_index >= made->bundle_specs.size()) {
        out.error = "backbone topology: bundle spec missing";
        return out;
      }
      const std::size_t spec_index = made->bundle_specs[bundle_index];
      const BackboneBundleSpec& bundle_spec = spec_.bundles[spec_index];
      EditResult<spec_view> v = view_for(state_, bundle_spec);
      if (!v.ok) {
        out.error = v.error;
        return out;
      }
      tr.ports[bundle_index].reserve(static_cast<std::size_t>(v.value.count));
      std::vector<PortPlacementBand> bands(static_cast<std::size_t>(v.value.count));
      if (!ownerless) {
        EditResult<std::vector<PortPlacementBand>> resolved_bands = bands_for(state_, tr.pole, v.value);
        if (!resolved_bands.ok) {
          out.error = resolved_bands.error;
          return out;
        }
        bands = std::move(resolved_bands.value);
      }
      tr.placement_band_ids[bundle_index].resize(static_cast<std::size_t>(v.value.count));
      const bool uses_lane_bands = !bundle_spec.placement_explicit &&
                                   std::adjacent_find(bands.begin(), bands.end(), [](const auto& a, const auto& b) {
                                     return a.band_id != b.band_id;
                                   }) != bands.end();
      for (int lane = 0; lane < v.value.count; ++lane) {
        const PortPlacementBand& band = bands[static_cast<std::size_t>(lane)];
        tr.placement_band_ids[bundle_index][static_cast<std::size_t>(lane)] = band.band_id;
        const port_scope scope{spec_.bundles[spec_index].bundle_template_id, made->bundles[bundle_index],
                               spec_.bundles[spec_index].placement_key,
                               PortKindForCategory(v.value.tmpl->category),
                               PortLayerForSpanLayer(v.value.layer), band.band_id};
        const Vec3d row_offset = (r.id < row_offsets.size()) ? row_offsets[r.id] : Vec3d{};
        Vec3d p{};
        if (ownerless) {
          if (g_.nodes[r.node].has_source_edge) {
            const SourceEdgeProjectionRef ref = source_projection_for(
                state_, g_.nodes[r.node], bundle_spec, static_cast<std::size_t>(lane));
            if (source_span_binding_for(state_, ref) == nullptr) {
              out.error = "backbone unsupported: source edge attachment is missing";
              return out;
            }
            p = g_.nodes[r.node].pos;
          } else {
            p = g_.nodes[r.node].pos;
          }
        } else {
          const Pole* pole = state_.edit_state_access().poles.find(tr.pole);
          if (pole == nullptr) {
            out.error = "backbone topology: active row pole missing";
            return out;
          }
          const double lane_offset = uses_lane_bands
                                         ? 0.0
                                         : LaneOffset(static_cast<std::size_t>(lane), v.value.count,
                                                      v.value.spacing_m);
          PortPlacementBand placement_band = band;
          if (bundle_spec.placement_explicit) {
            placement_band.height_center_m = bundle_spec.height_m;
            placement_band.lateral_center_m = bundle_spec.lateral_m;
          }
          p = PortWorldPosition(*pole, r.axis, placement_band, lane_offset,
                                spec_.constraints.lateral_offset_m, row_offset);
        }
        const SavedBackboneRowKey row_key = key_for(ps, tr, node_id_by_local, edge_by_link);
        ObjectId planned_port = kInvalidObjectId;
        for (const PromotionPlanEntry& entry : promotion_plan_) {
          if (entry.row == r.id && entry.spec_index == spec_index &&
              entry.lane_index == static_cast<std::size_t>(lane)) {
            planned_port = entry.port_id;
            break;
          }
        }
        EditResult<ObjectId> resolved{};
        resolved.ok = true;
        resolved.value = planned_port;
        if (resolved.value == kInvalidObjectId) {
          resolved = resolve_port_binding(state_, tr.pole, row_key, static_cast<std::size_t>(lane), scope);
          if (!resolved.ok) {
            out.error = resolved.error;
            return out;
          }
        }
        if (resolved.value != kInvalidObjectId) {
          Port* existing_port = state_.edit_state_access().ports.find(resolved.value);
          if (existing_port == nullptr) {
            out.error = "backbone topology: resolved port missing";
            return out;
          }
          if (existing_port->position_mode != PortPositionMode::kManual && !existing_port->user_edited_position &&
              moved_more_than_epsilon(existing_port->world_position, p)) {
            existing_port->world_position = p;
            ApplyPortBandTemplateFields(existing_port, band);
            CoreState::add_unique_id(changes->updated_ids, existing_port->id);
          }
          tr.ports[bundle_index].push_back(resolved.value);
          continue;
        }
        EditResult<ObjectId> port =
            state_.AddPort(ownerless ? kInvalidObjectId : made->poles[r.node], p, scope.kind, scope.layer);
        if (!port.ok) {
          out.error = port.error;
          return out;
        }
        Port* created_port = state_.edit_state_access().ports.find(port.value);
        ApplyPortBandTemplateFields(created_port, band);
        add(*changes, port.change_set);
        tr.ports[bundle_index].push_back(port.value);
      }
    }
  }

  out.value = true;
  out.ok = true;
  return out;
}

EditResult<bool> pipeline::emit_spans(topo* made, const pairs& ps, ChangeSet* changes) {
  EditResult<bool> out{};
  if (made == nullptr || changes == nullptr) {
    out.error = "backbone topology: output missing";
    return out;
  }
  for (const link& edge : ps.links) {
    if (!edge.is_new) {
      continue;
    }
    if (edge.arow >= made->rows.size() || edge.brow >= made->rows.size()) {
      out.error = "backbone topology: span row missing";
      return out;
    }
    for (std::size_t bundle_index = 0; bundle_index < made->bundles.size(); ++bundle_index) {
      if (bundle_index >= made->bundle_specs.size()) {
        out.error = "backbone topology: bundle spec missing";
        return out;
      }
      const BackboneBundleSpec& bundle_spec = spec_.bundles[made->bundle_specs[bundle_index]];
      EditResult<spec_view> v = view_for(state_, bundle_spec);
      if (!v.ok) {
        out.error = v.error;
        return out;
      }
      for (int lane = 0; lane < v.value.count; ++lane) {
        if (made->rows[edge.arow].ports.size() <= bundle_index ||
            made->rows[edge.brow].ports.size() <= bundle_index ||
            made->rows[edge.arow].ports[bundle_index].size() <= static_cast<std::size_t>(lane) ||
            made->rows[edge.brow].ports[bundle_index].size() <= static_cast<std::size_t>(lane)) {
          out.error = "backbone topology: span port missing";
          return out;
        }
        const ObjectId existing_edge_bundle_id = edge_bundle_for(state_, g_, edge, made->bundles[bundle_index]);
        const ObjectId existing_span_id = existing_span_for_lane(state_, existing_edge_bundle_id,
                                                                 static_cast<std::size_t>(lane));
        if (existing_span_id != kInvalidObjectId) {
          made->spans.push_back(tspan{existing_span_id, edge.id, bundle_index, static_cast<std::size_t>(lane),
                                      edge.arow, edge.brow, false});
          continue;
        }
        EditResult<ObjectId> span = state_.AddSpan(
            made->rows[edge.arow].ports[bundle_index][static_cast<std::size_t>(lane)],
            made->rows[edge.brow].ports[bundle_index][static_cast<std::size_t>(lane)],
            SpanKindForCategory(v.value.tmpl->category),
            v.value.layer, made->bundles[bundle_index]);
        if (!span.ok) {
          out.error = span.error;
          return out;
        }
        EditResult<bool> endpoints = state_.set_span_endpoint_nodes(span.value, made->rows[edge.arow].pole,
                                                                     made->rows[edge.brow].pole);
        if (!endpoints.ok) {
          out.error = endpoints.error;
          return out;
        }
        add(*changes, span.change_set);
        made->spans.push_back(
            tspan{span.value, edge.id, bundle_index, static_cast<std::size_t>(lane), edge.arow, edge.brow, true});
      }
    }
  }

  out.value = true;
  out.ok = true;
  return out;
}

EditResult<bool> pipeline::save_graph(const topo& made, const pairs& ps) {
  EditResult<bool> out{};
  std::vector<int> path_index_by_local(g_.nodes.size(), -1);
  for (std::size_t input_index = 0; input_index < local_by_input_.size(); ++input_index) {
    const std::size_t local = local_by_input_[input_index];
    if (local < path_index_by_local.size()) {
      path_index_by_local[local] = static_cast<int>(input_index);
    }
  }

  std::vector<ObjectId> node_id_by_local(g_.nodes.size(), kInvalidObjectId);
  for (std::size_t i = 0; i < g_.nodes.size() && i < made.poles.size(); ++i) {
    const ObjectId source_a = saved_node_id_for(state_, g_.nodes[i].source_edge_node_a);
    const ObjectId source_b = saved_node_id_for(state_, g_.nodes[i].source_edge_node_b);
    if (g_.nodes[i].saved != kInvalidObjectId) {
      node_id_by_local[i] = g_.nodes[i].saved;
      if (g_.nodes[i].on_route && i < path_index_by_local.size() && path_index_by_local[i] >= 0) {
        EditResult<bool> indexed =
            state_.bind_backbone_node_path_point_index(node_id_by_local[i], path_index_by_local[i]);
        if (!indexed.ok) {
          out.error = indexed.error;
          return out;
        }
      }
      if (g_.nodes[i].pole == kInvalidObjectId) {
        EditResult<bool> bound = state_.bind_backbone_node_bundle_modes(g_.nodes[i].saved, g_.nodes[i].bundle_modes);
        if (!bound.ok) {
          out.error = bound.error;
          return out;
        }
      }
      continue;
    }
    node_id_by_local[i] = state_.save_backbone_node(made.poles[i], g_.nodes[i].pos, g_.nodes[i].support, source_a,
                                                    source_b, g_.nodes[i].source_edge_t, g_.nodes[i].bundle_modes);
    if (g_.nodes[i].on_route && i < path_index_by_local.size() && path_index_by_local[i] >= 0) {
      EditResult<bool> indexed =
          state_.bind_backbone_node_path_point_index(node_id_by_local[i], path_index_by_local[i]);
      if (!indexed.ok) {
        out.error = indexed.error;
        return out;
      }
    }
  }

  std::vector<SavedBackboneEdgeRef> edge_by_link(ps.links.size());
  for (const link& edge : ps.links) {
    if (edge.a >= node_id_by_local.size() || edge.b >= node_id_by_local.size() || edge.id >= edge_by_link.size()) {
      continue;
    }
    if (edge.is_new) {
      if (edge.saved == kInvalidObjectId) {
        edge_by_link[edge.id] = state_.save_backbone_edge(node_id_by_local[edge.a], node_id_by_local[edge.b],
                                                          edge.route, edge.order, edge.dir,
                                                          spec_.constraints.lateral_offset_m);
        continue;
      }
      edge_by_link[edge.id] = ref_for_existing_edge(state_, g_, edge);
      if (edge_by_link[edge.id].edge_id == kInvalidObjectId) {
        out.error = "backbone graph: context link saved edge missing";
        return out;
      }
      continue;
    }
    edge_by_link[edge.id] = ref_for_existing_edge(state_, g_, edge);
    if (edge_by_link[edge.id].edge_id == kInvalidObjectId) {
      out.error = "backbone graph: context link saved edge missing";
      return out;
    }
  }

  for (const tspan& span : made.spans) {
    if (!span.is_new) {
      continue;
    }
    if (span.link >= edge_by_link.size() || span.link >= ps.links.size() || span.bundle >= made.bundles.size()) {
      continue;
    }
    const link& edge = ps.links[span.link];
    const SavedBackboneEdgeRef& stored = edge_by_link[span.link];
    if (stored.edge_id == kInvalidObjectId || edge.a >= node_id_by_local.size() || edge.b >= node_id_by_local.size()) {
      continue;
    }
    const bool edge_forward =
        stored.node_a == node_id_by_local[edge.a] && stored.node_b == node_id_by_local[edge.b];
    const ObjectId edge_bundle_id =
        state_.bind_backbone_bundle(stored.edge_id, made.bundles[span.bundle], edge_forward, edge.route, edge.order, edge.dir);
    EditResult<bool> span_bound = state_.bind_backbone_span(edge_bundle_id, span.lane, span.id);
    if (!span_bound.ok) {
      out.error = span_bound.error;
      return out;
    }
    auto bind_port = [&](std::size_t row_index) -> bool {
      if (row_index >= made.rows.size() || span.bundle >= made.rows[row_index].ports.size() ||
          span.lane >= made.rows[row_index].ports[span.bundle].size()) {
        out.error = "backbone graph: port binding row missing";
        return false;
      }
      const SavedBackboneRowKey row_key = key_for(ps, made.rows[row_index], node_id_by_local, edge_by_link);
      const Bundle* bundle = state_.view().bundles().find(made.bundles[span.bundle]);
      if (bundle == nullptr) {
        out.error = "backbone graph: bundle missing for port binding";
        return false;
      }
      const Port* port = state_.view().ports().find(made.rows[row_index].ports[span.bundle][span.lane]);
      if (port == nullptr) {
        out.error = "backbone graph: port missing for binding";
        return false;
      }
      const int placement_band_id =
          span.bundle < made.rows[row_index].placement_band_ids.size() &&
                  span.lane < made.rows[row_index].placement_band_ids[span.bundle].size()
              ? made.rows[row_index].placement_band_ids[span.bundle][span.lane]
              : 0;
      const port_scope scope{bundle->bundle_template_id, made.bundles[span.bundle], bundle->placement_key, port->kind,
                             port->layer, placement_band_id};
      const double layout_yaw_deg = PortLayoutYawDeg(made.rows[row_index].axis);
      const std::size_t spec_index =
          span.bundle < made.bundle_specs.size() ? made.bundle_specs[span.bundle] : bad;
      for (const PromotionPlanEntry& entry : promotion_plan_) {
        if (entry.row != row_index || entry.spec_index != spec_index || entry.lane_index != span.lane) {
          continue;
        }
        EditResult<bool> promoted = state_.promote_backbone_open_port_binding_exact(
            entry.existing_edge_bundle_id, entry.old_open_row_key, entry.lane_index,
            row_key, layout_yaw_deg, entry.port_id);
        if (!promoted.ok) {
          out.error = promoted.error;
          return false;
        }
        break;
      }
      EditResult<bool> bound =
          state_.bind_backbone_port(edge_bundle_id, row_key, span.lane, bundle->bundle_template_id, port->kind,
                                    port->layer, placement_band_id,
                                    layout_yaw_deg, port->id);
      if (!bound.ok) {
        out.error = bound.error;
        return false;
      }
      return true;
    };
    if (!bind_port(span.arow) || !bind_port(span.brow)) {
      return out;
    }
  }

  out.value = true;
  out.ok = true;
  return out;
}

rules pipeline::make(const topo& made, const pairs& ps, const groups& placement) const {
  rules out{};
  auto group_for = [&](std::size_t row, std::size_t bundle) -> const group* {
    for (const group& item : placement.items) {
      for (const group_member& member : item.row_members) {
        if (member.row == row && member.bundle == bundle) {
          return &item;
        }
      }
    }
    return nullptr;
  };
  for (const tspan& span : made.spans) {
    const trow& arow = made.rows[span.arow];
    const trow& brow = made.rows[span.brow];
    const group* start_group = group_for(span.arow, span.bundle);
    const group* end_group = group_for(span.brow, span.bundle);
    SpanLayoutRule rule{};
    rule.span_id = span.id;
    rule.flow_kind = BackboneFlowKind::kMain;
    rule.pass_mode = CurvePassMode::kPassThrough;
    if (start_group != nullptr || end_group != nullptr) {
      rule.flow_kind = BackboneFlowKind::kBranch;
      rule.pass_mode = CurvePassMode::kBranch;
      rule.lowering_kind = BackboneLoweringKind::kBranchSupport;
    }
    auto endpoint = [&](const trow& row, ObjectId port_id) {
      EndpointLayoutRule e{};
      e.endpoint_node_id = row.pole;
      e.port_id = port_id;
      e.semantic.owner_pole_id = row.pole;
      e.flow_kind = BackboneFlowKind::kMain;
      e.origin = LayoutOriginKind::kMainSupport;
      e.endpoint_source = LayoutEndpointSourceKind::kPlainSupport;
      e.port_source = PortPlacementSourceKind::kGenerated;
      e.side = SlotSide::kCenter;
      e.endpoint_mode = CurveEndpointMode::kDirectThrough;
      e.same_level_feasible = true;
      if (row.node < g_.nodes.size() && g_.nodes[row.node].has_source_edge) {
        e.source_projection = source_projection_for(
            state_, g_.nodes[row.node], spec_.bundles[made.bundle_specs[span.bundle]], span.lane);
      }
      return e;
    };
    rule.start = endpoint(arow, arow.ports[span.bundle][span.lane]);
    rule.end = endpoint(brow, brow.ports[span.bundle][span.lane]);
    auto apply_jumper = [&](std::size_t row_id, EndpointLayoutRule* endpoint) {
      if (endpoint == nullptr) {
        return;
      }
      for (const jumper& connection : ps.jumpers) {
        std::size_t peer_row = bad;
        if (connection.row_a == row_id) {
          peer_row = connection.row_b;
        } else if (connection.row_b == row_id) {
          peer_row = connection.row_a;
        } else {
          continue;
        }
        if (peer_row >= made.rows.size() || span.bundle >= made.rows[peer_row].ports.size() ||
            span.lane >= made.rows[peer_row].ports[span.bundle].size()) {
          continue;
        }
        endpoint->jumper_peer_port_id = made.rows[peer_row].ports[span.bundle][span.lane];
        return;
      }
    };
    apply_jumper(span.arow, &rule.start);
    apply_jumper(span.brow, &rule.end);
    const Span* state_span = state_.view().spans().find(span.id);
    auto apply_group = [&](const group* source, EndpointLayoutRule* endpoint) {
      if (source == nullptr || endpoint == nullptr) {
        return;
      }
      const double automatic_down_offset = std::max(0.0, -source->endpoint_offset_m);
      const double resolved_down_offset =
          state_span == nullptr ? automatic_down_offset
                                : state_.resolve_span_branch_down_offset_m(*state_span, automatic_down_offset);
      endpoint->flow_kind = BackboneFlowKind::kBranch;
      endpoint->origin = LayoutOriginKind::kBranchSupport;
      endpoint->default_lower_required = true;
      endpoint->same_level_feasible = false;
      endpoint->same_level_reason = SameLevelFeasibilityReason::kBundleRule;
      endpoint->semantic.lower_required = true;
      endpoint->semantic.lowering_blocked_by_policy = false;
      endpoint->semantic.support_group_id = static_cast<int>(source->id);
      endpoint->semantic.side_assignment_rule = SideAssignmentRuleKind::kChord;
      endpoint->semantic.support_orientation_rule = SupportOrientationRuleKind::kChord;
      endpoint->semantic.support_orientation_basis = SupportOrientationBasisKind::kChordForward;
      endpoint->semantic.has_side_axis = true;
      endpoint->semantic.side_axis = HorizontalNormalizedOr(source->group_axis);
      endpoint->semantic.chosen_side_sign = 1.0;
      endpoint->endpoint_offset_z_m = -resolved_down_offset;
      endpoint->automatic_endpoint_offset_z_m = source->endpoint_offset_m;
      endpoint->branch_down_offset_m = resolved_down_offset;
      endpoint->automatic_branch_down_offset_m = automatic_down_offset;
    };
    apply_group(start_group, &rule.start);
    apply_group(end_group, &rule.end);
    auto apply_branch_down_override = [&](EndpointLayoutRule* endpoint) {
      if (state_span == nullptr || endpoint == nullptr || !state_.has_span_branch_down_offset_override(span.id)) {
        return;
      }
      const double automatic_down = endpoint->automatic_branch_down_offset_m;
      const double resolved_down = state_.resolve_span_branch_down_offset_m(*state_span, automatic_down);
      if (resolved_down <= 0.0) {
        return;
      }
      endpoint->default_lower_required = true;
      endpoint->semantic.lower_required = true;
      endpoint->semantic.lowering_blocked_by_policy = false;
      endpoint->same_level_feasible = false;
      endpoint->same_level_reason = SameLevelFeasibilityReason::kBundleRule;
      endpoint->endpoint_offset_z_m = -resolved_down;
      endpoint->branch_down_offset_m = resolved_down;
      endpoint->automatic_branch_down_offset_m = automatic_down;
    };
    apply_branch_down_override(&rule.start);
    apply_branch_down_override(&rule.end);
    auto apply_socket_override = [&](bool is_start_endpoint, EndpointLayoutRule* endpoint) {
      if (state_span == nullptr || endpoint == nullptr) {
        return;
      }
      const int socket_id = state_.resolve_span_endpoint_socket_id(*state_span, is_start_endpoint);
      if (socket_id < 0) {
        return;
      }
      endpoint->endpoint_source = LayoutEndpointSourceKind::kAttachmentSocketOverride;
      endpoint->attachment_request.kind = EndpointAttachmentRequestKind::kAttachmentSocket;
      endpoint->attachment_request.requested_socket_id = socket_id;
      endpoint->resolved_socket_id = socket_id;
    };
    apply_socket_override(true, &rule.start);
    apply_socket_override(false, &rule.end);
    auto append_group_rule = [&](const EndpointLayoutRule& endpoint) {
      if (!UsesAuthoritativeGroupedLoweredSupport(endpoint.semantic)) {
        return;
      }
      const LoweredSupportGroupKey key = LoweredSupportGroupKeyFromDecision(endpoint.semantic);
      SupportGroupDecision& group = rule.support_group_rules[key];
      CopyLayoutSemantic(group, endpoint.semantic);
      group.side = endpoint.side;
      group.origin = endpoint.origin;
      group.order_decision_policy = endpoint.order_decision_policy;
      group.order_decision_choice = endpoint.order_decision_choice;
      group.order_decision_choice_reason = endpoint.order_decision_choice_reason;
      group.chosen_side = endpoint.chosen_side;
      group.used_junction_pair_side_assignment = endpoint.used_junction_pair_side_assignment;
    };
    append_group_rule(rule.start);
    append_group_rule(rule.end);
    out.data.spans.push_back(std::move(rule));
  }
  return out;
}

EditResult<layout> pipeline::make(const rules& made) const {
  EditResult<layout> out{};
  const EditState& edit = state_.view().edit_state();
  EditResult<FixturePlacementPlanByPort> fixture_plan =
      fixture_placement_plan_from_rules(state_, made.data.spans);
  if (!fixture_plan.ok) {
    out.error = fixture_plan.error;
    return out;
  }
  const auto endpoint_resolver = [&](const EndpointLayoutRule& endpoint) {
    return resolve_span_layout_endpoint(state_, edit, endpoint, &fixture_plan.value);
  };
  for (const SpanLayoutRule& rule : made.data.spans) {
    const Span* span = edit.spans.find(rule.span_id);
    if (span == nullptr) {
      out.error = "backbone layout: span not found";
      return out;
    }
    EditResult<SpanLayoutEntry> entry = derive_span_layout(rule, endpoint_resolver, 0);
    if (!entry.ok) {
      out.error = entry.error;
      return out;
    }
    out.value.entries.push_back(std::move(entry.value));
  }
  EditResult<VisualModelInstanceCache> model_instances =
      materialize_model_assemblies(state_, &fixture_plan.value);
  if (!model_instances.ok) {
    out.error = model_instances.error;
    return out;
  }
  out.value.model_instances = std::move(model_instances.value);
  out.ok = true;
  return out;
}

EditResult<geom> pipeline::make(const layout& made) const {
  EditResult<geom> out{};
  out.value.curves.data.reserve(made.entries.size());
  out.value.boxes.data.reserve(made.entries.size());
  for (const SpanLayoutEntry& entry : made.entries) {
    EditResult<DetailCurve> detail = make_curve(state_, entry.span_id, entry);
    if (!detail.ok) {
      out.error = detail.error;
      return out;
    }
    BoundsCacheEntry cached = bounds(detail.value);
    out.value.boxes.data.push_back({entry.span_id, std::move(cached)});
    out.value.curves.data.push_back({entry.span_id, std::move(detail.value)});
  }
  std::vector<ObjectId> visual_scope_span_ids{};
  visual_scope_span_ids.reserve(made.entries.size());
  for (const SpanLayoutEntry& entry : made.entries) {
    visual_scope_span_ids.push_back(entry.span_id);
  }
  out.value.visual_curves =
      make_visual_curve_parts(state_, made, visual_scope_span_ids, &out.value.curves);
  out.ok = true;
  return out;
}

draw pipeline::make(const layout& placed, const geom& shaped) const {
  draw out{};
  out.visuals.reserve(placed.entries.size());
  out.renders.reserve(shaped.curves.data.size());
  const VisualSettings& visual_settings = state_.view().visual_settings();
  for (const auto& item : shaped.curves.data) {
    out.renders.push_back({item.first, render(state_, item.first, item.second)});
  }
  for (const SpanLayoutEntry& entry : placed.entries) {
    out.visuals.push_back({entry.span_id, visual(visual_settings, entry)});
  }
  return out;
}

void pipeline::save(const rules& made) {
  state_.cache_span_rules(made.data);
}

void pipeline::save(layout made) {
  struct cached_group {
    LoweredSupportGroupKey key{};
    SupportGroupDecision decision{};
    LoweredSupportGroupPlacement placement{};
  };
  std::vector<cached_group> groups{};
  auto group_for = [&](const LoweredSupportGroupKey& key) -> cached_group& {
    for (cached_group& item : groups) {
      if (item.key == key) {
        return item;
      }
    }
    cached_group item{};
    item.key = key;
    groups.push_back(std::move(item));
    return groups.back();
  };
  auto collect_group = [&](const LayoutEndpoint& endpoint) {
    if (!UsesAuthoritativeGroupedLoweredSupport(endpoint)) {
      return;
    }
    const LoweredSupportGroupKey key = LoweredSupportGroupKeyFromDecision(endpoint);
    cached_group& item = group_for(key);
    CopyLayoutSemantic(item.decision, endpoint);
    item.decision.side = endpoint.side;
    item.decision.origin = endpoint.origin;
    item.decision.order_decision_policy = endpoint.order_decision_policy;
    item.decision.order_decision_choice = endpoint.order_decision_choice;
    item.decision.order_decision_choice_reason = endpoint.order_decision_choice_reason;
    item.decision.chosen_side = endpoint.chosen_side;
    item.decision.used_junction_pair_side_assignment = endpoint.used_junction_pair_side_assignment;
    item.placement.grouped_port_count += 1;
    item.placement.down_offset_m = std::max(item.placement.down_offset_m, endpoint.branch_down_offset_m);
    if (item.placement.attachment_worlds.empty()) {
      item.placement.mount_world = endpoint.support_world;
      item.placement.tip_world = endpoint.endpoint_world;
    }
    item.placement.attachment_worlds.push_back(endpoint.endpoint_world);
  };
  for (const SpanLayoutEntry& entry : made.entries) {
    collect_group(entry.start);
    collect_group(entry.end);
  }
  for (cached_group& item : groups) {
    state_.cache_support_group(std::move(item.decision), std::move(item.placement));
  }
  for (SpanLayoutEntry& entry : made.entries) {
    state_.cache_span_layout(std::move(entry));
  }
  state_.cache_visual_model_instances(std::move(made.model_instances));
}

void pipeline::save(geom made) {
  for (auto& item : made.curves.data) {
    state_.cache_span_curve(item.first, std::move(item.second));
  }
  for (auto& item : made.boxes.data) {
    state_.cache_span_bounds(item.first, std::move(item.second));
  }
  state_.cache_visual_curve_parts(std::move(made.visual_curves));
}

void pipeline::save(draw made) {
  for (auto& item : made.visuals) {
    state_.cache_span_visual(item.first, std::move(item.second));
  }
  for (auto& item : made.renders) {
    state_.cache_span_render(item.first, std::move(item.second));
  }
}

} // namespace wire::core::generation::backbone
