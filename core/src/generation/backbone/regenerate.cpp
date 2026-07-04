#include "wire/core/core_state.hpp"
#include "wire/core/core_view.hpp"
#include "wire/core/coord_utils.hpp"

#include "emit_shared.hpp"
#include "pipeline.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace wire::core {
namespace {

template <typename TValue> void append_unique(std::vector<TValue>& dst, const std::vector<TValue>& src) {
  for (const TValue& value : src) {
    if (std::find(dst.begin(), dst.end(), value) == dst.end()) {
      dst.push_back(value);
    }
  }
}

const SavedBackboneNode* saved_node_by_id(const SavedBackboneGraph& graph, ObjectId node_id) {
  const auto it = std::find_if(graph.nodes.begin(), graph.nodes.end(),
                               [&](const SavedBackboneNode& node) { return node.node_id == node_id; });
  return it == graph.nodes.end() ? nullptr : &*it;
}

const SavedBackboneEdge* saved_edge_by_id(const SavedBackboneGraph& graph, ObjectId edge_id) {
  const auto it = std::find_if(graph.edges.begin(), graph.edges.end(),
                               [&](const SavedBackboneEdge& edge) { return edge.edge_id == edge_id; });
  return it == graph.edges.end() ? nullptr : &*it;
}

const SavedBackboneEdgeBundle* saved_edge_bundle_by_id(const SavedBackboneGraph& graph, ObjectId edge_bundle_id) {
  const auto it = std::find_if(graph.edge_bundles.begin(), graph.edge_bundles.end(),
                               [&](const SavedBackboneEdgeBundle& edge_bundle) {
                                 return edge_bundle.edge_bundle_id == edge_bundle_id;
                               });
  return it == graph.edge_bundles.end() ? nullptr : &*it;
}

ObjectId span_for_lane(const CoreState& state, ObjectId edge_bundle_id, std::size_t lane_index) {
  const auto it = state.view().backbone_index().edge_bundle_span_bindings.find(edge_bundle_id);
  if (it == state.view().backbone_index().edge_bundle_span_bindings.end()) {
    return kInvalidObjectId;
  }
  ObjectId found = kInvalidObjectId;
  for (std::size_t binding_index : it->second) {
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

ObjectId port_for_lane(const CoreState& state, ObjectId edge_bundle_id, const SavedBackboneRowKey& row_key,
                       std::size_t lane_index, BundleKind bundle_template_id) {
  const auto it = state.view().backbone_index().edge_bundle_ports.find(edge_bundle_id);
  if (it == state.view().backbone_index().edge_bundle_ports.end()) {
    return kInvalidObjectId;
  }
  ObjectId found = kInvalidObjectId;
  for (std::size_t binding_index : it->second) {
    if (binding_index >= state.view().backbone().port_bindings.size()) {
      continue;
    }
    const SavedBackbonePortBinding& binding = state.view().backbone().port_bindings[binding_index];
    if (binding.lane_index != lane_index || binding.row_key != row_key ||
        binding.bundle_template_id != bundle_template_id) {
      continue;
    }
    if (found != kInvalidObjectId && found != binding.port_id) {
      return kInvalidObjectId;
    }
    found = binding.port_id;
  }
  return found;
}

struct regenerate_row_target {
  SavedBackboneRowKey row_key{};
  ObjectId pole_id = kInvalidObjectId;
  Vec3d row_axis{};
  PortPlacementBand previous_band{};
};

struct regenerate_target {
  ObjectId edge_bundle_id = kInvalidObjectId;
  ObjectId bundle_id = kInvalidObjectId;
  ObjectId edge_id = kInvalidObjectId;
  const SavedBackboneEdge* edge = nullptr;
  const SavedBackboneNode* node_a = nullptr;
  const SavedBackboneNode* node_b = nullptr;
  regenerate_row_target rows[2]{};
};

Vec3d expected_port_world(const CoreState& state, const regenerate_row_target& row,
                          const BundleTemplate& bundle_template, std::size_t lane_index) {
  const Pole* pole = state.view().poles().find(row.pole_id);
  if (pole == nullptr) {
    return {};
  }
  return generation::backbone::PortWorldPosition(*pole, row.row_axis, row.previous_band, lane_index,
                                                 bundle_template.fixed_count, bundle_template.default_spacing_m,
                                                 0.0, 0.0, {});
}

} // namespace

EditResult<bool> CoreState::regenerate_backbone_bundle_count_change(BundleKind bundle_template_id,
                                                                    const BundleTemplate& previous_template,
                                                                    const BundleTemplate& next_template,
                                                                    ChangeSet* change_set) {
  EditResult<bool> result{};
  auto fail = [&](std::string message) {
    result.error = std::move(message);
    return result;
  };

  if (previous_template.count_rule != BundleCountRuleKind::kFixed ||
      next_template.count_rule != BundleCountRuleKind::kFixed || previous_template.fixed_count <= 0 ||
      next_template.fixed_count <= previous_template.fixed_count) {
    return fail("backbone unsupported: bundle count regenerate supports fixed count increases only");
  }
  if (next_template.default_layer == SpanLayer::kUnknown) {
    return fail("backbone unsupported: bundle count regenerate requires a known layer");
  }

  std::vector<ObjectId> affected_edge_bundle_ids{};
  for (const SavedBackboneEdgeBundle& edge_bundle : view().backbone().edge_bundles) {
    const Bundle* bundle = view().bundles().find(edge_bundle.bundle_id);
    if (bundle != nullptr && bundle->bundle_template_id == bundle_template_id) {
      affected_edge_bundle_ids.push_back(edge_bundle.edge_bundle_id);
    }
  }
  if (affected_edge_bundle_ids.empty()) {
    result.ok = true;
    result.value = true;
    return result;
  }

  const SavedBackboneGraph& graph = view().backbone();
  for (ObjectId edge_bundle_id : affected_edge_bundle_ids) {
    const auto port_binding_it = runtime_.backbone_index.edge_bundle_ports.find(edge_bundle_id);
    if (port_binding_it == runtime_.backbone_index.edge_bundle_ports.end()) {
      continue;
    }
    for (std::size_t binding_index : port_binding_it->second) {
      if (binding_index >= graph.port_bindings.size()) {
        continue;
      }
      const SavedBackbonePortBinding& binding = graph.port_bindings[binding_index];
      if (!binding.row_key.source_is_open || binding.row_key.source_edge_b != kInvalidObjectId) {
        return fail("backbone unsupported: bundle count regenerate supports simple open rows only");
      }
    }
  }
  if (affected_edge_bundle_ids.size() != 1) {
    return fail("backbone unsupported: bundle count regenerate supports one edge bundle only");
  }

  std::vector<regenerate_target> targets{};
  targets.reserve(affected_edge_bundle_ids.size());

  for (ObjectId edge_bundle_id : affected_edge_bundle_ids) {
    const SavedBackboneEdgeBundle* edge_bundle = saved_edge_bundle_by_id(graph, edge_bundle_id);
    if (edge_bundle == nullptr) {
      return fail("backbone regenerate: edge bundle missing");
    }
    const SavedBackboneEdge* edge = saved_edge_by_id(graph, edge_bundle->edge_id);
    if (edge == nullptr) {
      return fail("backbone regenerate: edge missing");
    }
    const SavedBackboneNode* node_a = saved_node_by_id(graph, edge->node_a);
    const SavedBackboneNode* node_b = saved_node_by_id(graph, edge->node_b);
    if (node_a == nullptr || node_b == nullptr || node_a->pole_id == kInvalidObjectId ||
        node_b->pole_id == kInvalidObjectId) {
      return fail("backbone unsupported: bundle count regenerate supports pole-owned endpoints only");
    }
    const Bundle* bundle = view().bundles().find(edge_bundle->bundle_id);
    if (bundle == nullptr) {
      return fail("backbone regenerate: bundle missing");
    }
    if (bundle->conductor_count != previous_template.fixed_count) {
      return fail("backbone unsupported: bundle count is not synchronized with previous template");
    }

    const auto span_binding_it = runtime_.backbone_index.edge_bundle_span_bindings.find(edge_bundle_id);
    if (span_binding_it == runtime_.backbone_index.edge_bundle_span_bindings.end()) {
      return fail("backbone regenerate: span binding missing");
    }
    std::vector<bool> seen_lanes(static_cast<std::size_t>(previous_template.fixed_count), false);
    for (std::size_t binding_index : span_binding_it->second) {
      if (binding_index >= graph.span_bindings.size()) {
        return fail("backbone regenerate: span binding index invalid");
      }
      const SavedBackboneSpanBinding& binding = graph.span_bindings[binding_index];
      if (binding.lane_index >= seen_lanes.size()) {
        return fail("backbone unsupported: existing span lane is outside previous bundle count");
      }
      seen_lanes[binding.lane_index] = true;
      const auto attachment_it = runtime_.relation_index.attachments_by_span.find(binding.span_id);
      if (attachment_it != runtime_.relation_index.attachments_by_span.end() && !attachment_it->second.empty()) {
        return fail("backbone unsupported: bundle count regenerate does not preserve user attachments yet");
      }
    }
    if (std::find(seen_lanes.begin(), seen_lanes.end(), false) != seen_lanes.end()) {
      return fail("backbone regenerate: previous bundle lanes are incomplete");
    }

    std::vector<SavedBackboneRowKey> row_keys{};
    const auto port_binding_it = runtime_.backbone_index.edge_bundle_ports.find(edge_bundle_id);
    if (port_binding_it == runtime_.backbone_index.edge_bundle_ports.end()) {
      return fail("backbone regenerate: port binding missing");
    }
    for (std::size_t binding_index : port_binding_it->second) {
      if (binding_index >= graph.port_bindings.size()) {
        return fail("backbone regenerate: port binding index invalid");
      }
      const SavedBackbonePortBinding& binding = graph.port_bindings[binding_index];
      const Port* port = view().ports().find(binding.port_id);
      if (port == nullptr) {
        return fail("backbone regenerate: bound port missing");
      }
      if (port->position_mode == PortPositionMode::kManual || port->user_edited_position) {
        return fail("backbone unsupported: bundle count regenerate does not move manual ports");
      }
      if (!binding.row_key.source_is_open || binding.row_key.source_edge_a != edge_bundle->edge_id ||
          binding.row_key.source_edge_b != kInvalidObjectId) {
        return fail("backbone unsupported: bundle count regenerate supports simple open rows only");
      }
      if (std::find(row_keys.begin(), row_keys.end(), binding.row_key) == row_keys.end()) {
        row_keys.push_back(binding.row_key);
      }
    }
    if (row_keys.size() != 2) {
      return fail("backbone unsupported: bundle count regenerate requires two endpoint rows");
    }

    regenerate_target target{};
    target.edge_bundle_id = edge_bundle_id;
    target.bundle_id = edge_bundle->bundle_id;
    target.edge_id = edge->edge_id;
    target.edge = edge;
    target.node_a = node_a;
    target.node_b = node_b;
    for (std::size_t i = 0; i < row_keys.size(); ++i) {
      const SavedBackboneNode* node = saved_node_by_id(graph, row_keys[i].node_id);
      if (node == nullptr || node->pole_id == kInvalidObjectId) {
        return fail("backbone unsupported: bundle count regenerate supports pole-owned endpoints only");
      }
      const Pole* pole = view().poles().find(node->pole_id);
      if (pole == nullptr || pole->pole_type_id == kInvalidPoleTypeId) {
        return fail("backbone regenerate: endpoint pole missing");
      }
      if (pole->user_edited || pole->placement_mode == PlacementMode::kManual) {
        return fail("backbone unsupported: bundle count regenerate does not move manual poles");
      }
      const auto pole_type_it = view().pole_types().find(pole->pole_type_id);
      if (pole_type_it == view().pole_types().end()) {
        return fail("backbone regenerate: endpoint pole type missing");
      }
      const EditResult<PortPlacementBand> previous_band = generation::backbone::SelectPortPlacementBand(
          pole_type_it->second, previous_template.category, previous_template.default_layer);
      if (!previous_band.ok) {
        return fail(previous_band.error);
      }
      if (!generation::backbone::SelectPortPlacementBand(pole_type_it->second, next_template.category,
                                                         next_template.default_layer).ok) {
        return fail("backbone unsupported: port band missing");
      }
      if (row_keys[i].node_id != edge->node_a && row_keys[i].node_id != edge->node_b) {
        return fail("backbone regenerate: row node is not on edge");
      }
      target.rows[i] = regenerate_row_target{row_keys[i], node->pole_id, ComputeLateralAxis(edge->dir),
                                             previous_band.value};
    }
    targets.push_back(target);
  }

  for (const regenerate_target& target : targets) {
    for (std::size_t lane = 0; lane < static_cast<std::size_t>(next_template.fixed_count); ++lane) {
      const ObjectId span_id = span_for_lane(*this, target.edge_bundle_id, lane);
      if (lane < static_cast<std::size_t>(previous_template.fixed_count)) {
        if (span_id == kInvalidObjectId) {
          return fail("backbone regenerate: previous bundle lanes are incomplete");
        }
      } else if (span_id != kInvalidObjectId) {
        return fail("backbone unsupported: bundle count regenerate found existing new lane");
      }
    }
    for (const regenerate_row_target& row : target.rows) {
      for (std::size_t lane = 0; lane < static_cast<std::size_t>(next_template.fixed_count); ++lane) {
        const ObjectId port_id = port_for_lane(*this, target.edge_bundle_id, row.row_key, lane, bundle_template_id);
        if (lane >= static_cast<std::size_t>(previous_template.fixed_count)) {
          if (port_id != kInvalidObjectId) {
            return fail("backbone unsupported: bundle count regenerate found existing new lane port");
          }
          continue;
        }
        const Port* port = view().ports().find(port_id);
        if (port == nullptr) {
          return fail("backbone regenerate: bound port missing");
        }
        const Vec3d expected = expected_port_world(*this, row, previous_template, lane);
        if (LengthSquared(port->world_position - expected) > 1e-12) {
          return fail("backbone unsupported: bundle count regenerate cannot reconstruct existing port placement");
        }
      }
    }
  }

  generation::backbone::graph made_graph{};
  const regenerate_target& target = targets.front();
  generation::backbone::node a{};
  a.id = 0;
  a.pos = target.node_a->position;
  a.support = target.node_a->support_kind;
  a.pole = target.node_a->pole_id;
  a.saved = target.node_a->node_id;
  a.is_new = false;
  a.on_route = true;
  a.bundle_modes = target.node_a->bundle_modes;
  generation::backbone::node b{};
  b.id = 1;
  b.pos = target.node_b->position;
  b.support = target.node_b->support_kind;
  b.pole = target.node_b->pole_id;
  b.saved = target.node_b->node_id;
  b.is_new = false;
  b.on_route = true;
  b.bundle_modes = target.node_b->bundle_modes;
  generation::backbone::link link{};
  link.id = 0;
  link.a = 0;
  link.b = 1;
  link.route = target.edge->route;
  link.order = target.edge->order;
  link.dir = target.edge->dir;
  link.saved = target.edge_id;
  link.is_new = true;
  made_graph.nodes = {a, b};
  made_graph.links = {link};

  BackboneSpec spec{};
  spec.pole_type_id = kInvalidPoleTypeId;
  BackboneBundleSpec bundle_spec{};
  bundle_spec.bundle_template_id = bundle_template_id;
  bundle_spec.layer = next_template.default_layer;
  spec.bundles.push_back(bundle_spec);

  authoritative_.bundle_templates[bundle_template_id] = next_template;
  generation::backbone::pipeline pipeline(*this, spec);
  EditResult<GenerateBundleFromPathResult> rebuilt = pipeline.build_prepared_regenerate(std::move(made_graph), {0});
  if (!rebuilt.ok) {
    authoritative_.bundle_templates[bundle_template_id] = previous_template;
    return fail(rebuilt.error);
  }

  for (const regenerate_target& item : targets) {
    Bundle* bundle = authoritative_.edit_state.bundles.find(item.bundle_id);
    if (bundle != nullptr) {
      bundle->conductor_count = next_template.fixed_count;
      bundle->phase_spacing_m = next_template.default_spacing_m;
      if (change_set != nullptr) {
        CoreState::add_unique_id(change_set->updated_ids, bundle->id);
      }
    }
  }
  if (change_set != nullptr) {
    append_unique(change_set->created_ids, rebuilt.change_set.created_ids);
    append_unique(change_set->updated_ids, rebuilt.change_set.updated_ids);
    append_unique(change_set->deleted_ids, rebuilt.change_set.deleted_ids);
  }

  result.ok = true;
  result.value = true;
  return result;
}

} // namespace wire::core