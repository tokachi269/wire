#include "wire/core/core_state.hpp"
#include "wire/core/coord_utils.hpp"

#include "../../generation/backbone/curve_parts.hpp"

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

PortKind port_kind_for_category(ConnectionCategory category) {
  switch (category) {
  case ConnectionCategory::kCommunication:
  case ConnectionCategory::kOptical:
    return PortKind::kCommunication;
  case ConnectionCategory::kHighVoltage:
  case ConnectionCategory::kLowVoltage:
  case ConnectionCategory::kDrop:
  default:
    return PortKind::kPower;
  }
}

PortLayer port_layer_for_span_layer(SpanLayer layer) {
  switch (layer) {
  case SpanLayer::kHighVoltage:
    return PortLayer::kHighVoltage;
  case SpanLayer::kLowVoltage:
    return PortLayer::kLowVoltage;
  case SpanLayer::kCommunication:
    return PortLayer::kCommunication;
  case SpanLayer::kOptical:
    return PortLayer::kOptical;
  case SpanLayer::kDrop:
    return PortLayer::kDrop;
  case SpanLayer::kUnknown:
  default:
    return PortLayer::kUnknown;
  }
}

SpanKind span_kind_for_category(ConnectionCategory category) {
  return category == ConnectionCategory::kDrop ? SpanKind::kService : SpanKind::kDistribution;
}

int rank_for_span_layer(SpanLayer layer) {
  switch (layer) {
  case SpanLayer::kHighVoltage:
    return 2;
  case SpanLayer::kDrop:
    return 0;
  case SpanLayer::kLowVoltage:
  case SpanLayer::kCommunication:
  case SpanLayer::kOptical:
  case SpanLayer::kUnknown:
  default:
    return 1;
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

PortPlacementBand best_band_for_template(const PoleTypeDefinition& pole_type, const BundleTemplate& bundle_template,
                                         bool* found) {
  const int target_rank = rank_for_span_layer(bundle_template.default_layer);
  const PortPlacementBand* best = nullptr;
  for (const PortPlacementBand& band : pole_type.port_bands) {
    if (!band.enabled || band.category != bundle_template.category || band.layer != target_rank) {
      continue;
    }
    if (best == nullptr || band.priority > best->priority ||
        (band.priority == best->priority && band.band_id < best->band_id)) {
      best = &band;
    }
  }
  if (found != nullptr) {
    *found = best != nullptr;
  }
  return best == nullptr ? PortPlacementBand{} : *best;
}

} // namespace

EditResult<bool> CoreState::regenerate_backbone_bundle_count_change(BundleKind bundle_template_id,
                                                                    const BundleTemplate& previous_template,
                                                                    const BundleTemplate& next_template,
                                                                    ChangeSet* change_set) {
  EditResult<bool> result{};
  if (previous_template.count_rule != BundleCountRuleKind::kFixed ||
      next_template.count_rule != BundleCountRuleKind::kFixed ||
      previous_template.fixed_count <= 0 || next_template.fixed_count <= previous_template.fixed_count) {
    result.error = "backbone unsupported: bundle count regenerate supports fixed count increases only";
    return result;
  }
  if (next_template.default_layer == SpanLayer::kUnknown) {
    result.error = "backbone unsupported: bundle count regenerate requires a known layer";
    return result;
  }

  std::vector<ObjectId> affected_edge_bundle_ids{};
  for (const SavedBackboneEdgeBundle& edge_bundle : authoritative_.backbone.edge_bundles) {
    const Bundle* bundle = authoritative_.edit_state.bundles.find(edge_bundle.bundle_id);
    if (bundle != nullptr && bundle->bundle_template_id == bundle_template_id) {
      affected_edge_bundle_ids.push_back(edge_bundle.edge_bundle_id);
    }
  }
  if (affected_edge_bundle_ids.empty()) {
    result.ok = true;
    result.value = true;
    return result;
  }

  struct row_target {
    SavedBackboneRowKey row_key{};
    ObjectId pole_id = kInvalidObjectId;
    Vec3d row_axis{};
    PortPlacementBand band{};
  };
  struct edge_target {
    ObjectId edge_bundle_id = kInvalidObjectId;
    ObjectId bundle_id = kInvalidObjectId;
    row_target rows[2]{};
  };

  std::vector<edge_target> targets{};
  targets.reserve(affected_edge_bundle_ids.size());

  auto fail = [&](std::string message) {
    result.error = std::move(message);
    return result;
  };

  for (ObjectId edge_bundle_id : affected_edge_bundle_ids) {
    const SavedBackboneEdgeBundle* edge_bundle = saved_edge_bundle_by_id(authoritative_.backbone, edge_bundle_id);
    if (edge_bundle == nullptr) {
      return fail("backbone regenerate: edge bundle missing");
    }
    const SavedBackboneEdge* edge = saved_edge_by_id(authoritative_.backbone, edge_bundle->edge_id);
    if (edge == nullptr) {
      return fail("backbone regenerate: edge missing");
    }
    const Bundle* bundle = authoritative_.edit_state.bundles.find(edge_bundle->bundle_id);
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
      if (binding_index >= authoritative_.backbone.span_bindings.size()) {
        return fail("backbone regenerate: span binding index invalid");
      }
      const SavedBackboneSpanBinding& binding = authoritative_.backbone.span_bindings[binding_index];
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
      if (binding_index >= authoritative_.backbone.port_bindings.size()) {
        return fail("backbone regenerate: port binding index invalid");
      }
      const SavedBackbonePortBinding& binding = authoritative_.backbone.port_bindings[binding_index];
      const Port* port = authoritative_.edit_state.ports.find(binding.port_id);
      if (port == nullptr) {
        return fail("backbone regenerate: bound port missing");
      }
      if (port->position_mode == PortPositionMode::kManual) {
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

    edge_target target{};
    target.edge_bundle_id = edge_bundle_id;
    target.bundle_id = edge_bundle->bundle_id;
    for (std::size_t i = 0; i < row_keys.size(); ++i) {
      const SavedBackboneNode* node = saved_node_by_id(authoritative_.backbone, row_keys[i].node_id);
      if (node == nullptr || node->pole_id == kInvalidObjectId) {
        return fail("backbone unsupported: bundle count regenerate supports pole-owned endpoints only");
      }
      const Pole* pole = authoritative_.edit_state.poles.find(node->pole_id);
      if (pole == nullptr || pole->pole_type_id == kInvalidPoleTypeId) {
        return fail("backbone regenerate: endpoint pole missing");
      }
      const auto pole_type_it = authoritative_.pole_types.find(pole->pole_type_id);
      if (pole_type_it == authoritative_.pole_types.end()) {
        return fail("backbone regenerate: endpoint pole type missing");
      }
      bool has_band = false;
      const PortPlacementBand band = best_band_for_template(pole_type_it->second, next_template, &has_band);
      if (!has_band) {
        return fail("backbone unsupported: port band missing");
      }
      Vec3d row_axis = ComputeLateralAxis(edge->dir);
      if (row_keys[i].node_id == edge->node_b) {
        row_axis = ScaleVec(row_axis, -1.0);
      } else if (row_keys[i].node_id != edge->node_a) {
        return fail("backbone regenerate: row node is not on edge");
      }
      target.rows[i] = row_target{row_keys[i], node->pole_id, row_axis, band};
    }
    targets.push_back(std::move(target));
  }

  auto port_for_lane = [&](ObjectId edge_bundle_id, const SavedBackboneRowKey& row_key,
                           std::size_t lane_index) -> ObjectId {
    const auto it = runtime_.backbone_index.edge_bundle_ports.find(edge_bundle_id);
    if (it == runtime_.backbone_index.edge_bundle_ports.end()) {
      return kInvalidObjectId;
    }
    ObjectId found = kInvalidObjectId;
    for (std::size_t binding_index : it->second) {
      if (binding_index >= authoritative_.backbone.port_bindings.size()) {
        continue;
      }
      const SavedBackbonePortBinding& binding = authoritative_.backbone.port_bindings[binding_index];
      if (binding.lane_index == lane_index && binding.row_key == row_key &&
          binding.bundle_template_id == bundle_template_id) {
        if (found != kInvalidObjectId && found != binding.port_id) {
          return kInvalidObjectId;
        }
        found = binding.port_id;
      }
    }
    return found;
  };

  auto span_for_lane = [&](ObjectId edge_bundle_id, std::size_t lane_index) -> ObjectId {
    const auto it = runtime_.backbone_index.edge_bundle_span_bindings.find(edge_bundle_id);
    if (it == runtime_.backbone_index.edge_bundle_span_bindings.end()) {
      return kInvalidObjectId;
    }
    ObjectId found = kInvalidObjectId;
    for (std::size_t binding_index : it->second) {
      if (binding_index >= authoritative_.backbone.span_bindings.size()) {
        continue;
      }
      const SavedBackboneSpanBinding& binding = authoritative_.backbone.span_bindings[binding_index];
      if (binding.lane_index == lane_index) {
        if (found != kInvalidObjectId && found != binding.span_id) {
          return kInvalidObjectId;
        }
        found = binding.span_id;
      }
    }
    return found;
  };

  auto place_port = [&](const row_target& row, std::size_t lane_index) {
    const Pole* pole = authoritative_.edit_state.poles.find(row.pole_id);
    const Vec3d row_axis = HorizontalNormalizedOr(row.row_axis);
    const Vec3d forward_axis = ScaleVec(ComputeLateralAxis(row_axis), -1.0);
    const double layout_yaw_deg = YawDegFromXY(forward_axis);
    const double lane_offset =
        (static_cast<double>(lane_index) - (static_cast<double>(next_template.fixed_count - 1) * 0.5)) *
        next_template.default_spacing_m;
    const Vec3d local{0.0, lane_offset, row.band.height_center_m};
    return LocalPointToWorld(BuildPoleFrame(pole->world_transform, layout_yaw_deg), local);
  };

  const PortKind port_kind = port_kind_for_category(next_template.category);
  const PortLayer port_layer = port_layer_for_span_layer(next_template.default_layer);
  const SpanKind span_kind = span_kind_for_category(next_template.category);
  for (edge_target& target : targets) {
    Bundle* bundle = authoritative_.edit_state.bundles.find(target.bundle_id);
    if (bundle != nullptr) {
      bundle->conductor_count = next_template.fixed_count;
      bundle->phase_spacing_m = next_template.default_spacing_m;
      if (change_set != nullptr) {
        CoreState::add_unique_id(change_set->updated_ids, bundle->id);
      }
    }

    for (std::size_t lane = 0; lane < static_cast<std::size_t>(next_template.fixed_count); ++lane) {
      ObjectId ports[2]{kInvalidObjectId, kInvalidObjectId};
      for (std::size_t side = 0; side < 2; ++side) {
        const Vec3d world = place_port(target.rows[side], lane);
        ObjectId port_id = port_for_lane(target.edge_bundle_id, target.rows[side].row_key, lane);
        if (port_id == kInvalidObjectId) {
          EditResult<ObjectId> added = AddPort(target.rows[side].pole_id, world, port_kind, port_layer);
          if (!added.ok) {
            return fail(added.error);
          }
          port_id = added.value;
          if (change_set != nullptr) {
            append_unique(change_set->created_ids, added.change_set.created_ids);
          }
          EditResult<bool> bound = bind_backbone_port(target.edge_bundle_id, target.rows[side].row_key, lane,
                                                      bundle_template_id, port_kind, port_layer, port_id);
          if (!bound.ok) {
            return fail(bound.error);
          }
        } else {
          Port* port = authoritative_.edit_state.ports.find(port_id);
          if (port == nullptr) {
            return fail("backbone regenerate: bound port missing");
          }
          port->world_position = world;
          port->kind = port_kind;
          port->layer = port_layer;
          port->category = next_template.category;
          port->template_layer = rank_for_span_layer(next_template.default_layer);
          apply_port_position_mode(*port, PortPositionMode::kAuto, PortPlacementSourceKind::kGenerated);
          if (change_set != nullptr) {
            CoreState::add_unique_id(change_set->updated_ids, port_id);
          }
        }
        ports[side] = port_id;
      }

      ObjectId span_id = span_for_lane(target.edge_bundle_id, lane);
      if (span_id == kInvalidObjectId) {
        EditResult<ObjectId> added = AddSpan(ports[0], ports[1], span_kind, next_template.default_layer,
                                             target.bundle_id);
        if (!added.ok) {
          return fail(added.error);
        }
        span_id = added.value;
        EditResult<bool> nodes = set_span_endpoint_nodes(span_id, target.rows[0].pole_id, target.rows[1].pole_id);
        if (!nodes.ok) {
          return fail(nodes.error);
        }
        EditResult<bool> bound = bind_backbone_span(target.edge_bundle_id, lane, span_id);
        if (!bound.ok) {
          return fail(bound.error);
        }
        if (change_set != nullptr) {
          append_unique(change_set->created_ids, added.change_set.created_ids);
        }
      }
      auto endpoint = [](ObjectId pole_id, ObjectId port_id) {
        EndpointLayoutRule rule{};
        rule.endpoint_node_id = pole_id;
        rule.port_id = port_id;
        rule.semantic.owner_pole_id = pole_id;
        rule.flow_kind = BackboneFlowKind::kMain;
        rule.origin = LayoutOriginKind::kMainSupport;
        rule.endpoint_source = LayoutEndpointSourceKind::kPlainSupport;
        rule.port_source = PortPlacementSourceKind::kGenerated;
        rule.side = SlotSide::kCenter;
        rule.endpoint_mode = CurveEndpointMode::kDirectThrough;
        rule.same_level_feasible = true;
        return rule;
      };
      SpanLayoutRule rule{};
      rule.span_id = span_id;
      rule.flow_kind = BackboneFlowKind::kMain;
      rule.pass_mode = CurvePassMode::kPassThrough;
      rule.start = endpoint(target.rows[0].pole_id, ports[0]);
      rule.end = endpoint(target.rows[1].pole_id, ports[1]);
      SpanLayoutRules rules{};
      rules.spans.push_back(std::move(rule));
      cache_span_rules(rules);
      EditResult<bool> derived = DeriveGeneratedSpanOutputs(span_id);
      if (!derived.ok) {
        return fail(derived.error);
      }
      if (change_set != nullptr) {
        CoreState::add_unique_id(change_set->updated_ids, span_id);
      }
    }
  }

  cache_visual_curve_parts(generation::backbone::make_visual_curve_parts(*this, {}));
  result.ok = true;
  result.value = true;
  return result;
}

} // namespace wire::core
