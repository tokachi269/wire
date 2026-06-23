#include "wire/core/core_state.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace wire::core {

namespace {

struct BackboneIncidentMeta {
  std::uint64_t min_session_id = std::numeric_limits<std::uint64_t>::max();
  std::uint32_t min_generation_order = std::numeric_limits<std::uint32_t>::max();
};

std::vector<JunctionInfo> BuildJunctionsFromRelations(
    const std::unordered_map<ObjectId, JunctionRelation>& relations_by_node,
    const std::unordered_map<ObjectId, std::unordered_set<ObjectId>>& adjacency,
    const std::unordered_map<ObjectId, std::unordered_map<ObjectId, BackboneIncidentMeta>>& incident_meta_by_node,
    const std::unordered_map<ObjectId, std::uint64_t>& existing_prioritized_session_by_node,
    const std::unordered_map<ObjectId, std::unordered_map<ObjectId, std::uint64_t>>& existing_incident_session_by_node) {
  std::vector<JunctionInfo> junctions{};
  junctions.reserve(relations_by_node.size());
  for (const auto& [node_id, relation] : relations_by_node) {
    const auto it_neighbors = adjacency.find(node_id);
    if (it_neighbors == adjacency.end() || it_neighbors->second.size() < 3) {
      continue;
    }

    JunctionInfo junction{};
    junction.node_id = node_id;
    junction.incidents.reserve(relation.incidents.size());
    for (const JunctionIncidentRelation& source : relation.incidents) {
      if (!it_neighbors->second.contains(source.neighbor_node_id)) {
        continue;
      }

      JunctionIncident incident{};
      incident.neighbor_node_id = source.neighbor_node_id;
      incident.order = static_cast<int>(junction.incidents.size());
      incident.primary = junction.incidents.empty();

      if (const auto it_node = incident_meta_by_node.find(node_id); it_node != incident_meta_by_node.end()) {
        if (const auto it_meta = it_node->second.find(source.neighbor_node_id); it_meta != it_node->second.end() &&
            it_meta->second.min_session_id != std::numeric_limits<std::uint64_t>::max()) {
          incident.source_session_id = it_meta->second.min_session_id;
        }
      }
      if (incident.source_session_id == 0) {
        if (const auto it_existing_node = existing_incident_session_by_node.find(node_id);
            it_existing_node != existing_incident_session_by_node.end()) {
          if (const auto it_existing_source = it_existing_node->second.find(source.neighbor_node_id);
              it_existing_source != it_existing_node->second.end()) {
            incident.source_session_id = it_existing_source->second;
          }
        }
      }

      junction.incidents.push_back(incident);
    }

    if (junction.incidents.size() < 3) {
      continue;
    }

    junction.prioritized_session_id = junction.incidents.front().source_session_id;
    if (junction.prioritized_session_id == 0) {
      if (const auto it_prioritized = existing_prioritized_session_by_node.find(node_id);
          it_prioritized != existing_prioritized_session_by_node.end()) {
        junction.prioritized_session_id = it_prioritized->second;
      }
    }
    junctions.push_back(std::move(junction));
  }

  std::sort(junctions.begin(), junctions.end(),
            [](const JunctionInfo& a, const JunctionInfo& b) { return a.node_id < b.node_id; });
  return junctions;
}

} // namespace

ObjectId CoreState::save_backbone_node(ObjectId pole_id, const Vec3d& position, SupportKind support_kind,
                                       ObjectId source_edge_node_a, ObjectId source_edge_node_b,
                                       double source_edge_t, std::vector<SupportNodeBundleMode> bundle_modes) {
  if (pole_id != kInvalidObjectId) {
    const auto existing = runtime_.backbone_index.pole_node.find(pole_id);
    if (existing != runtime_.backbone_index.pole_node.end()) {
      return existing->second;
    }
  }

  SavedBackboneNode node{};
  node.node_id = identity_.id_generator.next();
  node.pole_id = pole_id;
  node.support_kind = (pole_id != kInvalidObjectId) ? SupportKind::kPole : support_kind;
  node.position = position;
  node.has_source_edge =
      pole_id == kInvalidObjectId && source_edge_node_a != kInvalidObjectId && source_edge_node_b != kInvalidObjectId;
  node.source_edge_node_a = node.has_source_edge ? source_edge_node_a : kInvalidObjectId;
  node.source_edge_node_b = node.has_source_edge ? source_edge_node_b : kInvalidObjectId;
  node.source_edge_t = node.has_source_edge ? source_edge_t : 0.0;
  node.bundle_modes = (pole_id == kInvalidObjectId) ? std::move(bundle_modes) : std::vector<SupportNodeBundleMode>{};
  authoritative_.backbone.nodes.push_back(node);
  if (pole_id != kInvalidObjectId) {
    runtime_.backbone_index.pole_node[pole_id] = node.node_id;
  }
  return node.node_id;
}

void CoreState::publish_backbone_result_nodes(std::vector<SupportNode> nodes) {
  nodes.erase(std::remove_if(nodes.begin(), nodes.end(), [](const SupportNode& node) {
                return node.node_id == kInvalidObjectId;
              }),
              nodes.end());
  std::sort(nodes.begin(), nodes.end(), [](const SupportNode& lhs, const SupportNode& rhs) {
    return lhs.node_id < rhs.node_id;
  });
  debug_.last_generation_support_nodes = std::move(nodes);
}

void CoreState::publish_lane_assignments(std::vector<SegmentLaneAssignment> assignments) {
  debug_.last_generation_lane_assignments = std::move(assignments);
}

void CoreState::cache_support_group(SupportGroupDecision decision, LoweredSupportGroupPlacement placement) {
  const LoweredSupportGroupKey key = LoweredSupportGroupKeyFromDecision(decision);
  if (key.owner_pole_id == kInvalidObjectId || key.support_group_id < 0) {
    return;
  }
  runtime_.cache_state.span_layout_cache.support_groups.authority.by_key[key] = std::move(decision);
  runtime_.cache_state.span_layout_cache.support_groups.placement.by_key[key] = std::move(placement);
}

EditResult<bool> CoreState::bind_backbone_node_bundle_modes(
    ObjectId node_id, const std::vector<SupportNodeBundleMode>& bundle_modes) {
  EditResult<bool> out{};
  if (bundle_modes.empty()) {
    out.ok = true;
    out.value = true;
    return out;
  }
  auto node_it = std::find_if(authoritative_.backbone.nodes.begin(), authoritative_.backbone.nodes.end(),
                              [&](const SavedBackboneNode& node) { return node.node_id == node_id; });
  if (node_it == authoritative_.backbone.nodes.end()) {
    out.error = "bb2 graph: saved node missing for bundle policy";
    return out;
  }
  if (node_it->pole_id != kInvalidObjectId) {
    out.error = "bb2 graph: pole node cannot carry bundle policy";
    return out;
  }
  for (const SupportNodeBundleMode& mode : bundle_modes) {
    const auto mode_it = std::find_if(node_it->bundle_modes.begin(), node_it->bundle_modes.end(),
                                      [&](const SupportNodeBundleMode& existing) {
                                        return existing.bundle_template_id == mode.bundle_template_id;
                                      });
    if (mode_it == node_it->bundle_modes.end()) {
      node_it->bundle_modes.push_back(mode);
      continue;
    }
    if (mode_it->mode != mode.mode) {
      out.error = "bb2 graph: conflicting saved node bundle policy";
      return out;
    }
  }
  std::sort(node_it->bundle_modes.begin(), node_it->bundle_modes.end(),
            [](const SupportNodeBundleMode& a, const SupportNodeBundleMode& b) {
              return static_cast<int>(a.bundle_template_id) < static_cast<int>(b.bundle_template_id);
            });
  out.ok = true;
  out.value = true;
  return out;
}

SavedBackboneEdgeRef CoreState::save_backbone_edge(ObjectId node_a, ObjectId node_b, std::size_t route,
                                                   std::size_t order, const Vec3d& dir) {
  SavedBackboneEdgeRef out{};
  if (node_a == kInvalidObjectId || node_b == kInvalidObjectId || node_a == node_b) {
    return out;
  }
  const BackboneEdgeKey key{std::min(node_a, node_b), std::max(node_a, node_b)};
  if (const auto existing = runtime_.backbone_index.edge_by_nodes.find(key);
      existing != runtime_.backbone_index.edge_by_nodes.end()) {
    for (const SavedBackboneEdge& edge : authoritative_.backbone.edges) {
      if (edge.edge_id == existing->second) {
        out.edge_id = edge.edge_id;
        out.node_a = edge.node_a;
        out.node_b = edge.node_b;
        return out;
      }
    }
    return out;
  }

  SavedBackboneEdge edge{};
  edge.edge_id = identity_.id_generator.next();
  edge.node_a = node_a;
  edge.node_b = node_b;
  edge.route = route;
  edge.order = order;
  edge.dir = dir;
  authoritative_.backbone.edges.push_back(edge);
  index_add(runtime_.backbone_index.node_edges, node_a, edge.edge_id);
  index_add(runtime_.backbone_index.node_edges, node_b, edge.edge_id);
  runtime_.backbone_index.edge_by_nodes[key] = edge.edge_id;
  out.edge_id = edge.edge_id;
  out.node_a = edge.node_a;
  out.node_b = edge.node_b;
  out.created = true;
  return out;
}

ObjectId CoreState::bind_backbone_bundle(ObjectId edge_id, ObjectId bundle_id, bool edge_forward, std::size_t route,
                                         std::size_t order, const Vec3d& dir) {
  if (edge_id == kInvalidObjectId || bundle_id == kInvalidObjectId) {
    return kInvalidObjectId;
  }
  for (SavedBackboneEdgeBundle& item : authoritative_.backbone.edge_bundles) {
    if (item.edge_id == edge_id && item.bundle_id == bundle_id) {
      return item.edge_bundle_id;
    }
  }

  SavedBackboneEdgeBundle item{};
  item.edge_bundle_id = identity_.id_generator.next();
  item.edge_id = edge_id;
  item.bundle_id = bundle_id;
  item.edge_forward = edge_forward;
  item.route = route;
  item.order = order;
  item.dir = dir;
  authoritative_.backbone.edge_bundles.push_back(item);
  index_add(runtime_.backbone_index.edge_bundles, edge_id, item.edge_bundle_id);
  index_add(runtime_.backbone_index.bundle_edge, bundle_id, edge_id);
  return item.edge_bundle_id;
}

EditResult<bool> CoreState::bind_backbone_span(ObjectId edge_bundle_id, std::size_t lane_index, ObjectId span_id) {
  EditResult<bool> out{};
  if (edge_bundle_id == kInvalidObjectId || span_id == kInvalidObjectId) {
    out.error = "invalid backbone span binding";
    return out;
  }
  SavedBackboneEdgeBundle* found = nullptr;
  for (SavedBackboneEdgeBundle& item : authoritative_.backbone.edge_bundles) {
    if (item.edge_bundle_id == edge_bundle_id) {
      found = &item;
      break;
    }
  }
  if (found == nullptr) {
    out.error = "invalid backbone span binding";
    return out;
  }
  const auto existing = runtime_.backbone_index.edge_bundle_span_bindings.find(edge_bundle_id);
  if (existing != runtime_.backbone_index.edge_bundle_span_bindings.end()) {
    for (std::size_t index : existing->second) {
      if (index >= authoritative_.backbone.span_bindings.size()) {
        continue;
      }
      const SavedBackboneSpanBinding& binding = authoritative_.backbone.span_bindings[index];
      if (binding.lane_index == lane_index) {
        out.error = "duplicate backbone span binding";
        return out;
      }
    }
  }
  const auto span_existing = runtime_.backbone_index.span_bindings_by_span.find(span_id);
  if (span_existing != runtime_.backbone_index.span_bindings_by_span.end() && !span_existing->second.empty()) {
    out.error = "duplicate backbone span binding";
    return out;
  }
  add_unique_id(found->span_ids, span_id);
  index_add(runtime_.backbone_index.edge_bundle_spans, edge_bundle_id, span_id);
  runtime_.backbone_index.span_edge_bundle[span_id] = edge_bundle_id;
  SavedBackboneSpanBinding binding{};
  binding.edge_bundle_id = edge_bundle_id;
  binding.lane_index = lane_index;
  binding.span_id = span_id;
  const std::size_t index = authoritative_.backbone.span_bindings.size();
  authoritative_.backbone.span_bindings.push_back(binding);
  runtime_.backbone_index.edge_bundle_span_bindings[edge_bundle_id].push_back(index);
  runtime_.backbone_index.span_bindings_by_span[span_id].push_back(index);
  out.value = true;
  out.ok = true;
  return out;
}

EditResult<bool> CoreState::bind_backbone_port(ObjectId edge_bundle_id, const SavedBackboneRowKey& row_key,
                                               std::size_t lane_index, BundleKind bundle_template_id,
                                               PortKind port_kind, PortLayer port_layer, ObjectId port_id) {
  EditResult<bool> out{};
  if (edge_bundle_id == kInvalidObjectId || port_id == kInvalidObjectId || row_key.node_id == kInvalidObjectId ||
      row_key.source_edge_a == kInvalidObjectId) {
    out.error = "invalid backbone port binding";
    return out;
  }
  const auto existing = runtime_.backbone_index.edge_bundle_ports.find(edge_bundle_id);
  if (existing != runtime_.backbone_index.edge_bundle_ports.end()) {
    for (std::size_t index : existing->second) {
      if (index >= authoritative_.backbone.port_bindings.size()) {
        continue;
      }
      const SavedBackbonePortBinding& binding = authoritative_.backbone.port_bindings[index];
      if (binding.row_key == row_key && binding.lane_index == lane_index) {
        out.error = "duplicate backbone port binding";
        return out;
      }
    }
  }
  const auto port_existing = runtime_.backbone_index.port_bindings_by_port.find(port_id);
  if (port_existing != runtime_.backbone_index.port_bindings_by_port.end()) {
    for (std::size_t index : port_existing->second) {
      if (index >= authoritative_.backbone.port_bindings.size()) {
        continue;
      }
      const SavedBackbonePortBinding& binding = authoritative_.backbone.port_bindings[index];
      if (binding.bundle_template_id != bundle_template_id || binding.port_kind != port_kind ||
          binding.port_layer != port_layer) {
        out.error = "incompatible backbone port binding";
        return out;
      }
    }
  }

  SavedBackbonePortBinding binding{};
  binding.edge_bundle_id = edge_bundle_id;
  binding.row_key = row_key;
  binding.lane_index = lane_index;
  binding.bundle_template_id = bundle_template_id;
  binding.port_kind = port_kind;
  binding.port_layer = port_layer;
  binding.port_id = port_id;
  const std::size_t index = authoritative_.backbone.port_bindings.size();
  authoritative_.backbone.port_bindings.push_back(binding);
  runtime_.backbone_index.edge_bundle_ports[edge_bundle_id].push_back(index);
  runtime_.backbone_index.port_bindings_by_port[port_id].push_back(index);
  out.value = true;
  out.ok = true;
  return out;
}

PoleDetailInfo CoreState::GetPoleDetail(ObjectId pole_id) const {
  PoleDetailInfo detail{};
  detail.pole = authoritative_.edit_state.poles.find(pole_id);
  if (detail.pole == nullptr) {
    return detail;
  }
  detail.pole_type = find_pole_type(detail.pole->pole_type_id);
  if (const auto it = runtime_.relation_index.ports_by_pole.find(pole_id); it != runtime_.relation_index.ports_by_pole.end()) {
    for (ObjectId port_id : it->second) {
      if (const Port* port = authoritative_.edit_state.ports.find(port_id); port != nullptr) {
        detail.owned_ports.push_back(port);
      }
    }
  }
  if (const auto it = runtime_.relation_index.anchors_by_pole.find(pole_id); it != runtime_.relation_index.anchors_by_pole.end()) {
    for (ObjectId anchor_id : it->second) {
      if (const Anchor* anchor = authoritative_.edit_state.anchors.find(anchor_id); anchor != nullptr) {
        detail.owned_anchors.push_back(anchor);
      }
    }
  }
  std::sort(detail.owned_ports.begin(), detail.owned_ports.end(),
            [](const Port* a, const Port* b) { return a->id < b->id; });
  std::sort(detail.owned_anchors.begin(), detail.owned_anchors.end(),
            [](const Anchor* a, const Anchor* b) { return a->id < b->id; });
  return detail;
}

std::vector<ObjectId> CoreState::GetSpansByBundle(ObjectId bundle_id) const {
  std::vector<ObjectId> span_ids{};
  if (bundle_id == kInvalidObjectId) {
    return span_ids;
  }
  auto it = runtime_.relation_index.spans_by_bundle.find(bundle_id);
  if (it == runtime_.relation_index.spans_by_bundle.end()) {
    return span_ids;
  }
  span_ids.reserve(it->second.size());
  for (ObjectId span_id : it->second) {
    if (authoritative_.edit_state.spans.find(span_id) != nullptr) {
      span_ids.push_back(span_id);
    }
  }
  std::sort(span_ids.begin(), span_ids.end());
  span_ids.erase(std::unique(span_ids.begin(), span_ids.end()), span_ids.end());
  return span_ids;
}

EditResult<ResolveBranchPickResult>
CoreState::ResolveBranchPick(const PickResult& pick) {
  return ResolveBranchPick(pick, ResolveBranchPickOptions{});
}

EditResult<ResolveBranchPickResult>
CoreState::ResolveBranchPick(const PickResult& pick, const ResolveBranchPickOptions& options) {
  EditResult<ResolveBranchPickResult> result{};
  if (options.snap_radius_world < 0.0) {
    result.error = "snap_radius_world must be >= 0";
    return result;
  }

  std::vector<BundleKind> selected_template_ids = options.selected_bundle_template_ids;
  std::sort(selected_template_ids.begin(), selected_template_ids.end(),
            [](BundleKind a, BundleKind b) { return static_cast<int>(a) < static_cast<int>(b); });
  selected_template_ids.erase(std::unique(selected_template_ids.begin(), selected_template_ids.end()),
                              selected_template_ids.end());

  struct SelectedTemplatePolicy {
    BundleKind id = BundleKind::kLowVoltage;
    const BundleTemplate* bundle_template = nullptr;
    bool allow_midair_path = true;
  };
  std::vector<SelectedTemplatePolicy> selected_templates{};
  selected_templates.reserve(selected_template_ids.size());
  for (BundleKind bundle_template_id : selected_template_ids) {
    const BundleTemplate* bundle_template = find_bundle_template(bundle_template_id);
    if (bundle_template == nullptr) {
      result.error = "bundle template not found";
      return result;
    }
    SelectedTemplatePolicy selected{};
    selected.id = bundle_template_id;
    selected.bundle_template = bundle_template;
    selected.allow_midair_path = bundle_template->allow_midair_node && bundle_template->allow_midair_branch;
    selected_templates.push_back(selected);
  }

  auto sqr_dist = [](const Vec3d& a, const Vec3d& b) {
    const Vec3d d = a - b;
    return d.x * d.x + d.y * d.y + d.z * d.z;
  };
  auto segment_t_xy = [](const Vec3d& p, const Vec3d& a, const Vec3d& b) {
    const double abx = b.x - a.x;
    const double aby = b.y - a.y;
    const double ab2 = abx * abx + aby * aby;
    if (ab2 <= 1e-12) {
      return 0.0;
    }
    const double apx = p.x - a.x;
    const double apy = p.y - a.y;
    return std::clamp((apx * abx + apy * aby) / ab2, 0.0, 1.0);
  };

  auto resolve_node_info = [&](ObjectId node_id, SupportKind* out_kind, Vec3d* out_position) {
    if (out_kind == nullptr || out_position == nullptr) {
      return false;
    }
    for (const SupportNode& node : debug_.last_generation_support_nodes) {
      if (node.node_id != node_id) {
        continue;
      }
      *out_kind = node.support_kind;
      *out_position = node.position;
      return true;
    }
    const Pole* pole = authoritative_.edit_state.poles.find(node_id);
    if (pole != nullptr) {
      *out_kind = SupportKind::kPole;
      *out_position = pole->world_transform.position;
      return true;
    }
    const auto saved_it = std::find_if(authoritative_.backbone.nodes.begin(), authoritative_.backbone.nodes.end(),
                                       [&](const SavedBackboneNode& node) {
                                         return node.node_id == node_id && node.pole_id == kInvalidObjectId;
                                       });
    if (saved_it != authoritative_.backbone.nodes.end()) {
      *out_kind = saved_it->support_kind;
      *out_position = saved_it->position;
      return true;
    }
    return false;
  };

  auto resolve_segment_endpoints = [&](ObjectId* out_node_a_id, ObjectId* out_node_b_id, Vec3d* out_pos_a,
                                       Vec3d* out_pos_b) {
    if (out_node_a_id == nullptr || out_node_b_id == nullptr || out_pos_a == nullptr || out_pos_b == nullptr) {
      return false;
    }
    *out_node_a_id = pick.segment_node_a_id;
    *out_node_b_id = pick.segment_node_b_id;
    *out_pos_a = pick.segment_endpoint_a_world;
    *out_pos_b = pick.segment_endpoint_b_world;
    bool has_endpoints = pick.has_segment_endpoints;

    if (const Span* span = authoritative_.edit_state.spans.find(pick.hit_id); span != nullptr) {
      auto saved_span_nodes = [&](ObjectId* out_saved_a, ObjectId* out_saved_b) {
        if (out_saved_a == nullptr || out_saved_b == nullptr) {
          return false;
        }
        const auto edge_bundle_it = runtime_.backbone_index.span_edge_bundle.find(span->id);
        if (edge_bundle_it == runtime_.backbone_index.span_edge_bundle.end()) {
          return false;
        }
        const auto bundle_it = std::find_if(authoritative_.backbone.edge_bundles.begin(),
                                            authoritative_.backbone.edge_bundles.end(),
                                            [&](const SavedBackboneEdgeBundle& item) {
                                              return item.edge_bundle_id == edge_bundle_it->second;
                                            });
        if (bundle_it == authoritative_.backbone.edge_bundles.end()) {
          return false;
        }
        const auto edge_it = std::find_if(authoritative_.backbone.edges.begin(), authoritative_.backbone.edges.end(),
                                          [&](const SavedBackboneEdge& item) {
                                            return item.edge_id == bundle_it->edge_id;
                                          });
        if (edge_it == authoritative_.backbone.edges.end()) {
          return false;
        }
        *out_saved_a = bundle_it->edge_forward ? edge_it->node_a : edge_it->node_b;
        *out_saved_b = bundle_it->edge_forward ? edge_it->node_b : edge_it->node_a;
        return true;
      };
      ObjectId saved_node_a = kInvalidObjectId;
      ObjectId saved_node_b = kInvalidObjectId;
      const bool has_saved_nodes = saved_span_nodes(&saved_node_a, &saved_node_b);
      const Port* pa = authoritative_.edit_state.ports.find(span->port_a_id);
      const Port* pb = authoritative_.edit_state.ports.find(span->port_b_id);
      if (pa != nullptr && pb != nullptr) {
        auto resolve_endpoint = [&](bool is_a, const Port* port, ObjectId* out_node_id, Vec3d* out_pos) {
          const ObjectId explicit_node_id = is_a ? span->endpoint_node_a_id : span->endpoint_node_b_id;
          const ObjectId saved_node_id = is_a ? saved_node_a : saved_node_b;
          const ObjectId node_id = (explicit_node_id != kInvalidObjectId) ? explicit_node_id
                                 : (port->owner_pole_id != kInvalidObjectId) ? port->owner_pole_id
                                 : has_saved_nodes                            ? saved_node_id
                                                                              : kInvalidObjectId;
          *out_node_id = node_id;
          *out_pos = port->world_position;
          SupportKind kind = SupportKind::kPole;
          Vec3d node_pos{};
          if (node_id != kInvalidObjectId && resolve_node_info(node_id, &kind, &node_pos)) {
            *out_pos = node_pos;
          } else if (port->owner_pole_id != kInvalidObjectId) {
            if (const Pole* pole = authoritative_.edit_state.poles.find(port->owner_pole_id); pole != nullptr) {
              *out_pos = pole->world_transform.position;
            }
          }
        };
        has_endpoints = true;
        resolve_endpoint(true, pa, out_node_a_id, out_pos_a);
        resolve_endpoint(false, pb, out_node_b_id, out_pos_b);
      }
    }
    return has_endpoints;
  };

  auto selected_bundle_modes = [&]() {
    std::vector<SupportNodeBundleMode> modes{};
    modes.reserve(selected_templates.size());
    for (const SelectedTemplatePolicy& selected : selected_templates) {
      SupportNodeBundleMode mode{};
      mode.bundle_template_id = selected.id;
      mode.mode = (!options.enforce_midair_template_policy || selected.allow_midair_path) ? BundleNodeMode::kPassThrough
                                                                                           : BundleNodeMode::kNotPresent;
      modes.push_back(mode);
    }
    std::sort(modes.begin(), modes.end(), [](const SupportNodeBundleMode& a, const SupportNodeBundleMode& b) {
      return static_cast<int>(a.bundle_template_id) < static_cast<int>(b.bundle_template_id);
    });
    return modes;
  };

  auto resolve_ownerless_surface_pick = [&](SupportKind kind) {
    if (options.enforce_midair_template_policy && !selected_templates.empty()) {
      const bool any_allow_midair = std::any_of(selected_templates.begin(), selected_templates.end(),
                                                [](const SelectedTemplatePolicy& selected) {
                                                  return selected.allow_midair_path;
                                                });
      if (!any_allow_midair) {
        result.error = "no selected bundle template allows midair branch";
        return;
      }
    }

    result.value.resolution = PickBranchResolutionKind::kMidair;
    result.value.position = pick.hit_pos_world;
    result.value.support_kind = kind;
    if (!selected_templates.empty() || options.create_midair_node_set) {
      SupportNode node{};
      node.node_id = debug_.next_virtual_support_node_id++;
      node.support_kind = kind;
      node.position = pick.hit_pos_world;
      node.pole_id = kInvalidObjectId;
      node.path_point_index = -1;
      node.has_tangent_hint = false;
      node.bundle_modes = selected_bundle_modes();
      debug_.last_generation_support_nodes.push_back(node);
      std::sort(debug_.last_generation_support_nodes.begin(), debug_.last_generation_support_nodes.end(),
                [](const SupportNode& a, const SupportNode& b) { return a.node_id < b.node_id; });
      result.value.resolved_node_id = node.node_id;
    } else {
      result.value.resolved_node_id = kInvalidObjectId;
    }
    result.ok = true;
  };

  if (pick.hit_kind == PickHitKind::kBuilding) {
    resolve_ownerless_surface_pick(SupportKind::kBuilding);
    return result;
  }

  if (pick.hit_kind == PickHitKind::kGround) {
    resolve_ownerless_surface_pick(SupportKind::kGround);
    return result;
  }

  if (pick.hit_kind == PickHitKind::kNode) {
    if (pick.hit_id == kInvalidObjectId) {
      result.error = "node pick must include a valid hit_id";
      return result;
    }
    result.value.resolution = PickBranchResolutionKind::kNode;
    result.value.resolved_node_id = pick.hit_id;
    result.value.position = pick.hit_pos_world;
    result.value.support_kind = SupportKind::kPole;
    (void)resolve_node_info(pick.hit_id, &result.value.support_kind, &result.value.position);
    if (!selected_templates.empty()) {
      const auto saved_it = std::find_if(authoritative_.backbone.nodes.begin(), authoritative_.backbone.nodes.end(),
                                         [&](const SavedBackboneNode& node) {
                                           return node.node_id == pick.hit_id &&
                                                  node.pole_id == kInvalidObjectId &&
                                                  node.support_kind == result.value.support_kind;
                                         });
      if (saved_it != authoritative_.backbone.nodes.end()) {
        SupportNode node{};
        node.node_id = debug_.next_virtual_support_node_id++;
        node.support_kind = saved_it->support_kind;
        node.position = saved_it->position;
        node.pole_id = kInvalidObjectId;
        node.saved_backbone_node_id = saved_it->node_id;
        node.path_point_index = -1;
        node.bundle_modes = selected_bundle_modes();
        debug_.last_generation_support_nodes.push_back(node);
        std::sort(debug_.last_generation_support_nodes.begin(), debug_.last_generation_support_nodes.end(),
                  [](const SupportNode& a, const SupportNode& b) { return a.node_id < b.node_id; });
        result.value.resolved_node_id = node.node_id;
        result.value.position = node.position;
      }
      if (result.value.support_kind == SupportKind::kPole) {
        const Pole* pole = authoritative_.edit_state.poles.find(pick.hit_id);
        if (pole != nullptr) {
          SupportNode node{};
          node.node_id = debug_.next_virtual_support_node_id++;
          node.support_kind = SupportKind::kPole;
          node.position = pole->world_transform.position;
          node.pole_id = pole->id;
          const auto saved_pole_it =
              std::find_if(authoritative_.backbone.nodes.begin(), authoritative_.backbone.nodes.end(),
                           [&](const SavedBackboneNode& saved) { return saved.pole_id == pole->id; });
          if (saved_pole_it != authoritative_.backbone.nodes.end()) {
            node.saved_backbone_node_id = saved_pole_it->node_id;
          }
          node.path_point_index = -1;
          node.bundle_modes = selected_bundle_modes();
          debug_.last_generation_support_nodes.push_back(node);
          std::sort(debug_.last_generation_support_nodes.begin(), debug_.last_generation_support_nodes.end(),
                    [](const SupportNode& a, const SupportNode& b) { return a.node_id < b.node_id; });
          result.value.resolved_node_id = node.node_id;
          result.value.position = node.position;
        }
      }
    }
    result.ok = true;
    return result;
  }

  if (pick.hit_kind != PickHitKind::kSegment) {
    result.error = "pick hit kind is not supported for branch resolution";
    return result;
  }

  ObjectId node_a_id = kInvalidObjectId;
  ObjectId node_b_id = kInvalidObjectId;
  Vec3d endpoint_a{};
  Vec3d endpoint_b{};
  const bool has_endpoints = resolve_segment_endpoints(&node_a_id, &node_b_id, &endpoint_a, &endpoint_b);
  const double source_edge_t = has_endpoints ? segment_t_xy(pick.hit_pos_world, endpoint_a, endpoint_b) : 0.0;
  Vec3d branch_position = pick.hit_pos_world;
  if (const Span* span = authoritative_.edit_state.spans.find(pick.hit_id); span != nullptr) {
    const Port* pa = authoritative_.edit_state.ports.find(span->port_a_id);
    const Port* pb = authoritative_.edit_state.ports.find(span->port_b_id);
    if (pa != nullptr && pb != nullptr) {
      const double t = segment_t_xy(pick.hit_pos_world, pa->world_position, pb->world_position);
      branch_position.z = pa->world_position.z + (pb->world_position.z - pa->world_position.z) * t;
    }
  } else if (has_endpoints) {
    branch_position.z = endpoint_a.z + (endpoint_b.z - endpoint_a.z) * source_edge_t;
  }
  if (has_endpoints && options.snap_radius_world > 0.0) {
    const double snap_r2 = options.snap_radius_world * options.snap_radius_world;
    const double da2 = sqr_dist(pick.hit_pos_world, endpoint_a);
    const double db2 = sqr_dist(pick.hit_pos_world, endpoint_b);
    if ((da2 <= snap_r2 && node_a_id != kInvalidObjectId) || (db2 <= snap_r2 && node_b_id != kInvalidObjectId)) {
      const bool use_a = (da2 <= db2);
      result.value.resolution = PickBranchResolutionKind::kNode;
      result.value.resolved_node_id = use_a ? node_a_id : node_b_id;
      result.value.position = use_a ? endpoint_a : endpoint_b;
      result.value.support_kind = SupportKind::kPole;
      result.value.snapped_from_segment_endpoint = true;
      (void)resolve_node_info(result.value.resolved_node_id, &result.value.support_kind, &result.value.position);
      result.ok = true;
      return result;
    }
  }

  if (options.enforce_midair_template_policy && !selected_templates.empty()) {
    const bool any_allow_midair = std::any_of(selected_templates.begin(), selected_templates.end(),
                                              [](const SelectedTemplatePolicy& selected) {
                                                return selected.allow_midair_path;
                                              });
    if (!any_allow_midair) {
      result.error = "no selected bundle template allows midair branch";
      return result;
    }
  }

  constexpr double kReuseEps2 = 1e-10;
  for (const SupportNode& node : debug_.last_generation_support_nodes) {
    if (node.support_kind != SupportKind::kMidair) {
      continue;
    }
    if (sqr_dist(node.position, branch_position) > kReuseEps2) {
      continue;
    }
    if (options.create_midair_node && has_endpoints) {
      for (SupportNode& mutable_node : debug_.last_generation_support_nodes) {
        if (mutable_node.node_id != node.node_id) {
          continue;
        }
        mutable_node.has_source_edge = true;
        mutable_node.source_edge_node_a_id = node_a_id;
        mutable_node.source_edge_node_b_id = node_b_id;
        mutable_node.source_edge_t = source_edge_t;
        for (const SelectedTemplatePolicy& selected : selected_templates) {
          const auto it_mode =
              std::find_if(mutable_node.bundle_modes.begin(), mutable_node.bundle_modes.end(),
                           [&](const SupportNodeBundleMode& mode) { return mode.bundle_template_id == selected.id; });
          const BundleNodeMode next_mode =
              (!options.enforce_midair_template_policy || selected.allow_midair_path) ? BundleNodeMode::kPassThrough
                                                                                       : BundleNodeMode::kNotPresent;
          if (it_mode == mutable_node.bundle_modes.end()) {
            SupportNodeBundleMode mode{};
            mode.bundle_template_id = selected.id;
            mode.mode = next_mode;
            mutable_node.bundle_modes.push_back(mode);
          } else {
            it_mode->mode = next_mode;
          }
        }
        std::sort(mutable_node.bundle_modes.begin(), mutable_node.bundle_modes.end(),
                  [](const SupportNodeBundleMode& a, const SupportNodeBundleMode& b) {
                    return static_cast<int>(a.bundle_template_id) < static_cast<int>(b.bundle_template_id);
                  });
        break;
      }
    }
    result.value.resolution = PickBranchResolutionKind::kMidair;
    result.value.resolved_node_id = node.node_id;
    result.value.support_kind = node.support_kind;
    result.value.position = node.position;
    result.ok = true;
    return result;
  }

  if (!options.create_midair_node || (selected_templates.empty() && !options.create_midair_node_set)) {
    result.value.resolution = PickBranchResolutionKind::kMidair;
    result.value.resolved_node_id = kInvalidObjectId;
    result.value.support_kind = SupportKind::kMidair;
    result.value.position = branch_position;
    result.ok = true;
    return result;
  }

  SupportNode midair{};
  midair.node_id = debug_.next_virtual_support_node_id++;
  midair.support_kind = SupportKind::kMidair;
  midair.position = branch_position;
  midair.pole_id = kInvalidObjectId;
  if (has_endpoints) {
    midair.has_source_edge = true;
    midair.source_edge_node_a_id = node_a_id;
    midair.source_edge_node_b_id = node_b_id;
    midair.source_edge_t = source_edge_t;
  }
  midair.path_point_index = -1;
  midair.has_tangent_hint = false;
  for (const SelectedTemplatePolicy& selected : selected_templates) {
    SupportNodeBundleMode mode{};
    mode.bundle_template_id = selected.id;
    mode.mode = (!options.enforce_midair_template_policy || selected.allow_midair_path) ? BundleNodeMode::kPassThrough
                                                                                         : BundleNodeMode::kNotPresent;
    midair.bundle_modes.push_back(mode);
  }
  debug_.last_generation_support_nodes.push_back(midair);
  std::sort(debug_.last_generation_support_nodes.begin(), debug_.last_generation_support_nodes.end(),
            [](const SupportNode& a, const SupportNode& b) { return a.node_id < b.node_id; });

  result.value.resolution = PickBranchResolutionKind::kMidair;
  result.value.resolved_node_id = midair.node_id;
  result.value.support_kind = midair.support_kind;
  result.value.position = midair.position;
  result.ok = true;
  return result;
}

BackboneResult CoreState::BuildBackboneResult() const {
  BackboneResult out{};
  auto resolve_span_endpoint_node = [&](const Span& span, const Port* port, bool is_a) -> ObjectId {
    const ObjectId explicit_node_id = is_a ? span.endpoint_node_a_id : span.endpoint_node_b_id;
    if (explicit_node_id != kInvalidObjectId) {
      return explicit_node_id;
    }
    return (port == nullptr) ? kInvalidObjectId : port->owner_pole_id;
  };
  auto resolve_span_endpoint_position = [&](ObjectId node_id, const Port* port) -> Vec3d {
    for (const SupportNode& node : debug_.last_generation_support_nodes) {
      if (node.node_id == node_id) {
        return node.position;
      }
    }
    if (const Pole* pole = authoritative_.edit_state.poles.find(node_id); pole != nullptr) {
      return pole->world_transform.position;
    }
    for (const Span& span : authoritative_.edit_state.spans.items()) {
      const Port* pa = authoritative_.edit_state.ports.find(span.port_a_id);
      const Port* pb = authoritative_.edit_state.ports.find(span.port_b_id);
      if (span.endpoint_node_a_id == node_id && pa != nullptr) {
        return pa->world_position;
      }
      if (span.endpoint_node_b_id == node_id && pb != nullptr) {
        return pb->world_position;
      }
    }
    return (port == nullptr) ? Vec3d{} : port->world_position;
  };
  out.edge_orientations = debug_.last_generation_edge_orientations;
  const bool has_saved_backbone = !authoritative_.backbone.nodes.empty() || !authoritative_.backbone.edges.empty();
  if (!debug_.last_generation_support_nodes.empty() || has_saved_backbone) {
    out.nodes = debug_.last_generation_support_nodes;
    out.nodes.reserve(out.nodes.size() + authoritative_.backbone.nodes.size());
    for (const SavedBackboneNode& saved_node : authoritative_.backbone.nodes) {
      if (saved_node.node_id == kInvalidObjectId) {
        continue;
      }
      SupportNode node{};
      node.node_id = saved_node.node_id;
      node.support_kind = saved_node.support_kind;
      node.position = saved_node.position;
      node.pole_id = saved_node.pole_id;
      node.saved_backbone_node_id = saved_node.node_id;
      node.has_source_edge = saved_node.has_source_edge;
      node.source_edge_node_a_id = saved_node.source_edge_node_a;
      node.source_edge_node_b_id = saved_node.source_edge_node_b;
      node.source_edge_t = saved_node.source_edge_t;
      node.bundle_modes = saved_node.bundle_modes;
      out.nodes.push_back(std::move(node));
    }

    std::unordered_map<ObjectId, std::vector<ObjectId>> bundles_by_saved_edge{};
    for (const SavedBackboneEdgeBundle& edge_bundle : authoritative_.backbone.edge_bundles) {
      if (edge_bundle.edge_id == kInvalidObjectId || edge_bundle.bundle_id == kInvalidObjectId) {
        continue;
      }
      bundles_by_saved_edge[edge_bundle.edge_id].push_back(edge_bundle.bundle_id);
    }
    out.edges.reserve(authoritative_.backbone.edges.size());
    for (const SavedBackboneEdge& saved_edge : authoritative_.backbone.edges) {
      if (saved_edge.edge_id == kInvalidObjectId || saved_edge.node_a == kInvalidObjectId ||
          saved_edge.node_b == kInvalidObjectId || saved_edge.node_a == saved_edge.node_b) {
        continue;
      }
      BackboneEdge edge{};
      edge.node_a = saved_edge.node_a;
      edge.node_b = saved_edge.node_b;
      if (const auto it_bundles = bundles_by_saved_edge.find(saved_edge.edge_id); it_bundles != bundles_by_saved_edge.end()) {
        edge.bundles = it_bundles->second;
      }
      out.edges.push_back(std::move(edge));
    }

    if (!has_saved_backbone) {
      // v1 scenes do not have a saved backbone graph, so public query still needs the span-derived edge view.
      const std::vector<BackboneEdge> pole_edges = BuildBackboneEdges();
      out.edges.insert(out.edges.end(), pole_edges.begin(), pole_edges.end());
    }

    struct EdgeKey {
      ObjectId a = kInvalidObjectId;
      ObjectId b = kInvalidObjectId;
      bool operator==(const EdgeKey& other) const { return a == other.a && b == other.b; }
    };
    struct EdgeKeyHash {
      std::size_t operator()(const EdgeKey& key) const {
        const std::size_t h1 = std::hash<ObjectId>{}(key.a);
        const std::size_t h2 = std::hash<ObjectId>{}(key.b);
        return h1 ^ (h2 + 0x9e3779b9 + (h1 << 6) + (h1 >> 2));
      }
    };
    struct EdgeAccum {
      ObjectId node_a = kInvalidObjectId;
      ObjectId node_b = kInvalidObjectId;
      std::unordered_set<ObjectId> bundles{};
    };
    std::unordered_map<EdgeKey, EdgeAccum, EdgeKeyHash> edge_map{};
    for (const BackboneEdge& edge : out.edges) {
      if (edge.node_a == kInvalidObjectId || edge.node_b == kInvalidObjectId || edge.node_a == edge.node_b) {
        continue;
      }
      const ObjectId a = std::min(edge.node_a, edge.node_b);
      const ObjectId b = std::max(edge.node_a, edge.node_b);
      EdgeAccum& acc = edge_map[{a, b}];
      acc.node_a = a;
      acc.node_b = b;
      for (ObjectId bundle_id : edge.bundles) {
        if (bundle_id != kInvalidObjectId) {
          acc.bundles.insert(bundle_id);
        }
      }
    }
    out.edges.clear();
    out.edges.reserve(edge_map.size());
    for (const auto& [_, acc] : edge_map) {
      BackboneEdge edge{};
      edge.node_a = acc.node_a;
      edge.node_b = acc.node_b;
      edge.bundles.assign(acc.bundles.begin(), acc.bundles.end());
      std::sort(edge.bundles.begin(), edge.bundles.end());
      out.edges.push_back(std::move(edge));
    }
    std::sort(out.edges.begin(), out.edges.end(), [](const BackboneEdge& lhs, const BackboneEdge& rhs) {
      if (lhs.node_a != rhs.node_a) {
        return lhs.node_a < rhs.node_a;
      }
      return lhs.node_b < rhs.node_b;
    });

    std::unordered_map<ObjectId, SupportNode> node_by_id{};
    node_by_id.reserve(out.nodes.size() + out.edges.size() * 2);
    for (const SupportNode& node : out.nodes) {
      if (node.node_id == kInvalidObjectId) {
        continue;
      }
      if (const auto it_existing = node_by_id.find(node.node_id); it_existing != node_by_id.end()) {
        SupportNode merged = node;
        const SupportNode& existing = it_existing->second;
        if (merged.path_point_index < 0) {
          merged.path_point_index = existing.path_point_index;
        }
        if (!merged.has_tangent_hint && existing.has_tangent_hint) {
          merged.has_tangent_hint = true;
          merged.tangent_hint = existing.tangent_hint;
        }
        if (merged.saved_backbone_node_id == kInvalidObjectId) {
          merged.saved_backbone_node_id = existing.saved_backbone_node_id;
        }
        if (!merged.has_source_edge && existing.has_source_edge) {
          merged.has_source_edge = true;
          merged.source_edge_node_a_id = existing.source_edge_node_a_id;
          merged.source_edge_node_b_id = existing.source_edge_node_b_id;
          merged.source_edge_t = existing.source_edge_t;
        }
        if (merged.bundle_modes.empty() && !existing.bundle_modes.empty()) {
          merged.bundle_modes = existing.bundle_modes;
        }
        it_existing->second = std::move(merged);
        continue;
      }
      node_by_id[node.node_id] = node;
    }
    auto ensure_node = [&](ObjectId node_id) {
      if (node_id == kInvalidObjectId || node_by_id.contains(node_id)) {
        return;
      }
      SupportNode node{};
      node.node_id = node_id;
      node.support_kind = SupportKind::kMidair;
      node.pole_id = kInvalidObjectId;
      if (const Pole* pole = authoritative_.edit_state.poles.find(node_id); pole != nullptr) {
        node.support_kind = SupportKind::kPole;
        node.position = pole->world_transform.position;
        node.pole_id = pole->id;
      } else {
        node.position = resolve_span_endpoint_position(node_id, nullptr);
      }
      node_by_id[node_id] = node;
    };
    for (const BackboneEdge& edge : out.edges) {
      ensure_node(edge.node_a);
      ensure_node(edge.node_b);
    }

    std::unordered_map<ObjectId, std::unordered_map<ObjectId, BackboneIncidentMeta>> incident_meta_by_node{};
    auto accumulate_incident_meta = [&](ObjectId node_id, ObjectId neighbor_id, std::uint64_t session_id,
                                        std::uint32_t generation_order) {
      BackboneIncidentMeta& acc = incident_meta_by_node[node_id][neighbor_id];
      if (session_id < acc.min_session_id ||
          (session_id == acc.min_session_id && generation_order < acc.min_generation_order)) {
        acc.min_session_id = session_id;
        acc.min_generation_order = generation_order;
      }
    };
    for (const Span& span : authoritative_.edit_state.spans.items()) {
      if (span.bundle_id == kInvalidObjectId) {
        continue;
      }
      const Port* pa = authoritative_.edit_state.ports.find(span.port_a_id);
      const Port* pb = authoritative_.edit_state.ports.find(span.port_b_id);
      const ObjectId node_a = resolve_span_endpoint_node(span, pa, true);
      const ObjectId node_b = resolve_span_endpoint_node(span, pb, false);
      if (pa == nullptr || pb == nullptr || node_a == kInvalidObjectId || node_b == kInvalidObjectId || node_a == node_b) {
        continue;
      }
      accumulate_incident_meta(node_a, node_b, span.generation.generation_session_id,
                               span.generation.generation_order);
      accumulate_incident_meta(node_b, node_a, span.generation.generation_session_id,
                               span.generation.generation_order);
    }

    std::unordered_map<ObjectId, ObjectId> existing_primary_neighbor_by_node{};
    std::unordered_map<ObjectId, std::uint64_t> existing_prioritized_session_by_node{};
    std::unordered_map<ObjectId, std::unordered_map<ObjectId, std::uint64_t>> existing_incident_session_by_node{};
    for (const JunctionInfo& junction : out.junctions) {
      existing_prioritized_session_by_node[junction.node_id] = junction.prioritized_session_id;
      for (const JunctionIncident& incident : junction.incidents) {
        existing_incident_session_by_node[junction.node_id][incident.neighbor_node_id] = incident.source_session_id;
        if (incident.primary) {
          existing_primary_neighbor_by_node[junction.node_id] = incident.neighbor_node_id;
        }
      }
    }

    std::unordered_map<ObjectId, std::unordered_set<ObjectId>> adjacency{};
    for (const BackboneEdge& edge : out.edges) {
      adjacency[edge.node_a].insert(edge.node_b);
      adjacency[edge.node_b].insert(edge.node_a);
    }

    if (!debug_.last_generation_junction_relations.empty()) {
      out.junctions = BuildJunctionsFromRelations(debug_.last_generation_junction_relations, adjacency, incident_meta_by_node,
                                                 existing_prioritized_session_by_node,
                                                 existing_incident_session_by_node);
    } else {
      out.junctions.clear();
      std::vector<ObjectId> junction_node_ids{};
      for (const auto& [node_id, neighbors] : adjacency) {
        if (neighbors.size() >= 3) {
          junction_node_ids.push_back(node_id);
        }
      }
      std::sort(junction_node_ids.begin(), junction_node_ids.end());

      auto normalize_dir = [](const Vec3d& v) -> Vec3d {
        const double len = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
        if (len <= 1e-9) {
          return {0.0, 0.0, 0.0};
        }
        return {v.x / len, v.y / len, v.z / len};
      };
      auto dot = [](const Vec3d& a, const Vec3d& b) { return a.x * b.x + a.y * b.y + a.z * b.z; };

      for (ObjectId node_id : junction_node_ids) {
        const auto it_neighbors = adjacency.find(node_id);
        if (it_neighbors == adjacency.end() || it_neighbors->second.size() < 3) {
          continue;
        }
        const Vec3d center = node_by_id[node_id].position;
        struct Candidate {
          ObjectId neighbor_id = kInvalidObjectId;
          std::uint64_t min_session_id = std::numeric_limits<std::uint64_t>::max();
          std::uint32_t min_generation_order = std::numeric_limits<std::uint32_t>::max();
          Vec3d dir{};
        };
        std::vector<Candidate> candidates{};
        candidates.reserve(it_neighbors->second.size());
        for (ObjectId neighbor_id : it_neighbors->second) {
          if (neighbor_id == kInvalidObjectId || !node_by_id.contains(neighbor_id)) {
            continue;
          }
          Candidate candidate{};
          candidate.neighbor_id = neighbor_id;
          if (const auto it_node = incident_meta_by_node.find(node_id); it_node != incident_meta_by_node.end()) {
            if (const auto it_meta = it_node->second.find(neighbor_id); it_meta != it_node->second.end()) {
              candidate.min_session_id = it_meta->second.min_session_id;
              candidate.min_generation_order = it_meta->second.min_generation_order;
            }
          }
          candidate.dir = normalize_dir(node_by_id[neighbor_id].position - center);
          candidates.push_back(candidate);
        }
        if (candidates.size() < 3) {
          continue;
        }
        std::sort(candidates.begin(), candidates.end(),
                  [](const Candidate& a, const Candidate& b) { return a.neighbor_id < b.neighbor_id; });

        int anchor_index = -1;
        if (const auto it_primary = existing_primary_neighbor_by_node.find(node_id);
            it_primary != existing_primary_neighbor_by_node.end()) {
          for (std::size_t i = 0; i < candidates.size(); ++i) {
            if (candidates[i].neighbor_id == it_primary->second) {
              anchor_index = static_cast<int>(i);
              break;
            }
          }
        }
        if (anchor_index < 0) {
          auto better_anchor = [&](int lhs, int rhs) {
            const Candidate& a = candidates[static_cast<std::size_t>(lhs)];
            const Candidate& b = candidates[static_cast<std::size_t>(rhs)];
            if (a.min_session_id != b.min_session_id) {
              return a.min_session_id < b.min_session_id;
            }
            if (a.min_generation_order != b.min_generation_order) {
              return a.min_generation_order < b.min_generation_order;
            }
            return a.neighbor_id < b.neighbor_id;
          };
          for (std::size_t i = 0; i < candidates.size(); ++i) {
            const int idx = static_cast<int>(i);
            if (anchor_index < 0 || better_anchor(idx, anchor_index)) {
              anchor_index = idx;
            }
          }
        }

        int opposite_index = -1;
        double best_straight = -2.0;
        for (std::size_t i = 0; i < candidates.size(); ++i) {
          const int idx = static_cast<int>(i);
          if (idx == anchor_index) {
            continue;
          }
          const double straight_score =
              dot(candidates[static_cast<std::size_t>(anchor_index)].dir,
                  Vec3d{-candidates[i].dir.x, -candidates[i].dir.y, -candidates[i].dir.z});
          if (straight_score > best_straight + 1e-9 ||
              (std::abs(straight_score - best_straight) <= 1e-9 &&
               (opposite_index < 0 || candidates[i].neighbor_id <
                                          candidates[static_cast<std::size_t>(opposite_index)].neighbor_id))) {
            best_straight = straight_score;
            opposite_index = idx;
          }
        }

        std::vector<int> order_indices{};
        order_indices.push_back(anchor_index);
        if (opposite_index >= 0 && opposite_index != anchor_index) {
          order_indices.push_back(opposite_index);
        }
        for (std::size_t i = 0; i < candidates.size(); ++i) {
          const int idx = static_cast<int>(i);
          if (idx == anchor_index || idx == opposite_index) {
            continue;
          }
          order_indices.push_back(idx);
        }
        std::sort(order_indices.begin() + ((opposite_index >= 0 && opposite_index != anchor_index) ? 2 : 1),
                  order_indices.end(), [&](int lhs, int rhs) {
                    const Candidate& a = candidates[static_cast<std::size_t>(lhs)];
                    const Candidate& b = candidates[static_cast<std::size_t>(rhs)];
                    if (a.min_session_id != b.min_session_id) {
                      return a.min_session_id < b.min_session_id;
                    }
                    if (a.min_generation_order != b.min_generation_order) {
                      return a.min_generation_order < b.min_generation_order;
                    }
                    return a.neighbor_id < b.neighbor_id;
                  });

        JunctionInfo junction{};
        junction.node_id = node_id;
        junction.prioritized_session_id = candidates[static_cast<std::size_t>(anchor_index)].min_session_id;
        if (junction.prioritized_session_id == std::numeric_limits<std::uint64_t>::max()) {
          junction.prioritized_session_id = 0;
          if (const auto it_prioritized = existing_prioritized_session_by_node.find(node_id);
              it_prioritized != existing_prioritized_session_by_node.end()) {
            junction.prioritized_session_id = it_prioritized->second;
          }
        }
        junction.incidents.reserve(order_indices.size());
        for (std::size_t rank = 0; rank < order_indices.size(); ++rank) {
          const Candidate& candidate = candidates[static_cast<std::size_t>(order_indices[rank])];
          JunctionIncident incident{};
          incident.neighbor_node_id = candidate.neighbor_id;
          incident.order = static_cast<int>(rank);
          incident.primary = (rank == 0);
          incident.source_session_id = candidate.min_session_id;
          if (incident.source_session_id == std::numeric_limits<std::uint64_t>::max()) {
            incident.source_session_id = 0;
            if (const auto it_existing_node = existing_incident_session_by_node.find(node_id);
                it_existing_node != existing_incident_session_by_node.end()) {
              if (const auto it_existing_source = it_existing_node->second.find(candidate.neighbor_id);
                  it_existing_source != it_existing_node->second.end()) {
                incident.source_session_id = it_existing_source->second;
              }
            }
          }
          junction.incidents.push_back(incident);
        }
        out.junctions.push_back(std::move(junction));
      }
      std::sort(out.junctions.begin(), out.junctions.end(),
                [](const JunctionInfo& a, const JunctionInfo& b) { return a.node_id < b.node_id; });
    }

    out.nodes.clear();
    out.nodes.reserve(node_by_id.size());
    for (const auto& [_, node] : node_by_id) {
      out.nodes.push_back(node);
    }
    std::sort(out.nodes.begin(), out.nodes.end(),
              [](const SupportNode& a, const SupportNode& b) { return a.node_id < b.node_id; });
    return out;
  }

  out.edges = BuildBackboneEdges();

  struct IncidentAccum {
    ObjectId neighbor_id = kInvalidObjectId;
    BackboneIncidentMeta meta{};
  };
  struct IncidentCandidate {
    ObjectId neighbor_id = kInvalidObjectId;
    std::uint64_t min_session_id = std::numeric_limits<std::uint64_t>::max();
    std::uint32_t min_generation_order = std::numeric_limits<std::uint32_t>::max();
    Vec3d dir{};
  };

  auto accumulate_incident = [&](ObjectId node_id, ObjectId neighbor_id, std::uint64_t session_id,
                                 std::uint32_t generation_order,
                                 std::unordered_map<ObjectId, std::unordered_map<ObjectId, IncidentAccum>>& map) {
    IncidentAccum& acc = map[node_id][neighbor_id];
    acc.neighbor_id = neighbor_id;
      if (session_id < acc.meta.min_session_id ||
          (session_id == acc.meta.min_session_id && generation_order < acc.meta.min_generation_order)) {
        acc.meta.min_session_id = session_id;
        acc.meta.min_generation_order = generation_order;
      }
    };

  std::unordered_map<ObjectId, std::unordered_map<ObjectId, IncidentAccum>> incident_map{};
  std::unordered_map<ObjectId, std::unordered_map<BundleKind, int>> node_bundle_degree{};
  for (const Span& span : authoritative_.edit_state.spans.items()) {
    if (span.bundle_id == kInvalidObjectId) {
      continue;
    }
    const Bundle* bundle = authoritative_.edit_state.bundles.find(span.bundle_id);
    if (bundle == nullptr) {
      continue;
    }
    const Port* pa = authoritative_.edit_state.ports.find(span.port_a_id);
    const Port* pb = authoritative_.edit_state.ports.find(span.port_b_id);
    const ObjectId node_a = resolve_span_endpoint_node(span, pa, true);
    const ObjectId node_b = resolve_span_endpoint_node(span, pb, false);
    if (pa == nullptr || pb == nullptr || node_a == kInvalidObjectId || node_b == kInvalidObjectId || node_a == node_b) {
      continue;
    }
    accumulate_incident(node_a, node_b, span.generation.generation_session_id,
                        span.generation.generation_order, incident_map);
    accumulate_incident(node_b, node_a, span.generation.generation_session_id,
                        span.generation.generation_order, incident_map);
    node_bundle_degree[node_a][bundle->bundle_template_id] += 1;
    node_bundle_degree[node_b][bundle->bundle_template_id] += 1;
  }

  std::vector<ObjectId> junction_nodes{};
  for (const auto& [node_id, incidents] : incident_map) {
    if (incidents.size() >= 3) {
      junction_nodes.push_back(node_id);
    }
  }
  std::sort(junction_nodes.begin(), junction_nodes.end(), [&](ObjectId a, ObjectId b) {
    auto min_session = [&](ObjectId node_id) -> std::uint64_t {
      std::uint64_t m = std::numeric_limits<std::uint64_t>::max();
      auto it = incident_map.find(node_id);
      if (it == incident_map.end()) {
        return m;
      }
      for (const auto& [_, acc] : it->second) {
        m = std::min(m, acc.meta.min_session_id);
      }
      return m;
    };
    const std::uint64_t sa = min_session(a);
    const std::uint64_t sb = min_session(b);
    if (sa != sb) {
      return sa < sb;
    }
    return a < b;
  });

  if (!debug_.last_generation_junction_relations.empty()) {
    std::unordered_map<ObjectId, std::unordered_set<ObjectId>> adjacency{};
    std::unordered_map<ObjectId, std::unordered_map<ObjectId, BackboneIncidentMeta>> incident_meta_by_node{};
    for (const auto& [node_id, incidents] : incident_map) {
      for (const auto& [neighbor_id, acc] : incidents) {
        adjacency[node_id].insert(neighbor_id);
        incident_meta_by_node[node_id][neighbor_id] = acc.meta;
      }
    }
    out.junctions = BuildJunctionsFromRelations(debug_.last_generation_junction_relations, adjacency, incident_meta_by_node,
                                                {}, {});
  } else {
    auto normalize_dir = [](const Vec3d& v) -> Vec3d {
      const double len = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
      if (len <= 1e-9) {
        return {0.0, 0.0, 0.0};
      }
      return {v.x / len, v.y / len, v.z / len};
    };
    auto dot = [](const Vec3d& a, const Vec3d& b) -> double { return a.x * b.x + a.y * b.y + a.z * b.z; };

    std::unordered_map<ObjectId, ObjectId> primary_neighbor_by_node{};
    for (ObjectId node_id : junction_nodes) {
      auto it_inc = incident_map.find(node_id);
      if (it_inc == incident_map.end()) {
        continue;
      }
      std::vector<IncidentCandidate> candidates{};
      candidates.reserve(it_inc->second.size());
      for (const auto& [neighbor_id, acc] : it_inc->second) {
        IncidentCandidate c{};
        c.neighbor_id = neighbor_id;
        c.min_session_id = acc.meta.min_session_id;
        c.min_generation_order = acc.meta.min_generation_order;
        c.dir = normalize_dir(resolve_span_endpoint_position(neighbor_id, nullptr) -
                              resolve_span_endpoint_position(node_id, nullptr));
        candidates.push_back(c);
      }
      if (candidates.size() < 3) {
        continue;
      }

      int anchor_index = -1;
      auto better_anchor = [&](int lhs, int rhs) {
        const auto& a = candidates[static_cast<std::size_t>(lhs)];
        const auto& b = candidates[static_cast<std::size_t>(rhs)];
        if (a.min_session_id != b.min_session_id) {
          return a.min_session_id < b.min_session_id;
        }
        if (a.min_generation_order != b.min_generation_order) {
          return a.min_generation_order < b.min_generation_order;
        }
        return a.neighbor_id < b.neighbor_id;
      };

      for (std::size_t i = 0; i < candidates.size(); ++i) {
        auto it_prev = primary_neighbor_by_node.find(candidates[i].neighbor_id);
        if (it_prev == primary_neighbor_by_node.end() || it_prev->second != node_id) {
          continue;
        }
        const int idx = static_cast<int>(i);
        if (anchor_index < 0 || better_anchor(idx, anchor_index)) {
          anchor_index = idx;
        }
      }
      if (anchor_index < 0) {
        for (std::size_t i = 0; i < candidates.size(); ++i) {
          const int idx = static_cast<int>(i);
          if (anchor_index < 0 || better_anchor(idx, anchor_index)) {
            anchor_index = idx;
          }
        }
      }

      int opposite_index = -1;
      double best_straight = -2.0;
      for (std::size_t i = 0; i < candidates.size(); ++i) {
        const int idx = static_cast<int>(i);
        if (idx == anchor_index) {
          continue;
        }
        const double straight_score =
            dot(candidates[static_cast<std::size_t>(anchor_index)].dir, Vec3d{-candidates[i].dir.x, -candidates[i].dir.y,
                                                                              -candidates[i].dir.z});
        if (straight_score > best_straight + 1e-9 ||
            (std::abs(straight_score - best_straight) <= 1e-9 &&
             candidates[i].neighbor_id < candidates[static_cast<std::size_t>(opposite_index < 0 ? idx : opposite_index)]
                                           .neighbor_id)) {
          best_straight = straight_score;
          opposite_index = idx;
        }
      }

      std::vector<int> ordered_indices{};
      ordered_indices.push_back(anchor_index);
      if (opposite_index >= 0 && opposite_index != anchor_index) {
        ordered_indices.push_back(opposite_index);
      }
      std::vector<int> tail{};
      for (std::size_t i = 0; i < candidates.size(); ++i) {
        const int idx = static_cast<int>(i);
        if (idx == anchor_index || idx == opposite_index) {
          continue;
        }
        tail.push_back(idx);
      }
      std::sort(tail.begin(), tail.end(), [&](int lhs, int rhs) {
        const auto& a = candidates[static_cast<std::size_t>(lhs)];
        const auto& b = candidates[static_cast<std::size_t>(rhs)];
        if (a.min_session_id != b.min_session_id) {
          return a.min_session_id < b.min_session_id;
        }
        if (a.min_generation_order != b.min_generation_order) {
          return a.min_generation_order < b.min_generation_order;
        }
        return a.neighbor_id < b.neighbor_id;
      });
      ordered_indices.insert(ordered_indices.end(), tail.begin(), tail.end());

      JunctionInfo junction{};
      junction.node_id = node_id;
      junction.prioritized_session_id = candidates[static_cast<std::size_t>(anchor_index)].min_session_id;
      for (std::size_t rank = 0; rank < ordered_indices.size(); ++rank) {
        const auto& c = candidates[static_cast<std::size_t>(ordered_indices[rank])];
        JunctionIncident inc{};
        inc.neighbor_node_id = c.neighbor_id;
        inc.order = static_cast<int>(rank);
        inc.primary = (rank == 0);
        inc.source_session_id = c.min_session_id;
        junction.incidents.push_back(inc);
      }
      primary_neighbor_by_node[node_id] = candidates[static_cast<std::size_t>(anchor_index)].neighbor_id;
      out.junctions.push_back(std::move(junction));
    }

    std::sort(out.junctions.begin(), out.junctions.end(),
              [](const JunctionInfo& a, const JunctionInfo& b) { return a.node_id < b.node_id; });
  }

  std::unordered_set<ObjectId> support_node_ids{};
  for (const BackboneEdge& edge : out.edges) {
    support_node_ids.insert(edge.node_a);
    support_node_ids.insert(edge.node_b);
  }
  for (const auto& [node_id, _] : incident_map) {
    support_node_ids.insert(node_id);
  }

  out.nodes.reserve(support_node_ids.size());
  for (ObjectId node_id : support_node_ids) {
    SupportNode node{};
    node.node_id = node_id;
    node.support_kind = SupportKind::kMidair;
    node.position = resolve_span_endpoint_position(node_id, nullptr);
    node.pole_id = kInvalidObjectId;
    if (const Pole* pole = authoritative_.edit_state.poles.find(node_id); pole != nullptr) {
      node.support_kind = SupportKind::kPole;
      node.position = pole->world_transform.position;
      node.pole_id = pole->id;
    }
    node.path_point_index = -1;

    auto it_deg = node_bundle_degree.find(node_id);
    if (it_deg != node_bundle_degree.end()) {
      node.bundle_modes.reserve(it_deg->second.size());
      for (const auto& [bundle_template_id, degree] : it_deg->second) {
        (void)degree;
        SupportNodeBundleMode mode{};
        mode.bundle_template_id = bundle_template_id;
        mode.mode = BundleNodeMode::kNotPresent;
        node.bundle_modes.push_back(mode);
      }
      std::sort(node.bundle_modes.begin(), node.bundle_modes.end(),
                [](const SupportNodeBundleMode& a, const SupportNodeBundleMode& b) {
                  return static_cast<int>(a.bundle_template_id) < static_cast<int>(b.bundle_template_id);
                });
    }

    out.nodes.push_back(std::move(node));
  }
  std::sort(out.nodes.begin(), out.nodes.end(),
            [](const SupportNode& a, const SupportNode& b) { return a.node_id < b.node_id; });
  return out;
}

std::vector<BackboneEdge> CoreState::BuildBackboneEdges() const {
  struct EdgeKey {
    ObjectId a = kInvalidObjectId;
    ObjectId b = kInvalidObjectId;
    bool operator==(const EdgeKey& other) const { return a == other.a && b == other.b; }
  };
  struct EdgeKeyHash {
    std::size_t operator()(const EdgeKey& key) const {
      const std::size_t h1 = std::hash<ObjectId>{}(key.a);
      const std::size_t h2 = std::hash<ObjectId>{}(key.b);
      return h1 ^ (h2 + 0x9e3779b9 + (h1 << 6) + (h1 >> 2));
    }
  };
  struct EdgeAgg {
    ObjectId a = kInvalidObjectId;
    ObjectId b = kInvalidObjectId;
    std::unordered_set<ObjectId> bundles{};
  };

  std::unordered_map<EdgeKey, EdgeAgg, EdgeKeyHash> edge_map{};
  auto resolve_span_endpoint_node = [&](const Span& span, const Port* port, bool is_a) -> ObjectId {
    const ObjectId explicit_node_id = is_a ? span.endpoint_node_a_id : span.endpoint_node_b_id;
    if (explicit_node_id != kInvalidObjectId) {
      return explicit_node_id;
    }
    return (port == nullptr) ? kInvalidObjectId : port->owner_pole_id;
  };

  for (const Span& span : authoritative_.edit_state.spans.items()) {
    if (span.bundle_id == kInvalidObjectId) {
      continue;
    }
    const Port* pa = authoritative_.edit_state.ports.find(span.port_a_id);
    const Port* pb = authoritative_.edit_state.ports.find(span.port_b_id);
    if (pa == nullptr || pb == nullptr) {
      continue;
    }
    const ObjectId node_a_raw = resolve_span_endpoint_node(span, pa, true);
    const ObjectId node_b_raw = resolve_span_endpoint_node(span, pb, false);
    if (node_a_raw == kInvalidObjectId || node_b_raw == kInvalidObjectId) {
      continue;
    }
    if (node_a_raw == node_b_raw) {
      continue;
    }

    const ObjectId node_a = std::min(node_a_raw, node_b_raw);
    const ObjectId node_b = std::max(node_a_raw, node_b_raw);
    const EdgeKey key{node_a, node_b};
    EdgeAgg& agg = edge_map[key];
    agg.a = node_a;
    agg.b = node_b;
    agg.bundles.insert(span.bundle_id);
  }

  std::vector<BackboneEdge> edges{};
  edges.reserve(edge_map.size());
  for (const auto& [key, agg] : edge_map) {
    (void)key;
    BackboneEdge edge{};
    edge.node_a = agg.a;
    edge.node_b = agg.b;
    edge.bundles.assign(agg.bundles.begin(), agg.bundles.end());
    std::sort(edge.bundles.begin(), edge.bundles.end());
    edges.push_back(std::move(edge));
  }
  std::sort(edges.begin(), edges.end(), [](const BackboneEdge& lhs, const BackboneEdge& rhs) {
    if (lhs.node_a != rhs.node_a) {
      return lhs.node_a < rhs.node_a;
    }
    return lhs.node_b < rhs.node_b;
  });
  return edges;
}

std::vector<ObjectId> CoreState::FindBackboneRoute(ObjectId start_node_id, ObjectId end_node_id) const {
  if (start_node_id == kInvalidObjectId || end_node_id == kInvalidObjectId) {
    return {};
  }
  if (start_node_id == end_node_id) {
    return {start_node_id};
  }

  auto saved_node_for_query = [&](ObjectId id) -> ObjectId {
    if (id == kInvalidObjectId) {
      return kInvalidObjectId;
    }
    const auto saved_it =
        std::find_if(authoritative_.backbone.nodes.begin(), authoritative_.backbone.nodes.end(),
                     [&](const SavedBackboneNode& node) { return node.node_id == id; });
    if (saved_it != authoritative_.backbone.nodes.end()) {
      return saved_it->node_id;
    }
    const auto pole_it = runtime_.backbone_index.pole_node.find(id);
    return pole_it == runtime_.backbone_index.pole_node.end() ? kInvalidObjectId : pole_it->second;
  };

  const ObjectId saved_start = saved_node_for_query(start_node_id);
  const ObjectId saved_end = saved_node_for_query(end_node_id);
  if (saved_start != kInvalidObjectId && saved_end != kInvalidObjectId && saved_start != saved_end) {
    std::unordered_map<ObjectId, std::vector<ObjectId>> saved_adjacency{};
    for (const SavedBackboneEdge& edge : authoritative_.backbone.edges) {
      if (edge.node_a == kInvalidObjectId || edge.node_b == kInvalidObjectId || edge.node_a == edge.node_b) {
        continue;
      }
      const auto bundles_it = runtime_.backbone_index.edge_bundles.find(edge.edge_id);
      if (bundles_it == runtime_.backbone_index.edge_bundles.end() || bundles_it->second.empty()) {
        continue;
      }
      saved_adjacency[edge.node_a].push_back(edge.node_b);
      saved_adjacency[edge.node_b].push_back(edge.node_a);
    }
    if (saved_adjacency.contains(saved_start) && saved_adjacency.contains(saved_end)) {
      std::queue<ObjectId> queue{};
      std::unordered_set<ObjectId> visited{};
      std::unordered_map<ObjectId, ObjectId> parent{};
      queue.push(saved_start);
      visited.insert(saved_start);

      bool found = false;
      while (!queue.empty() && !found) {
        const ObjectId node = queue.front();
        queue.pop();
        auto it = saved_adjacency.find(node);
        if (it == saved_adjacency.end()) {
          continue;
        }
        for (ObjectId next : it->second) {
          if (visited.contains(next)) {
            continue;
          }
          visited.insert(next);
          parent[next] = node;
          if (next == saved_end) {
            found = true;
            break;
          }
          queue.push(next);
        }
      }

      if (found) {
        std::vector<ObjectId> saved_path{};
        ObjectId cur = saved_end;
        saved_path.push_back(cur);
        while (cur != saved_start) {
          auto it = parent.find(cur);
          if (it == parent.end()) {
            saved_path.clear();
            break;
          }
          cur = it->second;
          saved_path.push_back(cur);
        }
        if (!saved_path.empty()) {
          std::reverse(saved_path.begin(), saved_path.end());
          std::vector<ObjectId> path{};
          path.reserve(saved_path.size());
          for (ObjectId node_id : saved_path) {
            const auto node_it =
                std::find_if(authoritative_.backbone.nodes.begin(), authoritative_.backbone.nodes.end(),
                             [&](const SavedBackboneNode& node) { return node.node_id == node_id; });
            if (node_it != authoritative_.backbone.nodes.end() && node_it->pole_id != kInvalidObjectId) {
              path.push_back(node_it->pole_id);
            } else {
              path.push_back(node_id);
            }
          }
          path.front() = start_node_id;
          path.back() = end_node_id;
          return path;
        }
      }
    }
  }

  const std::vector<BackboneEdge> edges = BuildBackboneEdges();
  std::unordered_map<ObjectId, std::vector<ObjectId>> adjacency{};
  for (const BackboneEdge& edge : edges) {
    if (edge.bundles.empty()) {
      continue;
    }
    adjacency[edge.node_a].push_back(edge.node_b);
    adjacency[edge.node_b].push_back(edge.node_a);
  }
  if (!adjacency.contains(start_node_id) || !adjacency.contains(end_node_id)) {
    return {};
  }

  std::queue<ObjectId> queue{};
  std::unordered_set<ObjectId> visited{};
  std::unordered_map<ObjectId, ObjectId> parent{};
  queue.push(start_node_id);
  visited.insert(start_node_id);

  bool found = false;
  while (!queue.empty() && !found) {
    const ObjectId node = queue.front();
    queue.pop();
    auto it = adjacency.find(node);
    if (it == adjacency.end()) {
      continue;
    }
    for (ObjectId next : it->second) {
      if (visited.contains(next)) {
        continue;
      }
      visited.insert(next);
      parent[next] = node;
      if (next == end_node_id) {
        found = true;
        break;
      }
      queue.push(next);
    }
  }

  if (!found) {
    return {};
  }
  std::vector<ObjectId> path{};
  ObjectId cur = end_node_id;
  path.push_back(cur);
  while (cur != start_node_id) {
    auto it = parent.find(cur);
    if (it == parent.end()) {
      return {};
    }
    cur = it->second;
    path.push_back(cur);
  }
  std::reverse(path.begin(), path.end());
  return path;
}

} // namespace wire::core
