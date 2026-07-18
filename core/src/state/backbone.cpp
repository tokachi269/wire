#include "wire/core/core_state.hpp"
#include "wire/core/core_view.hpp"
#include "wire/core/coord_utils.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace wire::core {

namespace {

bool is_open_row_for_edge(const SavedBackboneRowKey& row_key, ObjectId node_id, ObjectId edge_id) {
  return row_key.node_id == node_id && row_key.source_is_open && row_key.source_edge_a == edge_id &&
         row_key.source_edge_b == kInvalidObjectId;
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
  if (node.support_kind == SupportKind::kExternal) {
    node.bundle_modes = std::move(bundle_modes);
  }
  authoritative_.backbone.nodes.push_back(node);
  if (pole_id != kInvalidObjectId) {
    runtime_.backbone_index.pole_node[pole_id] = node.node_id;
  }
  return node.node_id;
}

void CoreState::cache_support_group(SupportGroupDecision decision, LoweredSupportGroupPlacement placement) {
  const LoweredSupportGroupKey key = LoweredSupportGroupKeyFromDecision(decision);
  if (key.owner_pole_id == kInvalidObjectId || key.support_group_id < 0) {
    return;
  }
  runtime_.cache_state.span_layout_cache.support_groups.decision.by_key[key] = std::move(decision);
  runtime_.cache_state.span_layout_cache.support_groups.placement.by_key[key] = std::move(placement);
}

EditResult<bool> CoreState::bind_backbone_node_bundle_modes(
    ObjectId node_id, const std::vector<SupportNodeBundleMode>& bundle_modes) {
  EditResult<bool> out{};
  auto node_it = std::find_if(authoritative_.backbone.nodes.begin(), authoritative_.backbone.nodes.end(),
                              [&](const SavedBackboneNode& node) { return node.node_id == node_id; });
  if (node_it == authoritative_.backbone.nodes.end()) {
    out.error = "backbone graph: saved node missing for bundle policy";
    return out;
  }
  if (node_it->support_kind == SupportKind::kExternal) {
    node_it->bundle_modes = bundle_modes;
  }
  out.ok = true;
  out.value = true;
  return out;
}

EditResult<bool> CoreState::bind_backbone_node_path_point_index(ObjectId node_id, int path_point_index) {
  EditResult<bool> out{};
  auto node_it = std::find_if(authoritative_.backbone.nodes.begin(), authoritative_.backbone.nodes.end(),
                              [&](const SavedBackboneNode& node) { return node.node_id == node_id; });
  if (node_it == authoritative_.backbone.nodes.end()) {
    out.error = "backbone graph: saved node missing for route index";
    return out;
  }
  node_it->path_point_index = path_point_index;
  out.ok = true;
  out.value = true;
  return out;
}

SavedBackboneEdgeRef CoreState::save_backbone_edge(ObjectId node_a, ObjectId node_b, std::size_t route,
                                                   std::size_t order, const Vec3d& dir, double lateral_offset_m) {
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
  edge.lateral_offset_m = lateral_offset_m;
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
  const BackboneEdgeBundleKey key{edge_id, bundle_id};
  if (const auto existing = runtime_.backbone_index.edge_bundle_by_edge_and_bundle.find(key);
      existing != runtime_.backbone_index.edge_bundle_by_edge_and_bundle.end()) {
    return existing->second;
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
  const std::size_t position = authoritative_.backbone.edge_bundles.size() - 1;
  index_add(runtime_.backbone_index.edge_bundles, edge_id, item.edge_bundle_id);
  runtime_.backbone_index.edge_bundle_by_edge_and_bundle.emplace(key, item.edge_bundle_id);
  runtime_.backbone_index.edge_bundle_positions[item.edge_bundle_id] = position;
  index_add(runtime_.backbone_index.bundle_edge, bundle_id, edge_id);
  return item.edge_bundle_id;
}

EditResult<bool> CoreState::bind_backbone_span(ObjectId edge_bundle_id, std::size_t lane_index, ObjectId span_id) {
  EditResult<bool> out{};
  if (edge_bundle_id == kInvalidObjectId || span_id == kInvalidObjectId) {
    out.error = "invalid backbone span binding";
    return out;
  }
  const auto position = runtime_.backbone_index.edge_bundle_positions.find(edge_bundle_id);
  if (position == runtime_.backbone_index.edge_bundle_positions.end() ||
      position->second >= authoritative_.backbone.edge_bundles.size()) {
    out.error = "invalid backbone span binding";
    return out;
  }
  SavedBackboneEdgeBundle* found = &authoritative_.backbone.edge_bundles[position->second];
  if (found->edge_bundle_id != edge_bundle_id) {
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
                                               std::size_t lane_index, BundleTemplateId bundle_template_id,
                                               PortKind port_kind, PortLayer port_layer, int placement_band_id,
                                               double layout_yaw_deg, ObjectId port_id) {
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
          binding.port_layer != port_layer || binding.placement_band_id != placement_band_id ||
          std::abs(NormalizeYawDeg(binding.layout_yaw_deg - layout_yaw_deg)) > 1e-9) {
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
  binding.placement_band_id = placement_band_id;
  binding.layout_yaw_deg = NormalizeYawDeg(layout_yaw_deg);
  binding.port_id = port_id;
  const std::size_t index = authoritative_.backbone.port_bindings.size();
  authoritative_.backbone.port_bindings.push_back(binding);
  runtime_.backbone_index.edge_bundle_ports[edge_bundle_id].push_back(index);
  runtime_.backbone_index.port_bindings_by_port[port_id].push_back(index);
  out.value = true;
  out.ok = true;
  return out;
}

EditResult<bool> CoreState::promote_backbone_open_port_binding_exact(
    ObjectId edge_bundle_id, const SavedBackboneRowKey& old_open_key, std::size_t lane_index,
    const SavedBackboneRowKey& pair_key, double layout_yaw_deg, ObjectId port_id) {
  EditResult<bool> out{};
  out.value = false;
  if (edge_bundle_id == kInvalidObjectId || old_open_key.node_id == kInvalidObjectId ||
      !old_open_key.source_is_open || old_open_key.source_edge_a == kInvalidObjectId ||
      pair_key.source_is_open || pair_key.node_id == kInvalidObjectId ||
      pair_key.source_edge_a == kInvalidObjectId || pair_key.source_edge_b == kInvalidObjectId ||
      port_id == kInvalidObjectId) {
    out.ok = true;
    return out;
  }
  std::size_t match_index = static_cast<std::size_t>(-1);
  for (std::size_t i = 0; i < authoritative_.backbone.port_bindings.size(); ++i) {
    const SavedBackbonePortBinding& binding = authoritative_.backbone.port_bindings[i];
    if (binding.edge_bundle_id != edge_bundle_id || binding.row_key != old_open_key ||
        binding.lane_index != lane_index || binding.port_id != port_id) {
      continue;
    }
    if (match_index != static_cast<std::size_t>(-1)) {
      out.error = "backbone unsupported: ambiguous exact promoted open row binding";
      return out;
    }
    match_index = i;
  }
  if (match_index == static_cast<std::size_t>(-1)) {
    out.ok = true;
    return out;
  }
  SavedBackbonePortBinding& binding = authoritative_.backbone.port_bindings[match_index];
  binding.row_key = pair_key;
  binding.layout_yaw_deg = NormalizeYawDeg(layout_yaw_deg);
  out.value = true;
  out.ok = true;
  return out;
}

EditResult<bool> CoreState::bind_backbone_row_continuity(ObjectId node_id,
                                                         ObjectId edge_bundle_a,
                                                         std::size_t lane_a,
                                                         ObjectId edge_bundle_b,
                                                         std::size_t lane_b) {
  EditResult<bool> out{};
  if (node_id == kInvalidObjectId || edge_bundle_a == kInvalidObjectId ||
      edge_bundle_b == kInvalidObjectId || edge_bundle_a == edge_bundle_b) {
    out.error = "invalid backbone row continuity";
    return out;
  }
  if (view().backbone_node(node_id) == nullptr ||
      view().backbone_edge_bundle(edge_bundle_a) == nullptr ||
      view().backbone_edge_bundle(edge_bundle_b) == nullptr) {
    out.error = "invalid backbone row continuity";
    return out;
  }
  for (const SavedBackboneRowContinuity& continuity : authoritative_.backbone.row_continuities) {
    if (continuity.node_id != node_id) {
      continue;
    }
    const bool forward = continuity.a.edge_bundle_id == edge_bundle_a &&
                         continuity.a.lane_index == lane_a &&
                         continuity.b.edge_bundle_id == edge_bundle_b &&
                         continuity.b.lane_index == lane_b;
    const bool reverse = continuity.a.edge_bundle_id == edge_bundle_b &&
                         continuity.a.lane_index == lane_b &&
                         continuity.b.edge_bundle_id == edge_bundle_a &&
                         continuity.b.lane_index == lane_a;
    if (forward || reverse) {
      out.value = true;
      out.ok = true;
      return out;
    }
  }
  SavedBackboneRowContinuity continuity{};
  continuity.node_id = node_id;
  continuity.a.edge_bundle_id = edge_bundle_a;
  continuity.a.lane_index = lane_a;
  continuity.b.edge_bundle_id = edge_bundle_b;
  continuity.b.lane_index = lane_b;
  authoritative_.backbone.row_continuities.push_back(continuity);
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

  std::vector<BundleTemplateId> selected_template_ids = options.selected_bundle_template_ids;
  std::sort(selected_template_ids.begin(), selected_template_ids.end(),
            [](BundleTemplateId a, BundleTemplateId b) { return a < b; });
  selected_template_ids.erase(std::unique(selected_template_ids.begin(), selected_template_ids.end()),
                              selected_template_ids.end());

  struct SelectedTemplatePolicy {
    BundleTemplateId id = kInvalidBundleTemplateId;
    const BundleTemplate* bundle_template = nullptr;
    bool allow_midair_path = true;
  };
  std::vector<SelectedTemplatePolicy> selected_templates{};
  selected_templates.reserve(selected_template_ids.size());
  for (BundleTemplateId bundle_template_id : selected_template_ids) {
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
  auto sqr_dist_xy = [](const Vec3d& a, const Vec3d& b) {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    return dx * dx + dy * dy;
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
    for (const SupportNode& node : debug_.pending_support_nodes) {
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
                                         return node.node_id == node_id;
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

  auto selected_bundle_modes = [&](bool apply_midair_policy) {
    std::vector<SupportNodeBundleMode> modes{};
    modes.reserve(selected_templates.size());
    for (const SelectedTemplatePolicy& selected : selected_templates) {
      SupportNodeBundleMode mode{};
      mode.bundle_template_id = selected.id;
      mode.mode = (!apply_midair_policy || !options.enforce_midair_template_policy || selected.allow_midair_path)
                      ? BundleNodeMode::kPassThrough
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
      node.bundle_modes = selected_bundle_modes(true);
      debug_.pending_support_nodes.push_back(node);
      std::sort(debug_.pending_support_nodes.begin(), debug_.pending_support_nodes.end(),
                [](const SupportNode& a, const SupportNode& b) { return a.node_id < b.node_id; });
      result.value.resolved_node_id = node.node_id;
    } else {
      result.value.resolved_node_id = kInvalidObjectId;
    }
    result.ok = true;
  };

  if (pick.hit_kind == PickHitKind::kExternal) {
    resolve_ownerless_surface_pick(SupportKind::kExternal);
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
    result.value.position = pick.hit_pos_world;
    result.value.support_kind = SupportKind::kPole;
    const auto saved_by_node_id =
        std::find_if(authoritative_.backbone.nodes.begin(), authoritative_.backbone.nodes.end(),
                     [&](const SavedBackboneNode& node) { return node.node_id == pick.hit_id; });
    if (saved_by_node_id != authoritative_.backbone.nodes.end()) {
      result.value.resolved_node_id = saved_by_node_id->node_id;
      result.value.support_kind = saved_by_node_id->support_kind;
      result.value.position = saved_by_node_id->position;
      if (!selected_templates.empty() && saved_by_node_id->pole_id == kInvalidObjectId) {
        SupportNode node{};
        node.node_id = debug_.next_virtual_support_node_id++;
        node.support_kind = saved_by_node_id->support_kind;
        node.position = saved_by_node_id->position;
        node.pole_id = kInvalidObjectId;
        node.saved_backbone_node_id = saved_by_node_id->node_id;
        node.path_point_index = -1;
        node.bundle_modes = selected_bundle_modes(true);
        debug_.pending_support_nodes.push_back(node);
        std::sort(debug_.pending_support_nodes.begin(), debug_.pending_support_nodes.end(),
                  [](const SupportNode& a, const SupportNode& b) { return a.node_id < b.node_id; });
        result.value.resolved_node_id = node.node_id;
        result.value.position = node.position;
      }
      result.ok = true;
      return result;
    }

    const Pole* pole = authoritative_.edit_state.poles.find(pick.hit_id);
    if (pole != nullptr) {
      const auto saved_pole_it =
          std::find_if(authoritative_.backbone.nodes.begin(), authoritative_.backbone.nodes.end(),
                       [&](const SavedBackboneNode& saved) {
                         return saved.pole_id == pole->id && saved.support_kind == SupportKind::kPole;
                       });
      if (saved_pole_it != authoritative_.backbone.nodes.end()) {
        result.value.resolved_node_id = saved_pole_it->node_id;
        result.value.support_kind = SupportKind::kPole;
        result.value.position = saved_pole_it->position;
        result.ok = true;
        return result;
      }
      if (!selected_templates.empty() || options.create_midair_node_set) {
        SupportNode node{};
        node.node_id = debug_.next_virtual_support_node_id++;
        node.support_kind = SupportKind::kPole;
        node.position = pole->world_transform.position;
        node.pole_id = pole->id;
        node.path_point_index = -1;
        node.bundle_modes = selected_bundle_modes(false);
        debug_.pending_support_nodes.push_back(node);
        std::sort(debug_.pending_support_nodes.begin(), debug_.pending_support_nodes.end(),
                  [](const SupportNode& a, const SupportNode& b) { return a.node_id < b.node_id; });
        result.value.resolved_node_id = node.node_id;
        result.value.support_kind = SupportKind::kPole;
        result.value.position = node.position;
        result.ok = true;
        return result;
      }
    }

    result.value.resolved_node_id = pick.hit_id;
    (void)resolve_node_info(pick.hit_id, &result.value.support_kind, &result.value.position);
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
    const double da2 = sqr_dist_xy(pick.hit_pos_world, endpoint_a);
    const double db2 = sqr_dist_xy(pick.hit_pos_world, endpoint_b);
    if ((da2 <= snap_r2 && node_a_id != kInvalidObjectId) || (db2 <= snap_r2 && node_b_id != kInvalidObjectId)) {
      const bool use_a = (da2 <= db2);
      ObjectId resolved_node_id = use_a ? node_a_id : node_b_id;
      SupportKind resolved_kind = SupportKind::kPole;
      Vec3d resolved_position = use_a ? endpoint_a : endpoint_b;
      (void)resolve_node_info(resolved_node_id, &resolved_kind, &resolved_position);
      const auto saved_it = std::find_if(authoritative_.backbone.nodes.begin(), authoritative_.backbone.nodes.end(),
                                         [&](const SavedBackboneNode& node) {
                                           return node.node_id == resolved_node_id &&
                                                  node.pole_id != kInvalidObjectId &&
                                                  resolved_kind == SupportKind::kPole;
                                         });
      if (saved_it != authoritative_.backbone.nodes.end()) {
        resolved_node_id = saved_it->pole_id;
        resolved_position = saved_it->position;
      }
      result.value.resolution = PickBranchResolutionKind::kNode;
      result.value.resolved_node_id = resolved_node_id;
      result.value.position = resolved_position;
      result.value.support_kind = resolved_kind;
      result.value.snapped_from_segment_endpoint = true;
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
  for (const SupportNode& node : debug_.pending_support_nodes) {
    if (node.support_kind != SupportKind::kMidair) {
      continue;
    }
    if (sqr_dist(node.position, branch_position) > kReuseEps2) {
      continue;
    }
    if (options.create_midair_node && has_endpoints) {
      for (SupportNode& mutable_node : debug_.pending_support_nodes) {
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
  debug_.pending_support_nodes.push_back(midair);
  std::sort(debug_.pending_support_nodes.begin(), debug_.pending_support_nodes.end(),
            [](const SupportNode& a, const SupportNode& b) { return a.node_id < b.node_id; });

  result.value.resolution = PickBranchResolutionKind::kMidair;
  result.value.resolved_node_id = midair.node_id;
  result.value.support_kind = midair.support_kind;
  result.value.position = midair.position;
  result.ok = true;
  return result;
}

BackboneResult CoreState::SavedBackboneResult() const {
  BackboneResult out{};
  out.edge_orientations = debug_.last_generation_edge_orientations;

  const bool has_saved_backbone = !authoritative_.backbone.nodes.empty() || !authoritative_.backbone.edges.empty();
  if (!has_saved_backbone) {
    return out;
  }

  out.nodes.reserve(authoritative_.backbone.nodes.size());
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
    node.path_point_index = saved_node.path_point_index;
    node.bundle_modes = saved_node.bundle_modes;
    out.nodes.push_back(std::move(node));
  }
  std::sort(out.nodes.begin(), out.nodes.end(),
            [](const SupportNode& lhs, const SupportNode& rhs) { return lhs.node_id < rhs.node_id; });

  std::unordered_map<ObjectId, std::vector<ObjectId>> bundles_by_edge{};
  for (const SavedBackboneEdgeBundle& edge_bundle : authoritative_.backbone.edge_bundles) {
    if (edge_bundle.edge_id == kInvalidObjectId || edge_bundle.bundle_id == kInvalidObjectId) {
      continue;
    }
    bundles_by_edge[edge_bundle.edge_id].push_back(edge_bundle.bundle_id);
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
    if (const auto it = bundles_by_edge.find(saved_edge.edge_id); it != bundles_by_edge.end()) {
      edge.bundles = it->second;
      std::sort(edge.bundles.begin(), edge.bundles.end());
      edge.bundles.erase(std::unique(edge.bundles.begin(), edge.bundles.end()), edge.bundles.end());
    }
    out.edges.push_back(std::move(edge));
  }
  std::sort(out.edges.begin(), out.edges.end(), [](const BackboneEdge& lhs, const BackboneEdge& rhs) {
    if (lhs.node_a != rhs.node_a) {
      return lhs.node_a < rhs.node_a;
    }
    return lhs.node_b < rhs.node_b;
  });

  std::unordered_map<ObjectId, std::vector<ObjectId>> adjacency{};
  for (const BackboneEdge& edge : out.edges) {
    adjacency[edge.node_a].push_back(edge.node_b);
    adjacency[edge.node_b].push_back(edge.node_a);
  }
  std::vector<ObjectId> junction_node_ids{};
  for (auto& [node_id, neighbors] : adjacency) {
    std::sort(neighbors.begin(), neighbors.end());
    neighbors.erase(std::unique(neighbors.begin(), neighbors.end()), neighbors.end());
    if (neighbors.size() >= 3) {
      junction_node_ids.push_back(node_id);
    }
  }
  std::sort(junction_node_ids.begin(), junction_node_ids.end());
  out.junctions.reserve(junction_node_ids.size());
  for (ObjectId node_id : junction_node_ids) {
    const auto it = adjacency.find(node_id);
    if (it == adjacency.end()) {
      continue;
    }
    JunctionInfo junction{};
    junction.node_id = node_id;
    junction.incidents.reserve(it->second.size());
    for (std::size_t i = 0; i < it->second.size(); ++i) {
      JunctionIncident incident{};
      incident.neighbor_node_id = it->second[i];
      incident.order = static_cast<int>(i);
      incident.primary = (i == 0);
      incident.source_session_id = 0;
      junction.incidents.push_back(incident);
    }
    out.junctions.push_back(std::move(junction));
  }
  return out;
}

std::vector<BackboneEdge> CoreState::SavedBackboneEdges() const {
  return SavedBackboneResult().edges;
}
std::vector<ObjectId> CoreState::FindSavedBackboneRoute(ObjectId start_node_id, ObjectId end_node_id) const {
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

  return {};
}

} // namespace wire::core
