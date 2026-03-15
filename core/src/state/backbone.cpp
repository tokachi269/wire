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

CoreState::PoleDetailInfo CoreState::GetPoleDetail(ObjectId pole_id) const {
  PoleDetailInfo detail{};
  detail.pole = edit_state_.poles.find(pole_id);
  if (detail.pole == nullptr) {
    return detail;
  }
  detail.pole_type = find_pole_type(detail.pole->pole_type_id);
  if (const auto it = relation_index_.ports_by_pole.find(pole_id); it != relation_index_.ports_by_pole.end()) {
    for (ObjectId port_id : it->second) {
      if (const Port* port = edit_state_.ports.find(port_id); port != nullptr) {
        detail.owned_ports.push_back(port);
      }
    }
  }
  if (const auto it = relation_index_.anchors_by_pole.find(pole_id); it != relation_index_.anchors_by_pole.end()) {
    for (ObjectId anchor_id : it->second) {
      if (const Anchor* anchor = edit_state_.anchors.find(anchor_id); anchor != nullptr) {
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
  auto it = relation_index_.spans_by_bundle.find(bundle_id);
  if (it == relation_index_.spans_by_bundle.end()) {
    return span_ids;
  }
  span_ids.reserve(it->second.size());
  for (ObjectId span_id : it->second) {
    if (edit_state_.spans.find(span_id) != nullptr) {
      span_ids.push_back(span_id);
    }
  }
  std::sort(span_ids.begin(), span_ids.end());
  span_ids.erase(std::unique(span_ids.begin(), span_ids.end()), span_ids.end());
  return span_ids;
}

EditResult<CoreState::ResolveBranchPickResult>
CoreState::ResolveBranchPick(const PickResult& pick) {
  return ResolveBranchPick(pick, ResolveBranchPickOptions{});
}

EditResult<CoreState::ResolveBranchPickResult>
CoreState::ResolveBranchPick(const PickResult& pick, const ResolveBranchPickOptions& options) {
  EditResult<ResolveBranchPickResult> result{};
  if (options.snap_radius_world < 0.0) {
    result.error = "snap_radius_world must be >= 0";
    return result;
  }

  std::vector<BundleKind> selected_template_ids = options.selected_bundle_template_ids;
  if (selected_template_ids.empty()) {
    selected_template_ids.push_back(BundleKind::kLowVoltage);
  }
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
    for (const SupportNode& node : last_generation_support_nodes_) {
      if (node.node_id != node_id) {
        continue;
      }
      *out_kind = node.support_kind;
      *out_position = node.position;
      return true;
    }
    const Pole* pole = edit_state_.poles.find(node_id);
    if (pole != nullptr) {
      *out_kind = SupportKind::kPole;
      *out_position = pole->world_transform.position;
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

    if (const Span* span = edit_state_.spans.find(pick.hit_id); span != nullptr) {
      const Port* pa = edit_state_.ports.find(span->port_a_id);
      const Port* pb = edit_state_.ports.find(span->port_b_id);
      if (pa != nullptr && pb != nullptr) {
        auto resolve_endpoint = [&](bool is_a, const Port* port, ObjectId* out_node_id, Vec3d* out_pos) {
          const ObjectId explicit_node_id = is_a ? span->endpoint_node_a_id : span->endpoint_node_b_id;
          const ObjectId node_id = (explicit_node_id != kInvalidObjectId) ? explicit_node_id : port->owner_pole_id;
          *out_node_id = node_id;
          *out_pos = port->world_position;
          SupportKind kind = SupportKind::kPole;
          Vec3d node_pos{};
          if (node_id != kInvalidObjectId && resolve_node_info(node_id, &kind, &node_pos)) {
            *out_pos = node_pos;
          } else if (port->owner_pole_id != kInvalidObjectId) {
            if (const Pole* pole = edit_state_.poles.find(port->owner_pole_id); pole != nullptr) {
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

  if (pick.hit_kind == PickHitKind::kNode || pick.hit_kind == PickHitKind::kBuilding) {
    if (pick.hit_id == kInvalidObjectId) {
      result.error = "node pick must include a valid hit_id";
      return result;
    }
    result.value.resolution = PickBranchResolutionKind::kNode;
    result.value.resolved_node_id = pick.hit_id;
    result.value.position = pick.hit_pos_world;
    result.value.support_kind = SupportKind::kPole;
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

  if (options.enforce_midair_template_policy) {
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
  for (const SupportNode& node : last_generation_support_nodes_) {
    if (node.support_kind != SupportKind::kMidair) {
      continue;
    }
    if (sqr_dist(node.position, pick.hit_pos_world) > kReuseEps2) {
      continue;
    }
    if (options.create_midair_node && has_endpoints) {
      for (SupportNode& mutable_node : last_generation_support_nodes_) {
        if (mutable_node.node_id != node.node_id) {
          continue;
        }
        mutable_node.has_source_edge = true;
        mutable_node.source_edge_node_a_id = node_a_id;
        mutable_node.source_edge_node_b_id = node_b_id;
        mutable_node.source_edge_t = segment_t_xy(pick.hit_pos_world, endpoint_a, endpoint_b);
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

  if (!options.create_midair_node) {
    result.value.resolution = PickBranchResolutionKind::kMidair;
    result.value.resolved_node_id = kInvalidObjectId;
    result.value.support_kind = SupportKind::kMidair;
    result.value.position = pick.hit_pos_world;
    result.ok = true;
    return result;
  }

  SupportNode midair{};
  midair.node_id = next_virtual_support_node_id_++;
  midair.support_kind = SupportKind::kMidair;
  midair.position = pick.hit_pos_world;
  midair.pole_id = kInvalidObjectId;
  if (has_endpoints) {
    midair.has_source_edge = true;
    midair.source_edge_node_a_id = node_a_id;
    midair.source_edge_node_b_id = node_b_id;
    midair.source_edge_t = segment_t_xy(pick.hit_pos_world, endpoint_a, endpoint_b);
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
  last_generation_support_nodes_.push_back(midair);
  std::sort(last_generation_support_nodes_.begin(), last_generation_support_nodes_.end(),
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
    for (const SupportNode& node : last_generation_support_nodes_) {
      if (node.node_id == node_id) {
        return node.position;
      }
    }
    if (const Pole* pole = edit_state_.poles.find(node_id); pole != nullptr) {
      return pole->world_transform.position;
    }
    for (const Span& span : edit_state_.spans.items()) {
      const Port* pa = edit_state_.ports.find(span.port_a_id);
      const Port* pb = edit_state_.ports.find(span.port_b_id);
      if (span.endpoint_node_a_id == node_id && pa != nullptr) {
        return pa->world_position;
      }
      if (span.endpoint_node_b_id == node_id && pb != nullptr) {
        return pb->world_position;
      }
    }
    return (port == nullptr) ? Vec3d{} : port->world_position;
  };
  out.edge_orientations = last_generation_edge_orientations_;
  if (!last_generation_support_nodes_.empty()) {
    out.nodes = last_generation_support_nodes_;
    // Merge in span-derived pole edges so BuildBackboneResult observes full-network continuity across sessions.
    const std::vector<BackboneEdge> pole_edges = BuildBackboneEdges();
    out.edges.insert(out.edges.end(), pole_edges.begin(), pole_edges.end());

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
      if (const Pole* pole = edit_state_.poles.find(node_id); pole != nullptr) {
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

    struct IncidentAccum {
      std::uint64_t min_session_id = std::numeric_limits<std::uint64_t>::max();
      std::uint32_t min_generation_order = std::numeric_limits<std::uint32_t>::max();
    };
    std::unordered_map<ObjectId, std::unordered_map<ObjectId, IncidentAccum>> incident_meta_by_node{};
    auto accumulate_incident_meta = [&](ObjectId node_id, ObjectId neighbor_id, std::uint64_t session_id,
                                        std::uint32_t generation_order) {
      IncidentAccum& acc = incident_meta_by_node[node_id][neighbor_id];
      if (session_id < acc.min_session_id ||
          (session_id == acc.min_session_id && generation_order < acc.min_generation_order)) {
        acc.min_session_id = session_id;
        acc.min_generation_order = generation_order;
      }
    };
    for (const Span& span : edit_state_.spans.items()) {
      if (span.bundle_id == kInvalidObjectId) {
        continue;
      }
      const Port* pa = edit_state_.ports.find(span.port_a_id);
      const Port* pb = edit_state_.ports.find(span.port_b_id);
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

    auto normalize_dir = [](const Vec3d& v) -> Vec3d {
      const double len = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
      if (len <= 1e-9) {
        return {0.0, 0.0, 0.0};
      }
      return {v.x / len, v.y / len, v.z / len};
    };
    auto dot = [](const Vec3d& a, const Vec3d& b) { return a.x * b.x + a.y * b.y + a.z * b.z; };

    out.junctions.clear();
    std::vector<ObjectId> junction_node_ids{};
    for (const auto& [node_id, neighbors] : adjacency) {
      if (neighbors.size() >= 3) {
        junction_node_ids.push_back(node_id);
      }
    }
    std::sort(junction_node_ids.begin(), junction_node_ids.end());

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
      bool used_neighbor_continuity = false;
      if (const auto it_primary = existing_primary_neighbor_by_node.find(node_id);
          it_primary != existing_primary_neighbor_by_node.end()) {
        for (std::size_t i = 0; i < candidates.size(); ++i) {
          if (candidates[i].neighbor_id == it_primary->second) {
            anchor_index = static_cast<int>(i);
            used_neighbor_continuity = true;
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
      junction.used_neighbor_continuity = used_neighbor_continuity;
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
    std::uint64_t min_session_id = std::numeric_limits<std::uint64_t>::max();
    std::uint32_t min_generation_order = std::numeric_limits<std::uint32_t>::max();
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
    if (session_id < acc.min_session_id ||
        (session_id == acc.min_session_id && generation_order < acc.min_generation_order)) {
      acc.min_session_id = session_id;
      acc.min_generation_order = generation_order;
    }
  };

  std::unordered_map<ObjectId, std::unordered_map<ObjectId, IncidentAccum>> incident_map{};
  std::unordered_map<ObjectId, std::unordered_map<BundleKind, int>> node_bundle_degree{};
  for (const Span& span : edit_state_.spans.items()) {
    if (span.bundle_id == kInvalidObjectId) {
      continue;
    }
    const Bundle* bundle = edit_state_.bundles.find(span.bundle_id);
    if (bundle == nullptr) {
      continue;
    }
    const Port* pa = edit_state_.ports.find(span.port_a_id);
    const Port* pb = edit_state_.ports.find(span.port_b_id);
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
        m = std::min(m, acc.min_session_id);
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
      c.min_session_id = acc.min_session_id;
      c.min_generation_order = acc.min_generation_order;
      c.dir = normalize_dir(resolve_span_endpoint_position(neighbor_id, nullptr) -
                            resolve_span_endpoint_position(node_id, nullptr));
      candidates.push_back(c);
    }
    if (candidates.size() < 3) {
      continue;
    }

    int anchor_index = -1;
    bool used_neighbor_continuity = false;
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
        used_neighbor_continuity = true;
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
    junction.used_neighbor_continuity = used_neighbor_continuity;
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
    if (const Pole* pole = edit_state_.poles.find(node_id); pole != nullptr) {
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

  for (const Span& span : edit_state_.spans.items()) {
    if (span.bundle_id == kInvalidObjectId) {
      continue;
    }
    const Port* pa = edit_state_.ports.find(span.port_a_id);
    const Port* pb = edit_state_.ports.find(span.port_b_id);
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
