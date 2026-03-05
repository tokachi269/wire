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

BackboneResult CoreState::BuildBackboneResult() const {
  BackboneResult out{};
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
  for (const Span& span : edit_state_.spans.items()) {
    if (span.bundle_id == kInvalidObjectId) {
      continue;
    }
    const Port* pa = edit_state_.ports.find(span.port_a_id);
    const Port* pb = edit_state_.ports.find(span.port_b_id);
    if (pa == nullptr || pb == nullptr || pa->owner_pole_id == kInvalidObjectId || pb->owner_pole_id == kInvalidObjectId ||
        pa->owner_pole_id == pb->owner_pole_id) {
      continue;
    }
    accumulate_incident(pa->owner_pole_id, pb->owner_pole_id, span.generation.generation_session_id,
                        span.generation.generation_order, incident_map);
    accumulate_incident(pb->owner_pole_id, pa->owner_pole_id, span.generation.generation_session_id,
                        span.generation.generation_order, incident_map);
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
    const Pole* center = edit_state_.poles.find(node_id);
    if (center == nullptr) {
      continue;
    }

    std::vector<IncidentCandidate> candidates{};
    candidates.reserve(it_inc->second.size());
    for (const auto& [neighbor_id, acc] : it_inc->second) {
      const Pole* neighbor = edit_state_.poles.find(neighbor_id);
      if (neighbor == nullptr) {
        continue;
      }
      IncidentCandidate c{};
      c.neighbor_id = neighbor_id;
      c.min_session_id = acc.min_session_id;
      c.min_generation_order = acc.min_generation_order;
      c.dir = normalize_dir(neighbor->world_transform.position - center->world_transform.position);
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

  for (const Span& span : edit_state_.spans.items()) {
    if (span.bundle_id == kInvalidObjectId) {
      continue;
    }
    const Port* pa = edit_state_.ports.find(span.port_a_id);
    const Port* pb = edit_state_.ports.find(span.port_b_id);
    if (pa == nullptr || pb == nullptr) {
      continue;
    }
    if (pa->owner_pole_id == kInvalidObjectId || pb->owner_pole_id == kInvalidObjectId) {
      continue;
    }
    if (pa->owner_pole_id == pb->owner_pole_id) {
      continue;
    }

    const ObjectId node_a = std::min(pa->owner_pole_id, pb->owner_pole_id);
    const ObjectId node_b = std::max(pa->owner_pole_id, pb->owner_pole_id);
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
