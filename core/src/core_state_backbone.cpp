#include "wire/core/core_state.hpp"

#include <algorithm>
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
  for (const Port& port : edit_state_.ports.items()) {
    if (port.owner_pole_id == pole_id) {
      detail.owned_ports.push_back(&port);
    }
  }
  for (const Anchor& anchor : edit_state_.anchors.items()) {
    if (anchor.owner_pole_id == pole_id) {
      detail.owned_anchors.push_back(&anchor);
    }
  }
  std::sort(detail.owned_ports.begin(), detail.owned_ports.end(),
            [](const Port* a, const Port* b) { return a->id < b->id; });
  std::sort(detail.owned_anchors.begin(), detail.owned_anchors.end(),
            [](const Anchor* a, const Anchor* b) { return a->id < b->id; });
  return detail;
}

const WireGroup* CoreState::GetWireGroup(ObjectId wire_group_id) const {
  return edit_state_.wire_groups.find(wire_group_id);
}

const WireLane* CoreState::GetWireLane(ObjectId wire_lane_id) const {
  return edit_state_.wire_lanes.find(wire_lane_id);
}

std::vector<ObjectId> CoreState::GetSpansByWireGroup(ObjectId wire_group_id) const {
  std::vector<ObjectId> span_ids;
  if (wire_group_id == kInvalidObjectId) {
    return span_ids;
  }
  for (const Span& span : edit_state_.spans.items()) {
    if (span.wire_group_id == wire_group_id) {
      span_ids.push_back(span.id);
    }
  }
  return span_ids;
}

std::vector<ObjectId> CoreState::GetWireLanesByGroup(ObjectId wire_group_id) const {
  std::vector<ObjectId> lane_ids;
  if (wire_group_id == kInvalidObjectId) {
    return lane_ids;
  }
  for (const WireLane& lane : edit_state_.wire_lanes.items()) {
    if (lane.wire_group_id == wire_group_id) {
      lane_ids.push_back(lane.id);
    }
  }
  return lane_ids;
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
    std::unordered_set<ObjectId> groups{};
  };

  std::unordered_map<EdgeKey, EdgeAgg, EdgeKeyHash> edge_map{};

  for (const Span& span : edit_state_.spans.items()) {
    if (span.wire_group_id == kInvalidObjectId) {
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
    agg.groups.insert(span.wire_group_id);
  }

  std::vector<BackboneEdge> edges{};
  edges.reserve(edge_map.size());
  for (const auto& [key, agg] : edge_map) {
    (void)key;
    BackboneEdge edge{};
    edge.node_a = agg.a;
    edge.node_b = agg.b;
    edge.groups.assign(agg.groups.begin(), agg.groups.end());
    std::sort(edge.groups.begin(), edge.groups.end());
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
    if (edge.groups.empty()) {
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
