#include "scene_query.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>

namespace {

wire::core::Vec3d FromRaylib(const Vector3& raylib_xyz) {
  return wire::core::Vec3d{
      static_cast<double>(raylib_xyz.x),
      static_cast<double>(raylib_xyz.z),
      static_cast<double>(raylib_xyz.y),
  };
}

constexpr double kBranchPickNodeHitRadiusWorld = 0.75;
constexpr double kBranchPickSegmentHitRadiusWorld = 1.25;

} // namespace

bool TryPickGroundPoint(const Camera3D& camera, double ue_plane_z, wire::core::Vec3d* out_ue_point) {
  if (out_ue_point == nullptr) {
    return false;
  }
  const Vector2 mouse = GetMousePosition();
  const Ray ray = GetMouseRay(mouse, camera);
  if (std::abs(ray.direction.y) <= 1e-6f) {
    return false;
  }
  const float plane_y = static_cast<float>(ue_plane_z);
  const float t = (plane_y - ray.position.y) / ray.direction.y;
  if (t < 0.0f) {
    return false;
  }
  const Vector3 hit{
      ray.position.x + ray.direction.x * t,
      ray.position.y + ray.direction.y * t,
      ray.position.z + ray.direction.z * t,
  };
  *out_ue_point = FromRaylib(hit);
  return true;
}

double DistancePointToSegmentSquared(const wire::core::Vec3d& p, const wire::core::Vec3d& a, const wire::core::Vec3d& b,
                                     double* out_t, wire::core::Vec3d* out_closest) {
  const wire::core::Vec3d ab = b - a;
  const double ab2 = ab.x * ab.x + ab.y * ab.y + ab.z * ab.z;
  double t = 0.0;
  if (ab2 > 1e-12) {
    const wire::core::Vec3d ap = p - a;
    t = (ap.x * ab.x + ap.y * ab.y + ap.z * ab.z) / ab2;
    t = std::clamp(t, 0.0, 1.0);
  }
  const wire::core::Vec3d closest{a.x + ab.x * t, a.y + ab.y * t, a.z + ab.z * t};
  if (out_t != nullptr) {
    *out_t = t;
  }
  if (out_closest != nullptr) {
    *out_closest = closest;
  }
  const wire::core::Vec3d d = p - closest;
  return d.x * d.x + d.y * d.y + d.z * d.z;
}

wire::core::PickResult ViewerSceneQuery::Raycast(const wire::core::CoreState& state, const Camera3D& camera,
                                                 double draw_plane_z) const {
  wire::core::PickResult pick{};
  wire::core::Vec3d ground{};
  if (!TryPickGroundPoint(camera, draw_plane_z, &ground)) {
    pick.hit_kind = wire::core::PickHitKind::kEmpty;
    return pick;
  }
  pick.hit_kind = wire::core::PickHitKind::kGround;
  pick.hit_pos_world = ground;

  struct NodeCandidate {
    wire::core::ObjectId node_id = wire::core::kInvalidObjectId;
    wire::core::SupportKind support_kind = wire::core::SupportKind::kPole;
    wire::core::Vec3d position{};
    double d2 = std::numeric_limits<double>::max();
  };
  NodeCandidate best_node{};

  const wire::core::BackboneResult backbone = state.BuildBackboneResult();
  std::unordered_map<wire::core::ObjectId, wire::core::Vec3d> backbone_node_positions{};
  backbone_node_positions.reserve(backbone.nodes.size());
  for (const wire::core::SupportNode& node : backbone.nodes) {
    if (node.node_id == wire::core::kInvalidObjectId) {
      continue;
    }
    backbone_node_positions[node.node_id] = node.position;
    const wire::core::Vec3d d = node.position - ground;
    const double d2 = d.x * d.x + d.y * d.y + d.z * d.z;
    if (d2 < best_node.d2) {
      best_node.node_id = node.node_id;
      best_node.support_kind = node.support_kind;
      best_node.position = node.position;
      best_node.d2 = d2;
    }
  }

  struct SegmentCandidate {
    wire::core::ObjectId span_id = wire::core::kInvalidObjectId;
    double d2 = std::numeric_limits<double>::max();
    wire::core::Vec3d closest{};
    wire::core::ObjectId node_a_id = wire::core::kInvalidObjectId;
    wire::core::ObjectId node_b_id = wire::core::kInvalidObjectId;
    wire::core::Vec3d endpoint_a{};
    wire::core::Vec3d endpoint_b{};
  };
  SegmentCandidate best_segment{};
  for (const wire::core::BackboneEdge& edge : backbone.edges) {
    const auto it_a = backbone_node_positions.find(edge.node_a);
    const auto it_b = backbone_node_positions.find(edge.node_b);
    if (it_a == backbone_node_positions.end() || it_b == backbone_node_positions.end()) {
      continue;
    }
    wire::core::Vec3d closest{};
    const double d2 = DistancePointToSegmentSquared(ground, it_a->second, it_b->second, nullptr, &closest);
    if (d2 >= best_segment.d2) {
      continue;
    }
    best_segment.span_id = wire::core::kInvalidObjectId;
    best_segment.d2 = d2;
    best_segment.closest = closest;
    best_segment.node_a_id = edge.node_a;
    best_segment.node_b_id = edge.node_b;
    best_segment.endpoint_a = it_a->second;
    best_segment.endpoint_b = it_b->second;
  }

  const bool node_hit = best_node.node_id != wire::core::kInvalidObjectId &&
                        best_node.d2 <= (kBranchPickNodeHitRadiusWorld * kBranchPickNodeHitRadiusWorld);
  const bool segment_hit = best_segment.node_a_id != wire::core::kInvalidObjectId &&
                           best_segment.d2 <= (kBranchPickSegmentHitRadiusWorld * kBranchPickSegmentHitRadiusWorld);
  if (node_hit && (!segment_hit || best_node.d2 <= best_segment.d2)) {
    pick.hit_kind = (best_node.support_kind == wire::core::SupportKind::kBuilding) ? wire::core::PickHitKind::kBuilding
                                                                                    : wire::core::PickHitKind::kNode;
    pick.hit_id = best_node.node_id;
    pick.hit_pos_world = best_node.position;
    return pick;
  }
  if (segment_hit) {
    pick.hit_kind = wire::core::PickHitKind::kSegment;
    pick.hit_id = best_segment.span_id;
    pick.hit_pos_world = best_segment.closest;
    pick.has_segment_endpoints = true;
    pick.segment_node_a_id = best_segment.node_a_id;
    pick.segment_node_b_id = best_segment.node_b_id;
    pick.segment_endpoint_a_world = best_segment.endpoint_a;
    pick.segment_endpoint_b_world = best_segment.endpoint_b;
    return pick;
  }
  return pick;
}

std::string PickTargetLabel(const wire::core::PickResult& pick) {
  if (pick.hit_kind == wire::core::PickHitKind::kSegment && pick.hit_id == wire::core::kInvalidObjectId &&
      pick.segment_node_a_id != wire::core::kInvalidObjectId && pick.segment_node_b_id != wire::core::kInvalidObjectId) {
    return std::string("backbone-edge ") + std::to_string(static_cast<unsigned long long>(pick.segment_node_a_id)) + "-" +
           std::to_string(static_cast<unsigned long long>(pick.segment_node_b_id));
  }
  if (pick.hit_id == wire::core::kInvalidObjectId) {
    return "none";
  }
  return std::to_string(static_cast<unsigned long long>(pick.hit_id));
}

