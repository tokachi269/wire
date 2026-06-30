#include "scene_query.hpp"
#include "backbone_plane.hpp"
#include "host_coords.hpp"
#include "wire/core/coord_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>

namespace {

struct UeRay {
  wire::core::Vec3d origin{};
  wire::core::Vec3d direction{};
};

constexpr double kBranchPickNodeHitRadiusWorld = 0.75;
constexpr double kBranchPickSegmentHitRadiusWorld = 1.25;

double DistancePointToRaySquared(const wire::core::Vec3d& p, const UeRay& ray) {
  const wire::core::Vec3d to_point = p - ray.origin;
  const double t = std::max(0.0, wire::core::Dot(to_point, ray.direction));
  const wire::core::Vec3d closest = ray.origin + wire::core::Vec3d{
                                                     ray.direction.x * t,
                                                     ray.direction.y * t,
                                                     ray.direction.z * t,
                                                 };
  return wire::core::LengthSquared(p - closest);
}

double DistanceRayToSegmentSquared(const UeRay& ray, const wire::core::Vec3d& a, const wire::core::Vec3d& b,
                                   wire::core::Vec3d* out_point_on_segment) {
  const wire::core::Vec3d u = ray.direction;
  const wire::core::Vec3d v = b - a;
  const wire::core::Vec3d w0 = ray.origin - a;
  const double a_dot = wire::core::Dot(u, u);
  const double b_dot = wire::core::Dot(u, v);
  const double c_dot = wire::core::Dot(v, v);
  const double d_dot = wire::core::Dot(u, w0);
  const double e_dot = wire::core::Dot(v, w0);
  const double denom = a_dot * c_dot - b_dot * b_dot;

  double s = 0.0;
  double t = 0.0;
  if (denom <= 1e-12) {
    t = std::clamp(e_dot / std::max(c_dot, 1e-12), 0.0, 1.0);
    s = std::max(0.0, b_dot * t - d_dot);
  } else {
    s = (b_dot * e_dot - c_dot * d_dot) / denom;
    t = (a_dot * e_dot - b_dot * d_dot) / denom;
    if (s < 0.0) {
      s = 0.0;
      t = std::clamp(e_dot / std::max(c_dot, 1e-12), 0.0, 1.0);
    } else if (t < 0.0) {
      t = 0.0;
      s = std::max(0.0, -d_dot / std::max(a_dot, 1e-12));
    } else if (t > 1.0) {
      t = 1.0;
      s = std::max(0.0, (b_dot - d_dot) / std::max(a_dot, 1e-12));
    }
  }

  const wire::core::Vec3d point_ray = ray.origin + wire::core::Vec3d{u.x * s, u.y * s, u.z * s};
  const wire::core::Vec3d point_seg = a + wire::core::Vec3d{v.x * t, v.y * t, v.z * t};
  if (out_point_on_segment != nullptr) {
    *out_point_on_segment = point_seg;
  }
  return wire::core::LengthSquared(point_ray - point_seg);
}

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
  *out_ue_point = HostWorldToInternal(hit);
  return true;
}

wire::core::PickResult ViewerSceneQuery::Raycast(const wire::core::CoreView& view, const Camera3D& camera,
                                                 double draw_plane_z) const {
  wire::core::PickResult pick{};

  const Vector2 mouse = GetMousePosition();
  const Ray raylib_ray = GetMouseRay(mouse, camera);
  UeRay ray{};
  ray.origin = HostWorldToInternal(raylib_ray.position);
  ray.direction = HostWorldToInternal(raylib_ray.direction);
  if (!wire::core::Normalize(&ray.direction)) {
    pick.hit_kind = wire::core::PickHitKind::kEmpty;
    return pick;
  }

  wire::core::Vec3d ground{};
  if (TryPickGroundPoint(camera, draw_plane_z, &ground)) {
    pick.hit_kind = wire::core::PickHitKind::kGround;
    pick.hit_pos_world = ground;
  } else {
    pick.hit_kind = wire::core::PickHitKind::kEmpty;
  }

  struct NodeCandidate {
    wire::core::ObjectId node_id = wire::core::kInvalidObjectId;
    wire::core::SupportKind support_kind = wire::core::SupportKind::kPole;
    wire::core::Vec3d position{};
    double d2 = std::numeric_limits<double>::max();
  };
  NodeCandidate best_node{};

  const wire::core::BackboneResult backbone = view.saved_backbone_result();
  std::unordered_map<wire::core::ObjectId, wire::core::Vec3d> backbone_node_positions{};
  backbone_node_positions.reserve(backbone.nodes.size());
  for (const wire::core::SupportNode& node : backbone.nodes) {
    if (node.node_id == wire::core::kInvalidObjectId) {
      continue;
    }
    const wire::core::Vec3d flattened = ProjectBackbonePointToDisplayPlane(node.position);
    backbone_node_positions[node.node_id] = flattened;
    const double d2 = DistancePointToRaySquared(flattened, ray);
    if (d2 < best_node.d2) {
      best_node.node_id = node.node_id;
      best_node.support_kind = node.support_kind;
      best_node.position = flattened;
      best_node.d2 = d2;
    }
  }

  struct SegmentCandidate {
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
    const double d2 = DistanceRayToSegmentSquared(ray, it_a->second, it_b->second, &closest);
    if (d2 >= best_segment.d2) {
      continue;
    }
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
    pick.hit_kind = (best_node.support_kind == wire::core::SupportKind::kExternal) ? wire::core::PickHitKind::kExternal
                                                                                    : wire::core::PickHitKind::kNode;
    pick.hit_id = best_node.node_id;
    pick.hit_pos_world = best_node.position;
    return pick;
  }
  if (segment_hit) {
    pick.hit_kind = wire::core::PickHitKind::kSegment;
    pick.hit_id = wire::core::kInvalidObjectId;
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
