#include "geometry.hpp"

#include <algorithm>
#include <cmath>

namespace city::road::internal {

bool is_finite(double value) { return std::isfinite(value); }

bool is_finite(Vec2d value) { return is_finite(value.x) && is_finite(value.y); }

Vec2d add(Vec2d a, Vec2d b) { return {a.x + b.x, a.y + b.y}; }

Vec2d subtract(Vec2d a, Vec2d b) { return {a.x - b.x, a.y - b.y}; }

Vec2d scale(Vec2d value, double factor) {
  return {value.x * factor, value.y * factor};
}

double dot(Vec2d a, Vec2d b) { return a.x * b.x + a.y * b.y; }

double cross(Vec2d a, Vec2d b) { return a.x * b.y - a.y * b.x; }

double magnitude(Vec2d value) { return std::hypot(value.x, value.y); }

Vec2d normalize(Vec2d value) {
  const double length = magnitude(value);
  if (length <= distance_epsilon)
    return {};
  return {value.x / length, value.y / length};
}

Vec3d to3(Vec2d value, double z) { return {value.x, value.y, z}; }

ApproachKey make_approach_key(const RoadSegment &segment, RoadNodeId node_id) {
  return ApproachKey{
      node_id,
      segment.id,
      node_id == segment.node_a ? EndpointRole::kStart : EndpointRole::kEnd,
  };
}

Result<Vec2d> inward_tangent(const RoadSegment &segment, const Path &alignment,
                             const ApproachKey &key) {
  if (key.segment_id != segment.id ||
      (key.endpoint_role == EndpointRole::kStart &&
       key.node_id != segment.node_a) ||
      (key.endpoint_role == EndpointRole::kEnd &&
       key.node_id != segment.node_b)) {
    return Result<Vec2d>::Fail(
        CommitFailureCategory::kInternalError,
        "road approach key does not match segment endpoint");
  }
  const BezierSpan &span = key.endpoint_role == EndpointRole::kStart
                               ? alignment.spans.front()
                               : alignment.spans.back();
  const std::array<Vec2d, 3> candidates =
      key.endpoint_role == EndpointRole::kStart
          ? std::array<Vec2d, 3>{subtract(span.p1, span.p0),
                                 subtract(span.p2, span.p0),
                                 subtract(span.p3, span.p0)}
          : std::array<Vec2d, 3>{subtract(span.p2, span.p3),
                                 subtract(span.p1, span.p3),
                                 subtract(span.p0, span.p3)};
  for (const Vec2d candidate : candidates) {
    const Vec2d tangent = normalize(candidate);
    if (magnitude(tangent) > distance_epsilon) {
      return Result<Vec2d>::Ok(tangent);
    }
  }
  return Result<Vec2d>::Fail(CommitFailureCategory::kNotImplemented,
                             "road approach tangent is degenerate");
}

Result<Vec2d> tangent_at(const Path &alignment,
                         double distance_along_path_m) {
  const Result<double> length = PathLength(alignment);
  if (!length.ok)
    return Result<Vec2d>::Fail(length.failure_category, length.error);
  const double delta = std::min(0.1, length.value);
  const double before_distance =
      std::max(0.0, distance_along_path_m - delta);
  const double after_distance =
      std::min(length.value, distance_along_path_m + delta);
  if (after_distance - before_distance <= distance_epsilon) {
    return Result<Vec2d>::Fail(CommitFailureCategory::kInternalError,
                               "road tangent distance is degenerate");
  }
  const Result<Vec2d> before = EvaluatePath(alignment, before_distance);
  const Result<Vec2d> after = EvaluatePath(alignment, after_distance);
  if (!before.ok || !after.ok) {
    return Result<Vec2d>::Fail(CommitFailureCategory::kInternalError,
                               "road tangent could not be evaluated");
  }
  return Result<Vec2d>::Ok(normalize(subtract(after.value, before.value)));
}

double endpoint_distance(const ApproachKey &key, double length_m) {
  return key.endpoint_role == EndpointRole::kStart ? 0.0 : length_m;
}

void sort_unique_distances(std::vector<double> &distances) {
  std::sort(distances.begin(), distances.end());
  distances.erase(std::unique(distances.begin(), distances.end(),
                             [](double a, double b) {
                               return std::abs(a - b) <= distance_epsilon;
                             }),
                 distances.end());
}

} // namespace city::road::internal
