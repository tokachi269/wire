#include "geometry.hpp"
#include "alignment.hpp"

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
  double remaining =
      std::clamp(distance_along_path_m, 0.0, length.value);
  for (const BezierSpan &span : alignment.spans) {
    const double span_length_m = internal::span_length(span);
    if (remaining <= span_length_m || &span == &alignment.spans.back()) {
      const double parameter =
          internal::span_parameter_at_length(span, remaining);
      const Vec2d tangent =
          normalize(internal::span_derivative(span, parameter));
      if (magnitude(tangent) <= distance_epsilon) {
        return Result<Vec2d>::Fail(
            CommitFailureCategory::kNotImplemented,
            "road cubic tangent control handle is degenerate");
      }
      return Result<Vec2d>::Ok(tangent);
    }
    remaining -= span_length_m;
  }
  return Result<Vec2d>::Fail(CommitFailureCategory::kInternalError,
                             "road tangent distance resolution fell through");
}

Result<Vec2d> lateral_at(const Path &alignment,
                         double distance_along_path_m) {
  const Result<double> length = PathLength(alignment);
  if (!length.ok)
    return Result<Vec2d>::Fail(length.failure_category, length.error);
  double remaining = std::clamp(distance_along_path_m, 0.0, length.value);
  for (std::size_t index = 0; index < alignment.spans.size(); ++index) {
    const BezierSpan &span = alignment.spans[index];
    const double span_length_m = internal::span_length(span);
    const bool internal_knot = index + 1 < alignment.spans.size() &&
                               std::abs(remaining - span_length_m) <=
                                   distance_epsilon;
    if (internal_knot) {
      const Vec2d incoming =
          normalize(internal::span_derivative(span, 1.0));
      const Vec2d outgoing = normalize(
          internal::span_derivative(alignment.spans[index + 1], 0.0));
      if (magnitude(incoming) <= distance_epsilon ||
          magnitude(outgoing) <= distance_epsilon) {
        return Result<Vec2d>::Fail(
            CommitFailureCategory::kNotImplemented,
            "road cubic tangent control handle is degenerate");
      }
      const Vec2d incoming_normal{-incoming.y, incoming.x};
      const Vec2d outgoing_normal{-outgoing.y, outgoing.x};
      const Vec2d miter = normalize(add(incoming_normal, outgoing_normal));
      const double projection = dot(miter, incoming_normal);
      if (magnitude(miter) <= distance_epsilon ||
          projection <= distance_epsilon) {
        return Result<Vec2d>::Fail(
            CommitFailureCategory::kNotImplemented,
            "road internal corner is too tight for an offset section");
      }
      return Result<Vec2d>::Ok(scale(miter, 1.0 / projection));
    }
    if (remaining <= span_length_m || index + 1 == alignment.spans.size()) {
      const double parameter =
          internal::span_parameter_at_length(span, remaining);
      const Vec2d tangent =
          normalize(internal::span_derivative(span, parameter));
      if (magnitude(tangent) <= distance_epsilon) {
        return Result<Vec2d>::Fail(
            CommitFailureCategory::kNotImplemented,
            "road cubic tangent control handle is degenerate");
      }
      return Result<Vec2d>::Ok(Vec2d{-tangent.y, tangent.x});
    }
    remaining -= span_length_m;
  }
  return Result<Vec2d>::Fail(CommitFailureCategory::kInternalError,
                             "road lateral distance resolution fell through");
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
