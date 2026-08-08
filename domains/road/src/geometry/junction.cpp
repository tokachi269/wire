#include "junction.hpp"

#include "geometry.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <map>

namespace city::road::internal {
namespace {

constexpr int kConnectionCurveSamples = 8;
constexpr int kJunctionCurveSamples = 6;

Vec3d add3(Vec3d a, Vec3d b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }

Vec3d scale3(Vec3d value, double factor) {
  return {value.x * factor, value.y * factor, value.z * factor};
}

Vec3d subtract3(Vec3d a, Vec3d b) {
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}

Vec3d gate_point_at(const ConnectionGate &gate, double section_lateral_m,
                    double height_m) {
  const double lateral_m = gate.approach.endpoint_role == EndpointRole::kEnd
                               ? -section_lateral_m
                               : section_lateral_m;
  return {
      gate.position.x + gate.lateral.x * lateral_m,
      gate.position.y + gate.lateral.y * lateral_m,
      gate.position.z + height_m,
  };
}

Vec3d boundary_point(const ConnectionGate &gate,
                     const SectionBoundarySample &boundary) {
  const double lateral_m = gate.approach.endpoint_role == EndpointRole::kEnd
                               ? -boundary.lateral_m
                               : boundary.lateral_m;
  return {
      gate.position.x + gate.lateral.x * lateral_m,
      gate.position.y + gate.lateral.y * lateral_m,
      gate.position.z + boundary.height_m,
  };
}

Vec3d cubic_point(Vec3d p0, Vec3d p1, Vec3d p2, Vec3d p3, double t) {
  const double u = 1.0 - t;
  return add3(add3(scale3(p0, u * u * u), scale3(p1, 3.0 * u * u * t)),
              add3(scale3(p2, 3.0 * u * t * t), scale3(p3, t * t * t)));
}

Vec3d cubic_tangent(Vec3d p0, Vec3d p1, Vec3d p2, Vec3d p3, double t) {
  const double u = 1.0 - t;
  return add3(
      add3(scale3(subtract3(p1, p0), 3.0 * u * u),
           scale3(subtract3(p2, p1), 6.0 * u * t)),
      scale3(subtract3(p3, p2), 3.0 * t * t));
}

struct curve_frame {
  Vec3d position{};
  Vec3d tangent{};
  Vec3d lateral{};
};

std::vector<curve_frame>
resolved_curve_frames(Vec3d a, Vec3d b, Vec3d tangent_a, Vec3d tangent_b,
                      double control_m, int samples) {
  const double chord_m = std::hypot(b.x - a.x, b.y - a.y);
  const Vec3d start_tangent = scale3(tangent_a, -1.0);
  const double effective_control_m = std::min(control_m, chord_m / 3.0);
  const Vec3d c1 = add3(a, scale3(start_tangent, effective_control_m));
  const Vec3d c2 = add3(b, scale3(tangent_b, -effective_control_m));
  std::vector<curve_frame> frames{};
  frames.reserve(static_cast<std::size_t>(samples) + 1);
  for (int i = 0; i <= samples; ++i) {
    const double t = static_cast<double>(i) / samples;
    const Vec3d derivative = cubic_tangent(a, c1, c2, b, t);
    const double horizontal_length =
        std::hypot(derivative.x, derivative.y);
    if (horizontal_length <= 1e-12)
      return {};
    const Vec3d tangent{derivative.x / horizontal_length,
                        derivative.y / horizontal_length, 0.0};
    frames.push_back(curve_frame{
        cubic_point(a, c1, c2, b, t),
        tangent,
        Vec3d{-tangent.y, tangent.x, 0.0},
    });
  }
  return frames;
}

double projected_offset(Vec3d point, Vec3d origin, Vec3d axis) {
  const Vec3d delta = subtract3(point, origin);
  return delta.x * axis.x + delta.y * axis.y + delta.z * axis.z;
}

std::vector<Vec3d>
sweep_boundary(const std::vector<curve_frame> &frames, Vec3d start,
               Vec3d end) {
  const double start_lateral =
      projected_offset(start, frames.front().position, frames.front().lateral);
  const double end_lateral =
      projected_offset(end, frames.back().position, frames.back().lateral);
  const double start_height = start.z - frames.front().position.z;
  const double end_height = end.z - frames.back().position.z;
  std::vector<Vec3d> points{};
  points.reserve(frames.size());
  for (std::size_t index = 0; index < frames.size(); ++index) {
    const double t =
        static_cast<double>(index) / static_cast<double>(frames.size() - 1);
    const double lateral =
        start_lateral + (end_lateral - start_lateral) * t;
    const double height = start_height + (end_height - start_height) * t;
    Vec3d point =
        add3(frames[index].position, scale3(frames[index].lateral, lateral));
    point.z += height;
    points.push_back(point);
  }
  return points;
}

struct boundary_token {
  std::uint64_t id = 0;
  BoundaryRole role = BoundaryRole::kOuterEdge;
  std::size_t occurrence = 0;

  bool operator<(const boundary_token &other) const {
    return std::tie(id, role, occurrence) <
           std::tie(other.id, other.role, other.occurrence);
  }
};

std::vector<boundary_token>
boundary_tokens(const std::vector<SectionBoundarySample> &boundaries) {
  std::map<std::pair<std::uint64_t, BoundaryRole>, std::size_t> occurrences{};
  std::vector<boundary_token> tokens{};
  tokens.reserve(boundaries.size());
  for (const SectionBoundarySample &boundary : boundaries) {
    const auto group = std::pair{boundary.boundary_id, boundary.role};
    tokens.push_back(boundary_token{boundary.boundary_id, boundary.role,
                                    occurrences[group]++});
  }
  return tokens;
}

Result<ConnectionGeometry> connection_geometry_from_gates(
    RoadNodeId node_id, const ConnectionGate &first,
    const ConnectionGate &second, const SectionEvaluation &first_section,
    const SectionEvaluation &second_section, double corner_control_m) {
  if (first_section.surface_styles != second_section.surface_styles) {
    return Result<ConnectionGeometry>::Fail(
        CommitFailureCategory::kNotImplemented,
        "road connection requires identical explicit surface mappings");
  }
  const std::vector<boundary_token> first_tokens =
      boundary_tokens(first.boundaries);
  const std::vector<boundary_token> second_tokens =
      boundary_tokens(second.boundaries);
  std::map<boundary_token, const SectionBoundarySample *> second_by_token{};
  for (std::size_t index = 0; index < second.boundaries.size(); ++index) {
    second_by_token.emplace(second_tokens[index], &second.boundaries[index]);
  }

  ConnectionGeometry geometry{};
  geometry.node_id = node_id;
  geometry.approaches = {first.approach, second.approach};
  const std::vector<curve_frame> frames = resolved_curve_frames(
      first.position, second.position, first.tangent, second.tangent,
      corner_control_m, kConnectionCurveSamples);
  if (frames.size() !=
      static_cast<std::size_t>(kConnectionCurveSamples + 1)) {
    return Result<ConnectionGeometry>::Fail(
        CommitFailureCategory::kInternalError,
        "road connection center curve has a degenerate tangent");
  }
  for (std::size_t index = 0; index < first.boundaries.size(); ++index) {
    const auto target = second_by_token.find(first_tokens[index]);
    if (target == second_by_token.end()) {
      return Result<ConnectionGeometry>::Fail(
          CommitFailureCategory::kNotImplemented,
          "road connection boundary ID/role mapping is incomplete");
    }
    geometry.boundary_curves.push_back(ResolvedBoundaryCurve{
        first.boundaries[index].boundary_id,
        target->second->boundary_id,
        first.boundaries[index].role,
        first.boundaries[index].marking.enabled &&
            target->second->marking.enabled,
        sweep_boundary(
            frames, boundary_point(first, first.boundaries[index]),
            boundary_point(second, *target->second)),
        sweep_boundary(
            frames,
            gate_point_at(first, first.boundaries[index].marking_lateral_m,
                          first.boundaries[index].height_m),
            gate_point_at(second, target->second->marking_lateral_m,
                          target->second->height_m)),
        first.approach,
        second.approach,
    });
  }
  if (first_section.surface_styles.size() + 1 !=
      geometry.boundary_curves.size()) {
    return Result<ConnectionGeometry>::Fail(
        CommitFailureCategory::kNotImplemented,
        "road connection surface mapping does not match boundary mapping");
  }
  for (std::size_t index = 0; index < first_section.surface_styles.size();
       ++index) {
    geometry.surface_strips.push_back(ResolvedSurfaceStrip{
        first_section.surface_styles[index],
        geometry.boundary_curves[index].source_boundary_id,
        geometry.boundary_curves[index + 1].source_boundary_id,
        geometry.boundary_curves[index].points,
        geometry.boundary_curves[index + 1].points,
    });
  }
  return Result<ConnectionGeometry>::Ok(std::move(geometry));
}

// One side of an approach, as the chain of section points running from where
// the road begins outward to the layout's edge, with the face between each
// neighbouring pair. Reducing this to a handful of named points is what turned
// a gutter into a slope.
struct junction_side {
  std::vector<const SectionBoundarySample *> chain{};
  std::vector<RenderStyleRef> faces{};
};

struct junction_section {
  junction_side left{};
  junction_side right{};
};

Result<junction_section>
resolve_junction_section(const ConnectionGate &gate,
                         const SectionEvaluation &section) {
  const std::vector<SectionBoundarySample> &samples = gate.boundaries;
  if (samples.size() < 2 || section.surface_styles.size() + 1 != samples.size()) {
    return Result<junction_section>::Fail(
        CommitFailureCategory::kInternalError,
        "road junction section does not describe its own surfaces");
  }
  std::vector<std::size_t> carriageway_edges{};
  for (std::size_t index = 0; index < samples.size(); ++index) {
    if (samples[index].carriageway_side != 0.0)
      carriageway_edges.push_back(index);
  }
  if (carriageway_edges.size() < 2) {
    return Result<junction_section>::Fail(
        CommitFailureCategory::kNotImplemented,
        "road junction section requires a carriageway with two edges");
  }
  junction_section resolved{};
  // Outward from the left edge is toward the front of the list; outward from
  // the right edge is toward the back.
  for (std::size_t index = carriageway_edges.front() + 1; index-- > 0;) {
    resolved.left.chain.push_back(&samples[index]);
    if (index > 0) resolved.left.faces.push_back(section.surface_styles[index - 1]);
  }
  for (std::size_t index = carriageway_edges.back(); index < samples.size(); ++index) {
    resolved.right.chain.push_back(&samples[index]);
    if (index + 1 < samples.size())
      resolved.right.faces.push_back(section.surface_styles[index]);
  }
  return Result<junction_section>::Ok(std::move(resolved));
}

struct side {
  ApproachKey approach{};
  Vec3d tangent{};
  std::vector<Vec3d> chain{};
  std::vector<RenderStyleRef> faces{};
  Vec3d painted_edge{};
  std::uint64_t carriageway_boundary_id = 0;
  std::uint64_t outer_boundary_id = 0;
  BoundaryRole carriageway_boundary_role = BoundaryRole::kOuterEdge;
  bool carriageway_is_painted = false;
};

double lateral_projection(const ConnectionGate &gate, const side &value) {
  return (value.chain.front().x - gate.position.x) * gate.lateral.x +
         (value.chain.front().y - gate.position.y) * gate.lateral.y;
}

} // namespace

Result<ConnectionGeometry>
generate_connection_geometry(RoadNodeId node_id, const ConnectionGate &first,
                             const ConnectionGate &second,
                             const SectionEvaluation &first_section,
                             const SectionEvaluation &second_section,
                             double corner_control_m) {
  return connection_geometry_from_gates(node_id, first, second, first_section,
                                        second_section, corner_control_m);
}

Result<JunctionGeometry>
generate_junction_geometry(RoadNodeId node_id,
                           const std::vector<ApproachKey> &ordered_approaches,
                           const std::vector<ConnectionGate> &gates,
                           const std::vector<const SectionEvaluation *> &sections,
                           double junction_corner_control_m) {
  if (gates.size() != ordered_approaches.size() ||
      sections.size() != gates.size()) {
    return Result<JunctionGeometry>::Fail(
        CommitFailureCategory::kInternalError, "road junction geometry input is incomplete");
  }
  JunctionGeometry geometry{};
  geometry.node_id = node_id;
  geometry.ordered_approaches = ordered_approaches;
  std::vector<side> sides{};
  for (std::size_t index = 0; index < gates.size(); ++index) {
    const ConnectionGate &gate = gates[index];
    if (sections[index] == nullptr) {
      return Result<JunctionGeometry>::Fail(CommitFailureCategory::kInternalError,
                                            "road junction section is missing");
    }
    Result<junction_section> supported =
        resolve_junction_section(gate, *sections[index]);
    if (!supported.ok) {
      return Result<JunctionGeometry>::Fail(supported.failure_category,
                                            supported.error);
    }
    const ApproachKey key = gate.approach;
    const auto side_from = [&gate, &key](const junction_side &profile) {
      side resolved{};
      resolved.approach = key;
      resolved.tangent = gate.tangent;
      resolved.faces = profile.faces;
      for (const SectionBoundarySample *sample : profile.chain)
        resolved.chain.push_back(boundary_point(gate, *sample));
      resolved.painted_edge =
          gate_point_at(gate, profile.chain.front()->marking_lateral_m,
                        profile.chain.front()->height_m);
      resolved.carriageway_boundary_id = profile.chain.front()->boundary_id;
      resolved.outer_boundary_id = profile.chain.back()->boundary_id;
      resolved.carriageway_boundary_role = profile.chain.front()->role;
      resolved.carriageway_is_painted = profile.chain.front()->marking.enabled;
      return resolved;
    };
    std::array<side, 2> gate_sides{
        side_from(supported.value.left),
        side_from(supported.value.right),
    };
    std::sort(gate_sides.begin(), gate_sides.end(),
              [&gate](const side &a, const side &b) {
                return lateral_projection(gate, a) <
                       lateral_projection(gate, b);
              });
    sides.push_back(gate_sides[0]);
    sides.push_back(gate_sides[1]);
  }
  if (gates.size() < 3 || sides.size() < 6) {
    return Result<JunctionGeometry>::Fail(
        CommitFailureCategory::kInternalError, "road junction resolved geometry is incomplete");
  }

  std::vector<Vec3d> asphalt_perimeter{};
  for (std::size_t index = 0; index < sides.size(); ++index) {
    const side &a = sides[index];
    const side &b = sides[(index + 1) % sides.size()];
    asphalt_perimeter.push_back(a.chain.front());
    if (a.approach == b.approach)
      continue;
    // One side may be built up where the other simply ends. The shorter one
    // stays at its outer edge so the pair still meets, which is what the road
    // does on the ground.
    const std::size_t depth = std::max(a.chain.size(), b.chain.size());
    const auto reach = [depth](const side &value, std::size_t point) {
      return value.chain[std::min(point, value.chain.size() - 1)];
    };
    const std::vector<RenderStyleRef> &faces =
        a.faces.size() >= b.faces.size() ? a.faces : b.faces;
    const auto curve_points = [&](Vec3d start, Vec3d end) {
      const std::vector<curve_frame> frames = resolved_curve_frames(
          start, end, a.tangent, b.tangent, junction_corner_control_m,
          kJunctionCurveSamples);
      std::vector<Vec3d> points{};
      points.reserve(frames.size());
      for (const curve_frame &frame : frames)
        points.push_back(frame.position);
      return points;
    };
    // One curve per section point, so whatever the edge is made of survives the
    // turn instead of being averaged into a ramp.
    std::vector<std::vector<Vec3d>> swept{};
    for (std::size_t point = 0; point < depth; ++point) {
      swept.push_back(curve_points(reach(a, point), reach(b, point)));
      if (swept.back().empty()) {
        return Result<JunctionGeometry>::Fail(
            CommitFailureCategory::kNotImplemented,
            "road junction boundary curve is degenerate");
      }
    }
    asphalt_perimeter.insert(asphalt_perimeter.end(), swept.front().begin() + 1,
                             swept.front().end() - 1);
    const std::vector<Vec3d> painted =
        curve_points(a.painted_edge, b.painted_edge);
    geometry.perimeter_curves.push_back(
        ResolvedBoundaryCurve{a.carriageway_boundary_id,
                              b.carriageway_boundary_id,
                              a.carriageway_boundary_role ==
                                      b.carriageway_boundary_role
                                  ? a.carriageway_boundary_role
                                  : BoundaryRole::kOuterEdge,
                              a.carriageway_is_painted && b.carriageway_is_painted,
                              swept.front(),
                              painted,
                              a.approach,
                              b.approach});
    for (std::size_t face = 0; face + 1 < swept.size() && face < faces.size();
         ++face) {
      geometry.surface_strips.push_back(ResolvedSurfaceStrip{
          faces[face], a.carriageway_boundary_id, b.outer_boundary_id,
          swept[face], swept[face + 1]});
    }
  }
  geometry.surface_regions.push_back(ResolvedSurfaceRegion{
      RenderStyleFromSurface(builtin_surface_styles::kAsphalt),
      std::move(asphalt_perimeter)});
  return Result<JunctionGeometry>::Ok(std::move(geometry));
}

} // namespace city::road::internal
