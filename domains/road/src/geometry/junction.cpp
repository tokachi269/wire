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
  const Vec3d c1 = add3(a, scale3(tangent_a, -control_m));
  const Vec3d c2 = add3(b, scale3(tangent_b, -control_m));
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

std::vector<Vec3d> resolved_curve(Vec3d a, Vec3d b, Vec3d tangent_a,
                                  Vec3d tangent_b, double control_m,
                                  int samples) {
  const std::vector<curve_frame> frames = resolved_curve_frames(
      a, b, tangent_a, tangent_b, control_m, samples);
  std::vector<Vec3d> points{};
  points.reserve(frames.size());
  for (const curve_frame &frame : frames)
    points.push_back(frame.position);
  return points;
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
        ErrorKind::kUnsupported,
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
        ErrorKind::kInternal,
        "road connection center curve has a degenerate tangent");
  }
  for (std::size_t index = 0; index < first.boundaries.size(); ++index) {
    const auto target = second_by_token.find(first_tokens[index]);
    if (target == second_by_token.end()) {
      return Result<ConnectionGeometry>::Fail(
          ErrorKind::kUnsupported,
          "road connection boundary ID/role mapping is incomplete");
    }
    geometry.boundary_curves.push_back(ResolvedBoundaryCurve{
        first.boundaries[index].boundary_id,
        target->second->boundary_id,
        first.boundaries[index].role,
        sweep_boundary(
            frames, boundary_point(first, first.boundaries[index]),
            boundary_point(second, *target->second)),
    });
  }
  if (first_section.surface_styles.size() + 1 !=
      geometry.boundary_curves.size()) {
    return Result<ConnectionGeometry>::Fail(
        ErrorKind::kUnsupported,
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

struct curb {
  const SectionBoundarySample *outer = nullptr;
  const SectionBoundarySample *sidewalk = nullptr;
  const SectionBoundarySample *carriageway = nullptr;
};

struct junction_section {
  curb left{};
  curb right{};
  double carriageway_left_m = 0.0;
  double carriageway_right_m = 0.0;
};

Result<junction_section>
validate_supported_junction_section(const ConnectionGate &gate,
                                    const SectionEvaluation &section) {
  if (section.surface_styles.size() < 5) {
    return Result<junction_section>::Fail(
        ErrorKind::kUnsupported,
        "road junction supports only sidewalk-curb-carriageway-curb-sidewalk "
        "sections");
  }
  std::map<std::uint64_t, std::vector<const SectionBoundarySample *>>
      curb_groups{};
  std::vector<const SectionBoundarySample *> outer_edges{};
  for (const SectionBoundarySample &boundary : gate.boundaries) {
    if (boundary.role == BoundaryRole::kCurb) {
      curb_groups[boundary.boundary_id].push_back(&boundary);
    } else if (boundary.role == BoundaryRole::kOuterEdge) {
      outer_edges.push_back(&boundary);
    }
  }
  if (curb_groups.size() != 2 || outer_edges.size() != 2) {
    return Result<junction_section>::Fail(
        ErrorKind::kUnsupported, "road junction section requires two explicit "
                                 "curb boundary IDs and two outer edges");
  }
  std::vector<std::vector<const SectionBoundarySample *>> curbs{};
  for (auto &[id, samples] : curb_groups) {
    (void)id;
    if (samples.size() != 2) {
      return Result<junction_section>::Fail(
          ErrorKind::kUnsupported,
          "road junction curb boundary mapping requires two profile samples");
    }
    std::sort(samples.begin(), samples.end(), [](const auto *a, const auto *b) {
      return a->lateral_m < b->lateral_m;
    });
    curbs.push_back(samples);
  }
  std::sort(curbs.begin(), curbs.end(), [](const auto &a, const auto &b) {
    return a.front()->lateral_m < b.front()->lateral_m;
  });
  std::sort(
      outer_edges.begin(), outer_edges.end(),
      [](const auto *a, const auto *b) { return a->lateral_m < b->lateral_m; });
  junction_section resolved{};
  resolved.left =
      curb{outer_edges.front(), curbs.front().front(), curbs.front().back()};
  resolved.right =
      curb{outer_edges.back(), curbs.back().back(), curbs.back().front()};
  resolved.carriageway_left_m = resolved.left.carriageway->lateral_m;
  resolved.carriageway_right_m = resolved.right.carriageway->lateral_m;
  if (resolved.carriageway_right_m <= resolved.carriageway_left_m) {
    return Result<junction_section>::Fail(
        ErrorKind::kUnsupported,
        "road junction carriageway boundary mapping is inverted");
  }
  return Result<junction_section>::Ok(resolved);
}

struct side {
  ApproachKey approach{};
  Vec3d tangent{};
  Vec3d outer{};
  Vec3d sidewalk{};
  Vec3d carriageway{};
  std::uint64_t outer_boundary_id = 0;
  std::uint64_t curb_boundary_id = 0;
};

double lateral_projection(const ConnectionGate &gate, const side &value) {
  return (value.carriageway.x - gate.position.x) * gate.lateral.x +
         (value.carriageway.y - gate.position.y) * gate.lateral.y;
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
        ErrorKind::kInternal, "road junction geometry input is incomplete");
  }
  JunctionGeometry geometry{};
  geometry.node_id = node_id;
  geometry.ordered_approaches = ordered_approaches;
  std::vector<side> sides{};
  for (std::size_t index = 0; index < gates.size(); ++index) {
    const ConnectionGate &gate = gates[index];
    if (sections[index] == nullptr) {
      return Result<JunctionGeometry>::Fail(ErrorKind::kInternal,
                                            "road junction section is missing");
    }
    Result<junction_section> supported =
        validate_supported_junction_section(gate, *sections[index]);
    if (!supported.ok) {
      return Result<JunctionGeometry>::Fail(supported.error_kind,
                                            supported.error);
    }
    const ApproachKey key = gate.approach;
    const auto side_from = [&gate, &key](const curb &profile) {
      return side{
          key,
          gate.tangent,
          boundary_point(gate, *profile.outer),
          boundary_point(gate, *profile.sidewalk),
          boundary_point(gate, *profile.carriageway),
          profile.outer->boundary_id,
          profile.carriageway->boundary_id,
      };
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
        ErrorKind::kInternal, "road junction resolved geometry is incomplete");
  }

  std::vector<Vec3d> asphalt_perimeter{};
  for (std::size_t index = 0; index < sides.size(); ++index) {
    const side &a = sides[index];
    const side &b = sides[(index + 1) % sides.size()];
    asphalt_perimeter.push_back(a.carriageway);
    if (a.approach == b.approach)
      continue;
    const std::vector<Vec3d> carriageway = resolved_curve(
        a.carriageway, b.carriageway, a.tangent, b.tangent,
        junction_corner_control_m, kJunctionCurveSamples);
    const std::vector<Vec3d> sidewalk =
        resolved_curve(a.sidewalk, b.sidewalk, a.tangent, b.tangent,
                       junction_corner_control_m, kJunctionCurveSamples);
    const std::vector<Vec3d> outer =
        resolved_curve(a.outer, b.outer, a.tangent, b.tangent,
                       junction_corner_control_m, kJunctionCurveSamples);
    asphalt_perimeter.insert(asphalt_perimeter.end(), carriageway.begin() + 1,
                             carriageway.end() - 1);
    geometry.perimeter_curves.push_back(
        ResolvedBoundaryCurve{a.curb_boundary_id, b.curb_boundary_id,
                              BoundaryRole::kCurb, carriageway});
    geometry.surface_strips.push_back(ResolvedSurfaceStrip{
        RenderStyleFromSurface(builtin_surface_styles::kCurb),
        a.curb_boundary_id, b.curb_boundary_id, carriageway, sidewalk});
    geometry.surface_strips.push_back(ResolvedSurfaceStrip{
        RenderStyleFromSurface(builtin_surface_styles::kSidewalk),
        a.outer_boundary_id, b.outer_boundary_id, sidewalk, outer});
  }
  geometry.surface_regions.push_back(ResolvedSurfaceRegion{
      RenderStyleFromSurface(builtin_surface_styles::kAsphalt),
      std::move(asphalt_perimeter)});
  return Result<JunctionGeometry>::Ok(std::move(geometry));
}

} // namespace city::road::internal
