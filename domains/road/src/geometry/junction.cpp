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

Vec3d cross3(Vec3d a, Vec3d b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
          a.x * b.y - a.y * b.x};
}

double dot3(Vec3d a, Vec3d b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

double signed_area_xy(const std::vector<Vec3d> &points) {
  double twice_area = 0.0;
  for (std::size_t index = 0; index < points.size(); ++index) {
    const Vec3d &a = points[index];
    const Vec3d &b = points[(index + 1) % points.size()];
    twice_area += a.x * b.y - b.x * a.y;
  }
  return twice_area * 0.5;
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

struct junction_side {
  std::vector<const SectionBoundarySample *> chain{};
  std::vector<RenderStyleRef> faces{};
  bool follows_section_order = false;
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
  for (std::size_t index = carriageway_edges.front() + 1; index-- > 0;) {
    resolved.left.chain.push_back(&samples[index]);
    if (index > 0) resolved.left.faces.push_back(section.surface_styles[index - 1]);
  }
  resolved.left.follows_section_order = false;
  for (std::size_t index = carriageway_edges.back(); index < samples.size(); ++index) {
    resolved.right.chain.push_back(&samples[index]);
    if (index + 1 < samples.size())
      resolved.right.faces.push_back(section.surface_styles[index]);
  }
  resolved.right.follows_section_order = true;
  return Result<junction_section>::Ok(std::move(resolved));
}

std::size_t aligned_side_index(std::size_t size, std::size_t depth,
                               std::size_t point) {
  const std::size_t missing = depth - size;
  return point < missing ? 0 : point - missing;
}

const SectionBoundarySample *aligned_side_sample(const junction_side &side,
                                                  std::size_t depth,
                                                  std::size_t point) {
  return side.chain[aligned_side_index(side.chain.size(), depth, point)];
}

Result<ConnectionGeometry> connection_geometry_from_gates(
    RoadNodeId node_id, const ConnectionGate &first,
    const ConnectionGate &second, const SectionEvaluation &first_section,
    const SectionEvaluation &second_section, double corner_control_m) {
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

  const Result<junction_section> first_profile =
      resolve_junction_section(first, first_section);
  const Result<junction_section> second_profile =
      resolve_junction_section(second, second_section);
  if (!first_profile.ok || !second_profile.ok) {
    const auto &failed = first_profile.ok ? second_profile : first_profile;
    return Result<ConnectionGeometry>::Fail(failed.failure_category,
                                             failed.error);
  }

  struct matched_rail {
    const SectionBoundarySample *source = nullptr;
    const SectionBoundarySample *target = nullptr;
    Vec3d source_point{};
    Vec3d target_point{};
    bool source_exists = true;
    bool target_exists = true;
  };
  struct matched_face {
    RenderStyleRef style{};
    Vec3d section_normal{};
    bool owned_by_source = true;
  };
  std::vector<matched_rail> rails{};
  std::vector<matched_face> faces{};
  const auto side_face_normal = [](const ConnectionGate &gate,
                                   const junction_side &side,
                                   std::size_t face) {
    const Vec3d profile_edge = subtract3(
        boundary_point(gate, *side.chain[face + 1]),
        boundary_point(gate, *side.chain[face]));
    const Vec3d section_edge =
        side.follows_section_order ? profile_edge : scale3(profile_edge, -1.0);
    const Vec3d segment_tangent =
        gate.approach.endpoint_role == EndpointRole::kStart
            ? gate.tangent
            : scale3(gate.tangent, -1.0);
    return cross3(segment_tangent, section_edge);
  };
  const auto append_side = [&rails, &faces, &first, &second,
                            &side_face_normal](
                               const junction_side &source,
                               const junction_side &target, bool reverse) {
    const std::size_t depth = std::max(source.chain.size(), target.chain.size());
    const bool owned_by_source = source.faces.size() >= target.faces.size();
    const junction_side &face_owner = owned_by_source ? source : target;
    const ConnectionGate &owner_gate = owned_by_source ? first : second;
    const auto rail_at = [&](std::size_t point) {
      const std::size_t source_missing = depth - source.chain.size();
      const std::size_t target_missing = depth - target.chain.size();
      const bool source_exists = point >= source_missing;
      const bool target_exists = point >= target_missing;
      const SectionBoundarySample *source_sample =
          aligned_side_sample(source, depth, point);
      const SectionBoundarySample *target_sample =
          aligned_side_sample(target, depth, point);
      return matched_rail{
          source_sample,
          target_sample,
          boundary_point(first, *source_sample),
          boundary_point(second, *target_sample),
          source_exists,
          target_exists,
      };
    };
    if (reverse) {
      for (std::size_t point = depth; point-- > 0;) {
        rails.push_back(rail_at(point));
      }
      for (std::size_t face = face_owner.faces.size(); face-- > 0;) {
        faces.push_back(matched_face{
            face_owner.faces[face],
            side_face_normal(owner_gate, face_owner, face), owned_by_source});
      }
    } else {
      for (std::size_t point = 1; point < depth; ++point) {
        const std::size_t face = point - 1;
        faces.push_back(matched_face{
            face_owner.faces[face],
            side_face_normal(owner_gate, face_owner, face), owned_by_source});
        rails.push_back(rail_at(point));
      }
    }
  };

  append_side(first_profile.value.left, second_profile.value.left, true);

  const auto source_left = static_cast<std::size_t>(
      first_profile.value.left.chain.front() - first.boundaries.data());
  const auto source_right = static_cast<std::size_t>(
      first_profile.value.right.chain.front() - first.boundaries.data());
  const auto target_left = static_cast<std::size_t>(
      second_profile.value.left.chain.front() - second.boundaries.data());
  const auto target_right = static_cast<std::size_t>(
      second_profile.value.right.chain.front() - second.boundaries.data());
  std::map<std::pair<std::uint64_t, BoundaryRole>,
           const SectionBoundarySample *> target_carriageway{};
  for (std::size_t index = target_left + 1; index < target_right; ++index) {
    const SectionBoundarySample &sample = second.boundaries[index];
    if (!target_carriageway
             .emplace(std::pair{sample.boundary_id, sample.role}, &sample)
             .second) {
      return Result<ConnectionGeometry>::Fail(
          CommitFailureCategory::kNotImplemented,
          "road connection carriageway boundary mapping is ambiguous");
    }
  }
  for (std::size_t index = source_left + 1; index < source_right; ++index) {
    const SectionBoundarySample &sample = first.boundaries[index];
    const auto target = target_carriageway.find(
        std::pair{sample.boundary_id, sample.role});
    if (target == target_carriageway.end()) {
      return Result<ConnectionGeometry>::Fail(
          CommitFailureCategory::kNotImplemented,
          "road connection carriageway boundary mapping is incomplete for " +
              std::to_string(sample.boundary_id) + "/" +
              std::to_string(static_cast<int>(sample.role)));
    }
    const Vec3d section_edge =
        subtract3(boundary_point(first, first.boundaries[index]),
                  boundary_point(first, first.boundaries[index - 1]));
    const Vec3d segment_tangent =
        first.approach.endpoint_role == EndpointRole::kStart
            ? first.tangent
            : scale3(first.tangent, -1.0);
    faces.push_back(matched_face{first_section.surface_styles[index - 1],
                                 cross3(segment_tangent, section_edge), true});
    rails.push_back(matched_rail{&sample, target->second,
                                 boundary_point(first, sample),
                                 boundary_point(second, *target->second)});
  }

  const Vec3d carriageway_edge =
      subtract3(boundary_point(first, first.boundaries[source_right]),
                boundary_point(first, first.boundaries[source_right - 1]));
  const Vec3d source_segment_tangent =
      first.approach.endpoint_role == EndpointRole::kStart
          ? first.tangent
          : scale3(first.tangent, -1.0);
  faces.push_back(matched_face{first_section.surface_styles[source_right - 1],
                               cross3(source_segment_tangent, carriageway_edge),
                               true});
  rails.push_back(matched_rail{
      first_profile.value.right.chain.front(),
      second_profile.value.right.chain.front(),
      boundary_point(first, *first_profile.value.right.chain.front()),
      boundary_point(second, *second_profile.value.right.chain.front())});

  append_side(first_profile.value.right, second_profile.value.right, false);
  if (rails.size() < 2 || faces.size() + 1 != rails.size()) {
    return Result<ConnectionGeometry>::Fail(
        CommitFailureCategory::kInternalError,
        "road connection semantic section mapping is incomplete");
  }

  for (const matched_rail &rail : rails) {
    if (rail.source == nullptr || rail.target == nullptr) {
      return Result<ConnectionGeometry>::Fail(
          CommitFailureCategory::kInternalError,
          "road connection semantic boundary is missing");
    }
    geometry.boundary_curves.push_back(ResolvedBoundaryCurve{
        rail.source->boundary_id,
        rail.target->boundary_id,
        rail.source->role == rail.target->role ? rail.source->role
                                                : BoundaryRole::kOuterEdge,
        rail.source_exists && rail.target_exists &&
            rail.source->role == rail.target->role &&
            rail.source->marking.enabled && rail.target->marking.enabled,
        sweep_boundary(frames, rail.source_point, rail.target_point),
        sweep_boundary(
            frames,
            gate_point_at(first, rail.source->marking_lateral_m,
                          rail.source->height_m),
            gate_point_at(second, rail.target->marking_lateral_m,
                          rail.target->height_m)),
        first.approach,
        second.approach,
    });
  }
  for (std::size_t index = 0; index < faces.size(); ++index) {
    const bool owned_by_source = faces[index].owned_by_source;
    const Vec3d profile_edge = owned_by_source
                                   ? subtract3(rails[index + 1].source_point,
                                               rails[index].source_point)
                                   : subtract3(rails[index + 1].target_point,
                                               rails[index].target_point);
    const Vec3d curve_normal =
        cross3(owned_by_source ? frames.front().tangent : frames.back().tangent,
               profile_edge);
    const SurfaceWinding winding =
        dot3(faces[index].section_normal, curve_normal) >= 0.0
            ? SurfaceWinding::kLeftToRight
            : SurfaceWinding::kRightToLeft;
    geometry.surface_strips.push_back(ResolvedSurfaceStrip{
        faces[index].style,
        geometry.boundary_curves[index].source_boundary_id,
        geometry.boundary_curves[index + 1].source_boundary_id,
        geometry.boundary_curves[index].points,
        geometry.boundary_curves[index + 1].points,
        winding,
    });
  }
  return Result<ConnectionGeometry>::Ok(std::move(geometry));
}

struct side {
  ApproachKey approach{};
  Vec3d tangent{};
  std::vector<Vec3d> chain{};
  std::vector<double> outward_m{};
  std::vector<double> height_m{};
  Vec3d outward{};
  std::vector<RenderStyleRef> faces{};
  std::vector<Vec3d> face_normals{};
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
      for (const SectionBoundarySample *sample : profile.chain) {
        resolved.chain.push_back(boundary_point(gate, *sample));
        resolved.outward_m.push_back(
            std::abs(sample->lateral_m - profile.chain.front()->lateral_m));
        resolved.height_m.push_back(sample->height_m);
      }
      const Vec3d segment_tangent =
          gate.approach.endpoint_role == EndpointRole::kStart
              ? gate.tangent
              : scale3(gate.tangent, -1.0);
      for (std::size_t face = 0; face + 1 < resolved.chain.size(); ++face) {
        const Vec3d profile_edge =
            subtract3(resolved.chain[face + 1], resolved.chain[face]);
        const Vec3d section_edge = profile.follows_section_order
                                       ? profile_edge
                                       : scale3(profile_edge, -1.0);
        resolved.face_normals.push_back(
            cross3(segment_tangent, section_edge));
      }
      const double spread = profile.chain.back()->lateral_m -
                            profile.chain.front()->lateral_m;
      const double mirror =
          gate.approach.endpoint_role == EndpointRole::kEnd ? -1.0 : 1.0;
      resolved.outward = scale3(gate.lateral, spread < 0.0 ? -mirror : mirror);
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
    const std::size_t depth = std::max(a.chain.size(), b.chain.size());
    const auto reach = [depth](const side &value, std::size_t point) {
      return value.chain[
          aligned_side_index(value.chain.size(), depth, point)];
    };
    const std::vector<RenderStyleRef> &faces =
        a.faces.size() >= b.faces.size() ? a.faces : b.faces;
    const side &face_owner = a.faces.size() >= b.faces.size() ? a : b;
    const auto curve_frames = [&](Vec3d start, Vec3d end) {
      return resolved_curve_frames(start, end, a.tangent, b.tangent,
                                   junction_corner_control_m,
                                   kJunctionCurveSamples);
    };
    std::vector<std::vector<Vec3d>> swept{};
    std::vector<std::vector<curve_frame>> swept_frames{};
    for (std::size_t point = 0; point < depth; ++point) {
      swept_frames.push_back(curve_frames(reach(a, point), reach(b, point)));
      if (swept_frames.back().empty()) {
        return Result<JunctionGeometry>::Fail(
            CommitFailureCategory::kNotImplemented,
            "road junction boundary curve is degenerate");
      }
      std::vector<Vec3d> points{};
      points.reserve(swept_frames.back().size());
      for (const curve_frame &frame : swept_frames.back())
        points.push_back(frame.position);
      swept.push_back(std::move(points));
    }
    asphalt_perimeter.insert(asphalt_perimeter.end(), swept.front().begin() + 1,
                             swept.front().end() - 1);
    const std::vector<curve_frame> painted_frames =
        curve_frames(a.painted_edge, b.painted_edge);
    std::vector<Vec3d> painted{};
    painted.reserve(painted_frames.size());
    for (const curve_frame &frame : painted_frames)
      painted.push_back(frame.position);
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
      const bool owned_by_start = &face_owner == &a;
      const Vec3d profile_edge =
          owned_by_start
              ? subtract3(swept[face + 1].front(), swept[face].front())
              : subtract3(swept[face + 1].back(), swept[face].back());
      const Vec3d curve_normal = cross3(
          owned_by_start ? swept_frames[face].front().tangent
                         : swept_frames[face].back().tangent,
          profile_edge);
      const SurfaceWinding winding =
          dot3(face_owner.face_normals[face], curve_normal) >= 0.0
              ? SurfaceWinding::kLeftToRight
              : SurfaceWinding::kRightToLeft;
      geometry.surface_strips.push_back(ResolvedSurfaceStrip{
          faces[face], a.carriageway_boundary_id, b.outer_boundary_id,
          swept[face], swept[face + 1], winding});
    }
  }
  const double perimeter_area = signed_area_xy(asphalt_perimeter);
  if (std::abs(perimeter_area) <= 1e-9) {
    return Result<JunctionGeometry>::Fail(
        CommitFailureCategory::kInternalError,
        "road junction perimeter has zero signed area");
  }
  if (perimeter_area < 0.0)
    std::reverse(asphalt_perimeter.begin(), asphalt_perimeter.end());
  geometry.surface_regions.push_back(ResolvedSurfaceRegion{
      RenderStyleFromSurface(builtin_surface_styles::kAsphalt),
      std::move(asphalt_perimeter)});
  return Result<JunctionGeometry>::Ok(std::move(geometry));
}

} // namespace city::road::internal
