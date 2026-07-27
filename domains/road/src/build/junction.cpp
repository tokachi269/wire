#include "pipeline.hpp"

#include "geometry.hpp"
#include "read.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <map>

namespace city::road::build {
namespace {

constexpr int kConnectionCurveSamples = 8;
constexpr int kJunctionCurveSamples = 6;

const ConnectionGate *find_gate(const DerivedRoad &out,
                                const ApproachKey &key) {
  const auto found = std::find_if(
      out.gates.begin(), out.gates.end(),
      [&key](const ConnectionGate &gate) { return gate.approach == key; });
  return found == out.gates.end() ? nullptr : &*found;
}

Vec3d add3(Vec3d a, Vec3d b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }

Vec3d scale3(Vec3d value, double factor) {
  return {value.x * factor, value.y * factor, value.z * factor};
}

Vec3d boundary_point(const ConnectionGate &gate,
                     const SectionBoundarySample &boundary) {
  return {
      gate.position.x + gate.lateral.x * boundary.lateral_m,
      gate.position.y + gate.lateral.y * boundary.lateral_m,
      gate.position.z + boundary.height_m,
  };
}

Vec3d cubic_point(Vec3d p0, Vec3d p1, Vec3d p2, Vec3d p3, double t) {
  const double u = 1.0 - t;
  return add3(add3(scale3(p0, u * u * u), scale3(p1, 3.0 * u * u * t)),
              add3(scale3(p2, 3.0 * u * t * t), scale3(p3, t * t * t)));
}

std::vector<Vec3d> resolved_curve(Vec3d a, Vec3d b, Vec3d tangent_a,
                                  Vec3d tangent_b, double control_m,
                                  int samples) {
  const Vec3d c1 = add3(a, scale3(tangent_a, -control_m));
  const Vec3d c2 = add3(b, scale3(tangent_b, -control_m));
  std::vector<Vec3d> points{};
  points.reserve(static_cast<std::size_t>(samples) + 1);
  for (int i = 0; i <= samples; ++i) {
    points.push_back(
        cubic_point(a, c1, c2, b, static_cast<double>(i) / samples));
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

Result<ConnectionGeometry> build_connection_geometry(
    const NodeConnectionDecision &decision, const ConnectionGate &first,
    const ConnectionGate &second, const DerivedRoad &out) {
  const SectionEvaluation *first_section = find_section(
      out, first.approach.segment_id,
      find_approach_connection(decision, first.approach)->gate_station_m);
  const SectionEvaluation *second_section = find_section(
      out, second.approach.segment_id,
      find_approach_connection(decision, second.approach)->gate_station_m);
  if (first_section == nullptr || second_section == nullptr) {
    return Result<ConnectionGeometry>::Fail(
        ErrorKind::kInternal, "road connection section read model is missing");
  }
  if (first_section->surface_styles != second_section->surface_styles) {
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
  geometry.node_id = decision.node_id;
  geometry.approaches = {first.approach, second.approach};
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
        resolved_curve(boundary_point(first, first.boundaries[index]),
                       boundary_point(second, *target->second), first.tangent,
                       second.tangent, decision.corner_control_m,
                       kConnectionCurveSamples),
    });
  }
  if (first_section->surface_styles.size() + 1 !=
      geometry.boundary_curves.size()) {
    return Result<ConnectionGeometry>::Fail(
        ErrorKind::kUnsupported,
        "road connection surface mapping does not match boundary mapping");
  }
  for (std::size_t index = 0; index < first_section->surface_styles.size();
       ++index) {
    geometry.surface_strips.push_back(ResolvedSurfaceStrip{
        first_section->surface_styles[index],
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

} // namespace

Result<bool> make_junctions(pipeline &pipe) {
  pipe.out.connection_areas.clear();
  pipe.out.junction_areas.clear();
  pipe.out.connections.clear();
  pipe.out.junctions.clear();

  for (const NodeConnectionDecision &decision : pipe.out.decisions) {
    if (decision.kind == NodeConnectionKind::kPassThrough)
      continue;
    if (decision.kind == NodeConnectionKind::kCorner) {
      if (decision.ordered_approaches.size() != 2) {
        return Result<bool>::Fail(ErrorKind::kInternal,
                                  "road corner approach order is invalid");
      }
      const ConnectionGate *first =
          find_gate(pipe.out, decision.ordered_approaches[0]);
      const ConnectionGate *second =
          find_gate(pipe.out, decision.ordered_approaches[1]);
      if (first == nullptr || second == nullptr) {
        return Result<bool>::Fail(ErrorKind::kInternal,
                                  "road corner gate is missing");
      }
      ConnectionArea area{};
      area.node_id = decision.node_id;
      area.gates = {*first, *second};
      Result<ConnectionGeometry> geometry =
          build_connection_geometry(decision, *first, *second, pipe.out);
      if (!geometry.ok) {
        return Result<bool>::Fail(geometry.error_kind, geometry.error);
      }
      pipe.out.connection_areas.push_back(std::move(area));
      pipe.out.connections.push_back(std::move(geometry.value));
      continue;
    }
    if (decision.kind != NodeConnectionKind::kJunction) {
      return Result<bool>::Fail(ErrorKind::kUnsupported,
                                "road connection decision is unsupported");
    }

    JunctionArea area{};
    area.policy_override_id = decision.applied_policy_override_id;
    area.node_id = decision.node_id;
    JunctionGeometry geometry{};
    geometry.node_id = decision.node_id;
    geometry.ordered_approaches = decision.ordered_approaches;
    std::vector<side> sides{};
    std::vector<std::pair<const ConnectionGate *, junction_section>>
        resolved_sections{};

    for (const ApproachKey &key : decision.ordered_approaches) {
      const ConnectionGate *gate = find_gate(pipe.out, key);
      const ResolvedNodeLayout *layout = find_layout(pipe.out, key.node_id);
      const ResolvedApproachLayout *approach =
          layout == nullptr ? nullptr : find_approach_layout(*layout, key);
      if (gate == nullptr || approach == nullptr) {
        return Result<bool>::Fail(ErrorKind::kInternal,
                                  "road junction gate is missing");
      }
      const SectionEvaluation *section =
          find_section(pipe.out, key.segment_id, approach->gate_station_m);
      if (section == nullptr) {
        return Result<bool>::Fail(ErrorKind::kInternal,
                                  "road junction section is missing");
      }
      Result<junction_section> supported =
          validate_supported_junction_section(*gate, *section);
      if (!supported.ok) {
        return Result<bool>::Fail(supported.error_kind, supported.error);
      }
      area.gates.push_back(*gate);
      resolved_sections.push_back({gate, supported.value});
      const auto side_from = [gate, &key](const curb &profile) {
        return side{
            key,
            gate->tangent,
            boundary_point(*gate, *profile.outer),
            boundary_point(*gate, *profile.sidewalk),
            boundary_point(*gate, *profile.carriageway),
            profile.outer->boundary_id,
            profile.carriageway->boundary_id,
        };
      };
      sides.push_back(side_from(supported.value.right));
      sides.push_back(side_from(supported.value.left));
    }
    if (area.gates.size() < 3 || sides.size() < 6) {
      return Result<bool>::Fail(
          ErrorKind::kInternal,
          "road junction resolved geometry is incomplete");
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
          decision.junction_corner_control_m, kJunctionCurveSamples);
      const std::vector<Vec3d> sidewalk = resolved_curve(
          a.sidewalk, b.sidewalk, a.tangent, b.tangent,
          decision.junction_corner_control_m, kJunctionCurveSamples);
      const std::vector<Vec3d> outer = resolved_curve(
          a.outer, b.outer, a.tangent, b.tangent,
          decision.junction_corner_control_m, kJunctionCurveSamples);
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

    pipe.out.junction_areas.push_back(std::move(area));
    pipe.out.junctions.push_back(std::move(geometry));
  }
  return Result<bool>::Ok(true);
}

} // namespace city::road::build
