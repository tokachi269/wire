#include "stages.hpp"

#include "stage_support.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <map>

namespace city::road::build {
namespace {

constexpr int kConnectionCurveSamples = 8;
constexpr int kJunctionCurveSamples = 6;

const ConnectionGate* find_gate(const DerivedRoad& derived,
                                const ApproachKey& key) {
  const auto found = std::find_if(
      derived.connection_gates.begin(), derived.connection_gates.end(),
      [&key](const ConnectionGate& gate) { return gate.approach == key; });
  return found == derived.connection_gates.end() ? nullptr : &*found;
}

Vec3d add3(Vec3d a, Vec3d b) {
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}

Vec3d scale3(Vec3d value, double scale) {
  return {value.x * scale, value.y * scale, value.z * scale};
}

Vec3d boundary_point(const ConnectionGate& gate,
                     const SectionBoundarySample& boundary) {
  return {
      gate.position.x + gate.lateral.x * boundary.lateral_m,
      gate.position.y + gate.lateral.y * boundary.lateral_m,
      gate.position.z + boundary.height_m,
  };
}

Vec3d cubic_point(Vec3d p0, Vec3d p1, Vec3d p2, Vec3d p3, double t) {
  const double u = 1.0 - t;
  return add3(
      add3(scale3(p0, u * u * u), scale3(p1, 3.0 * u * u * t)),
      add3(scale3(p2, 3.0 * u * t * t), scale3(p3, t * t * t)));
}

std::vector<Vec3d> resolved_curve(Vec3d a,
                                  Vec3d b,
                                  Vec3d tangent_a,
                                  Vec3d tangent_b,
                                  double control_m,
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

struct BoundaryToken {
  std::uint64_t id = 0;
  BoundaryRole role = BoundaryRole::kOuterEdge;
  std::size_t occurrence = 0;

  bool operator<(const BoundaryToken& other) const {
    return std::tie(id, role, occurrence) <
           std::tie(other.id, other.role, other.occurrence);
  }
};

std::vector<BoundaryToken> boundary_tokens(
    const std::vector<SectionBoundarySample>& boundaries) {
  std::map<std::pair<std::uint64_t, BoundaryRole>, std::size_t> occurrences{};
  std::vector<BoundaryToken> tokens{};
  tokens.reserve(boundaries.size());
  for (const SectionBoundarySample& boundary : boundaries) {
    const auto group = std::pair{boundary.boundary_id, boundary.role};
    tokens.push_back(
        BoundaryToken{boundary.boundary_id, boundary.role, occurrences[group]++});
  }
  return tokens;
}

Result<ConnectionGeometry> build_connection_geometry(
    const NodeConnectionDecision& decision,
    const ConnectionGate& first,
    const ConnectionGate& second,
    const DerivedRoad& derived) {
  const SectionEvaluation* first_section = FindSectionEvaluation(
      derived, first.approach.segment_id,
      FindApproachDecision(decision, first.approach)->gate_station_m);
  const SectionEvaluation* second_section = FindSectionEvaluation(
      derived, second.approach.segment_id,
      FindApproachDecision(decision, second.approach)->gate_station_m);
  if (first_section == nullptr || second_section == nullptr) {
    return Result<ConnectionGeometry>::Fail(
        ErrorKind::kInternal, "road connection section read model is missing");
  }
  if (first_section->surface_materials != second_section->surface_materials) {
    return Result<ConnectionGeometry>::Fail(
        ErrorKind::kUnsupported,
        "road connection requires identical explicit surface mappings");
  }
  const std::vector<BoundaryToken> first_tokens =
      boundary_tokens(first.boundaries);
  const std::vector<BoundaryToken> second_tokens =
      boundary_tokens(second.boundaries);
  std::map<BoundaryToken, const SectionBoundarySample*> second_by_token{};
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
  if (first_section->surface_materials.size() + 1 !=
      geometry.boundary_curves.size()) {
    return Result<ConnectionGeometry>::Fail(
        ErrorKind::kUnsupported,
        "road connection surface mapping does not match boundary mapping");
  }
  for (std::size_t index = 0;
       index < first_section->surface_materials.size(); ++index) {
    geometry.surface_strips.push_back(ResolvedSurfaceStrip{
        first_section->surface_materials[index],
        geometry.boundary_curves[index].source_boundary_id,
        geometry.boundary_curves[index + 1].source_boundary_id,
        geometry.boundary_curves[index].points,
        geometry.boundary_curves[index + 1].points,
    });
  }
  return Result<ConnectionGeometry>::Ok(std::move(geometry));
}

struct CurbProfile {
  const SectionBoundarySample* outer = nullptr;
  const SectionBoundarySample* sidewalk = nullptr;
  const SectionBoundarySample* carriageway = nullptr;
};

struct SupportedJunctionSection {
  CurbProfile left{};
  CurbProfile right{};
  double carriageway_left_m = 0.0;
  double carriageway_right_m = 0.0;
};

Result<SupportedJunctionSection> validate_supported_junction_section(
    const ConnectionGate& gate,
    const SectionEvaluation& section) {
  const bool supported_material_order =
      section.surface_materials.size() >= 5 &&
      section.surface_materials.front() == "sidewalk" &&
      section.surface_materials[1] == "curb" &&
      section.surface_materials[section.surface_materials.size() - 2] ==
          "curb" &&
      section.surface_materials.back() == "sidewalk" &&
      std::all_of(section.surface_materials.begin() + 2,
                  section.surface_materials.end() - 2,
                  [](const std::string& material) {
                    return material == "asphalt";
                  });
  if (!supported_material_order) {
    return Result<SupportedJunctionSection>::Fail(
        ErrorKind::kUnsupported,
        "road junction supports only sidewalk-curb-carriageway-curb-sidewalk sections");
  }
  std::map<std::uint64_t, std::vector<const SectionBoundarySample*>> curb_groups{};
  std::vector<const SectionBoundarySample*> outer_edges{};
  for (const SectionBoundarySample& boundary : gate.boundaries) {
    if (boundary.role == BoundaryRole::kCurb) {
      curb_groups[boundary.boundary_id].push_back(&boundary);
    } else if (boundary.role == BoundaryRole::kOuterEdge) {
      outer_edges.push_back(&boundary);
    }
  }
  if (curb_groups.size() != 2 || outer_edges.size() != 2) {
    return Result<SupportedJunctionSection>::Fail(
        ErrorKind::kUnsupported,
        "road junction section requires two explicit curb boundary IDs and two outer edges");
  }
  std::vector<std::vector<const SectionBoundarySample*>> curbs{};
  for (auto& [id, samples] : curb_groups) {
    (void)id;
    if (samples.size() != 2) {
      return Result<SupportedJunctionSection>::Fail(
          ErrorKind::kUnsupported,
          "road junction curb boundary mapping requires two profile samples");
    }
    std::sort(samples.begin(), samples.end(),
              [](const auto* a, const auto* b) {
                return a->lateral_m < b->lateral_m;
              });
    curbs.push_back(samples);
  }
  std::sort(curbs.begin(), curbs.end(), [](const auto& a, const auto& b) {
    return a.front()->lateral_m < b.front()->lateral_m;
  });
  std::sort(outer_edges.begin(), outer_edges.end(), [](const auto* a, const auto* b) {
    return a->lateral_m < b->lateral_m;
  });
  SupportedJunctionSection resolved{};
  resolved.left = CurbProfile{outer_edges.front(), curbs.front().front(),
                              curbs.front().back()};
  resolved.right = CurbProfile{outer_edges.back(), curbs.back().back(),
                               curbs.back().front()};
  resolved.carriageway_left_m = resolved.left.carriageway->lateral_m;
  resolved.carriageway_right_m = resolved.right.carriageway->lateral_m;
  if (resolved.carriageway_right_m <= resolved.carriageway_left_m) {
    return Result<SupportedJunctionSection>::Fail(
        ErrorKind::kUnsupported,
        "road junction carriageway boundary mapping is inverted");
  }
  return Result<SupportedJunctionSection>::Ok(resolved);
}

double surface_height(const ConnectionGate& gate, double lateral_m) {
  if (gate.boundaries.empty()) return 0.0;
  if (lateral_m <= gate.boundaries.front().lateral_m) {
    return gate.boundaries.front().height_m;
  }
  for (std::size_t index = 1; index < gate.boundaries.size(); ++index) {
    const SectionBoundarySample& a = gate.boundaries[index - 1];
    const SectionBoundarySample& b = gate.boundaries[index];
    if (lateral_m > b.lateral_m) continue;
    const double width = b.lateral_m - a.lateral_m;
    const double t =
        width <= kStationEpsilon ? 0.0 : (lateral_m - a.lateral_m) / width;
    return a.height_m + (b.height_m - a.height_m) * t;
  }
  return gate.boundaries.back().height_m;
}

ResolvedAutoMarking marking_quad(const ConnectionGate& gate,
                                 double longitudinal_center,
                                 double longitudinal_half,
                                 double lateral_center,
                                 double lateral_half) {
  ResolvedAutoMarking marking{};
  marking.approach = gate.approach;
  marking.material = "marking";
  std::array<Vec3d, 4> corners{};
  std::size_t index = 0;
  for (const auto [longitudinal, lateral] :
       std::array<std::pair<double, double>, 4>{
           std::pair{longitudinal_center - longitudinal_half,
                     lateral_center - lateral_half},
           std::pair{longitudinal_center + longitudinal_half,
                     lateral_center - lateral_half},
           std::pair{longitudinal_center - longitudinal_half,
                     lateral_center + lateral_half},
           std::pair{longitudinal_center + longitudinal_half,
                     lateral_center + lateral_half}}) {
    corners[index++] = Vec3d{
        gate.position.x + gate.tangent.x * longitudinal +
            gate.lateral.x * lateral,
        gate.position.y + gate.tangent.y * longitudinal +
            gate.lateral.y * lateral,
        surface_height(gate, lateral) + 0.025,
    };
  }
  marking.quads.push_back(corners);
  return marking;
}

struct JunctionSide {
  ApproachKey approach{};
  Vec3d tangent{};
  Vec3d outer{};
  Vec3d sidewalk{};
  Vec3d carriageway{};
  std::uint64_t outer_boundary_id = 0;
  std::uint64_t curb_boundary_id = 0;
};

} // namespace

Result<bool> BuildJunctionGeometries(BuildContext& context) {
  context.derived.connection_areas.clear();
  context.derived.junction_areas.clear();
  context.derived.connection_geometries.clear();
  context.derived.junction_geometries.clear();

  for (const NodeConnectionDecision& decision :
       context.derived.node_connection_decisions) {
    if (decision.kind == NodeConnectionKind::kPassThrough) continue;
    if (decision.kind == NodeConnectionKind::kCorner) {
      if (decision.ordered_approaches.size() != 2) {
        return Result<bool>::Fail(ErrorKind::kInternal,
                                  "road corner approach order is invalid");
      }
      const ConnectionGate* first =
          find_gate(context.derived, decision.ordered_approaches[0]);
      const ConnectionGate* second =
          find_gate(context.derived, decision.ordered_approaches[1]);
      if (first == nullptr || second == nullptr) {
        return Result<bool>::Fail(ErrorKind::kInternal,
                                  "road corner gate is missing");
      }
      ConnectionArea area{};
      area.node_id = decision.node_id;
      area.gates = {*first, *second};
      Result<ConnectionGeometry> geometry =
          build_connection_geometry(decision, *first, *second,
                                    context.derived);
      if (!geometry.ok) {
        return Result<bool>::Fail(geometry.error_kind, geometry.error);
      }
      context.derived.connection_areas.push_back(std::move(area));
      context.derived.connection_geometries.push_back(
          std::move(geometry.value));
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
    std::vector<JunctionSide> sides{};
    std::vector<std::pair<const ConnectionGate*, SupportedJunctionSection>>
        resolved_sections{};

    for (const ApproachKey& key : decision.ordered_approaches) {
      const ConnectionGate* gate = find_gate(context.derived, key);
      const ApproachConnectionDecision* approach =
          FindApproachDecision(decision, key);
      if (gate == nullptr || approach == nullptr) {
        return Result<bool>::Fail(ErrorKind::kInternal,
                                  "road junction gate is missing");
      }
      const SectionEvaluation* section = FindSectionEvaluation(
          context.derived, key.segment_id, approach->gate_station_m);
      if (section == nullptr) {
        return Result<bool>::Fail(ErrorKind::kInternal,
                                  "road junction section is missing");
      }
      Result<SupportedJunctionSection> supported =
          validate_supported_junction_section(*gate, *section);
      if (!supported.ok) {
        return Result<bool>::Fail(supported.error_kind, supported.error);
      }
      area.gates.push_back(*gate);
      resolved_sections.push_back({gate, supported.value});
      const auto side_from = [gate, &key](const CurbProfile& profile) {
        return JunctionSide{
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
      return Result<bool>::Fail(ErrorKind::kInternal,
                                "road junction resolved geometry is incomplete");
    }

    std::vector<Vec3d> asphalt_perimeter{};
    for (std::size_t index = 0; index < sides.size(); ++index) {
      const JunctionSide& a = sides[index];
      const JunctionSide& b = sides[(index + 1) % sides.size()];
      asphalt_perimeter.push_back(a.carriageway);
      if (a.approach == b.approach) continue;
      const std::vector<Vec3d> carriageway =
          resolved_curve(a.carriageway, b.carriageway, a.tangent, b.tangent,
                         decision.junction_corner_control_m,
                         kJunctionCurveSamples);
      const std::vector<Vec3d> sidewalk =
          resolved_curve(a.sidewalk, b.sidewalk, a.tangent, b.tangent,
                         decision.junction_corner_control_m,
                         kJunctionCurveSamples);
      const std::vector<Vec3d> outer =
          resolved_curve(a.outer, b.outer, a.tangent, b.tangent,
                         decision.junction_corner_control_m,
                         kJunctionCurveSamples);
      asphalt_perimeter.insert(asphalt_perimeter.end(),
                               carriageway.begin() + 1,
                               carriageway.end() - 1);
      geometry.perimeter_curves.push_back(ResolvedBoundaryCurve{
          a.curb_boundary_id, b.curb_boundary_id, BoundaryRole::kCurb,
          carriageway});
      geometry.surface_strips.push_back(ResolvedSurfaceStrip{
          "curb", a.curb_boundary_id, b.curb_boundary_id, carriageway,
          sidewalk});
      geometry.surface_strips.push_back(ResolvedSurfaceStrip{
          "sidewalk", a.outer_boundary_id, b.outer_boundary_id, sidewalk,
          outer});
    }
    geometry.surface_regions.push_back(
        ResolvedSurfaceRegion{"asphalt", std::move(asphalt_perimeter)});

    for (const auto& [gate, section] : resolved_sections) {
      const double left = section.carriageway_left_m;
      const double right = section.carriageway_right_m;
      geometry.auto_markings.push_back(
          marking_quad(*gate, 0.35, 0.08, (left + right) * 0.5,
                       (right - left) * 0.5));
      ResolvedAutoMarking zebra{};
      zebra.approach = gate->approach;
      zebra.material = "marking";
      for (double center = left + 0.35; center + 0.175 <= right;
           center += 0.7) {
        ResolvedAutoMarking stripe =
            marking_quad(*gate, 2.0, 1.4, center, 0.175);
        zebra.quads.push_back(stripe.quads.front());
      }
      geometry.auto_markings.push_back(std::move(zebra));
    }
    context.derived.junction_areas.push_back(std::move(area));
    context.derived.junction_geometries.push_back(std::move(geometry));
  }
  return Result<bool>::Ok(true);
}

} // namespace city::road::build
