#include "generation.hpp"

#include "../geometry/geometry.hpp"
#include "../geometry/junction.hpp"
#include "../lookup.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>
#include <tuple>

namespace city::road::generation {
namespace {

using internal::cross;
using internal::dot;
using internal::find_approach_override;
using internal::find_node;
using internal::find_policy_override;
using internal::find_segment;
using internal::find_template;
using internal::find_transition;
using internal::inward_tangent;
using internal::is_finite;
using internal::normalize;
using internal::scale;
using internal::distance_epsilon;
using internal::subtract;
using internal::tangent_at;
using internal::to3;

struct policy {
  double straight_tolerance_rad = 5.0 * std::numbers::pi / 180.0;
  double minimum_connection_angle_rad = 45.0 * std::numbers::pi / 180.0;
  double maximum_connection_angle_rad = 135.0 * std::numbers::pi / 180.0;
  double corner_radius_m = 4.0;
  double minimum_junction_setback_m = 4.0;
  double curve_control_factor = 0.45;
  double parallel_sine_tolerance = 1e-3;
  std::size_t maximum_approaches = 4;
};

constexpr policy rules{};

struct ordered_approach {
  ApproachKey key{};
  Vec2d tangent{};
  Vec2d chord{};
  std::int64_t angle_key = 0;
};

std::int64_t angle_key(Vec2d tangent) {
  constexpr double scale_factor = 1.0e12;
  return static_cast<std::int64_t>(std::llround(
      (std::atan2(tangent.y, tangent.x) + std::numbers::pi) * scale_factor));
}

bool approach_key_less(const ApproachKey &a, const ApproachKey &b) {
  return std::tie(a.node_id, a.segment_id, a.endpoint_role) <
         std::tie(b.node_id, b.segment_id, b.endpoint_role);
}

double endpoint_outer_half_width(const SavedRoadGraph &graph,
                                 const RoadSegment &segment,
                                 const ApproachKey &key) {
  CrossSectionTemplateId template_id = segment.section_template;
  if (segment.transition.has_value()) {
    const SectionTransition *transition =
        find_transition(graph, *segment.transition);
    if (transition == nullptr)
      return 0.0;
    template_id = key.endpoint_role == EndpointRole::kStart
                      ? transition->from_template
                      : transition->to_template;
  }
  const CrossSectionTemplate *section = find_template(graph, template_id);
  if (section == nullptr)
    return 0.0;
  double width = 0.0;
  for (const SectionStrip &strip : section->strips)
    width += strip.width_m;
  for (const BoundaryProfile &boundary : section->boundaries)
    width += boundary.width_m;
  return width * 0.5;
}

CrossSectionTemplateId endpoint_template_id(const SavedRoadGraph &graph,
                                            const RoadSegment &segment,
                                            const ApproachKey &key) {
  if (!segment.transition.has_value())
    return segment.section_template;
  const SectionTransition *transition =
      find_transition(graph, *segment.transition);
  if (transition == nullptr)
    return 0;
  return key.endpoint_role == EndpointRole::kStart ? transition->from_template
                                                   : transition->to_template;
}

const ResolvedApproach *approach_of(const ResolvedConnection &connection,
                                   const ApproachKey &key) {
  const auto found =
      std::find_if(connection.approaches.begin(), connection.approaches.end(),
                   [&key](const ResolvedApproach &approach) {
                     return approach.key == key;
                   });
  return found == connection.approaches.end() ? nullptr : &*found;
}

const DerivedSegment *segment_of(const std::vector<DerivedSegment> &segments,
                                 RoadSegmentId segment_id) {
  const auto found = std::find_if(segments.begin(), segments.end(),
                                  [segment_id](const DerivedSegment &segment) {
                                    return segment.id == segment_id;
                                  });
  return found == segments.end() ? nullptr : &*found;
}

ConnectionGate gate_at(const ApproachKey &key, Vec2d position, Vec2d tangent,
                       Vec2d lateral) {
  ConnectionGate gate{};
  gate.approach = key;
  gate.segment_id = key.segment_id;
  gate.node_id = key.node_id;
  gate.position = to3(position);
  gate.tangent = to3(tangent);
  gate.lateral = to3(lateral);
  gate.normal = Vec3d{0.0, 0.0, 1.0};
  return gate;
}

// The auto value, the user override and the frame that follows from them are
// decided together so no second table has to be kept in step.
Result<ResolvedApproach> resolve_approach(const SavedRoadGraph &graph,
                                          const DerivedSegment &segment,
                                          const ApproachKey &key,
                                          CrossSectionTemplateId endpoint_section,
                                          double auto_setback_m) {
  double setback = auto_setback_m;
  double lateral_shift = 0.0;
  if (const ApproachGeometryOverride *override =
          find_approach_override(graph, key)) {
    if (override->setback_m.has_value)
      setback = override->setback_m.value;
    if (override->lateral_shift_m.has_value)
      lateral_shift = override->lateral_shift_m.value;
  }
  if (!is_finite(setback) || setback < 0.0 ||
      setback > segment.length_m + distance_epsilon ||
      !is_finite(lateral_shift)) {
    return Result<ResolvedApproach>::Fail(
        ErrorKind::kUnsupported,
        "road approach override exceeds supported layout range");
  }
  const double distance = key.endpoint_role == EndpointRole::kStart
                             ? setback
                             : segment.length_m - setback;
  const Result<Vec2d> position = EvaluatePath(segment.alignment, distance);
  const Result<Vec2d> path_tangent = tangent_at(segment.alignment, distance);
  if (!position.ok || !path_tangent.ok) {
    return Result<ResolvedApproach>::Fail(
        ErrorKind::kInternal, "road approach frame could not be evaluated");
  }
  const Vec2d tangent = key.endpoint_role == EndpointRole::kStart
                            ? path_tangent.value
                            : scale(path_tangent.value, -1.0);
  const Vec2d lateral{-tangent.y, tangent.x};
  const Vec2d shifted =
      internal::add(position.value, scale(lateral, lateral_shift));

  ResolvedApproach approach{};
  approach.key = key;
  approach.endpoint_template_id = endpoint_section;
  approach.position = to3(shifted);
  approach.tangent = to3(tangent);
  approach.lateral = to3(lateral);
  approach.normal = Vec3d{0.0, 0.0, 1.0};
  approach.auto_setback_m = auto_setback_m;
  approach.resolved_setback_m = setback;
  approach.auto_lateral_shift_m = 0.0;
  approach.resolved_lateral_shift_m = lateral_shift;
  approach.gate_segment_distance_m = distance;
  approach.gate = gate_at(key, shifted, tangent, lateral);
  return Result<ResolvedApproach>::Ok(std::move(approach));
}

} // namespace

Result<std::vector<ResolvedConnection>>
resolve_connections(const SavedRoadGraph &graph,
                    const std::vector<NodeIncidence> &incidence,
                    const std::vector<DerivedSegment> &segments) {
  using Out = Result<std::vector<ResolvedConnection>>;
  std::vector<ResolvedConnection> connections{};
  connections.reserve(incidence.size());

  for (const NodeIncidence &topo : incidence) {
    const NodeConnectionPolicyOverride *policy_override =
        find_policy_override(graph, topo.node_id);
    // Endpoints and lone nodes carry no connection entity at all.
    if (topo.endpoints.size() <= 1 && policy_override == nullptr) {
      continue;
    }
    ResolvedConnection connection{};
    connection.node_id = topo.node_id;
    connection.corner_radius_m = rules.corner_radius_m;

    std::vector<ordered_approach> ordered{};
    ordered.reserve(topo.endpoints.size());
    for (const NodeEndpoint &incident : topo.endpoints) {
      const ApproachKey key{topo.node_id, incident.segment_id, incident.role};
      const RoadSegment *segment = find_segment(graph, key.segment_id);
      const DerivedSegment *derived = nullptr;
      for (const DerivedSegment &candidate : segments) {
        if (candidate.id == key.segment_id)
          derived = &candidate;
      }
      if (segment == nullptr || derived == nullptr) {
        return Out::Fail(ErrorKind::kInternal,
                         "road approach read model is missing");
      }
      const Result<Vec2d> tangent =
          inward_tangent(*segment, derived->alignment, key);
      if (!tangent.ok)
        return Out::Fail(tangent.error_kind, tangent.error);
      const RoadNode *node = find_node(graph, key.node_id);
      const RoadNodeId other_id = key.endpoint_role == EndpointRole::kStart
                                      ? segment->node_b
                                      : segment->node_a;
      const RoadNode *other = find_node(graph, other_id);
      if (node == nullptr || other == nullptr) {
        return Out::Fail(ErrorKind::kInternal,
                         "road approach chord endpoint is missing");
      }
      const Vec2d chord = normalize(subtract(other->position, node->position));
      ordered.push_back(ordered_approach{key, tangent.value, chord,
                                         angle_key(tangent.value)});
    }
    std::sort(ordered.begin(), ordered.end(),
              [](const ordered_approach &a, const ordered_approach &b) {
                if (a.angle_key != b.angle_key)
                  return a.angle_key < b.angle_key;
                return approach_key_less(a.key, b.key);
              });
    for (const ordered_approach &approach : ordered) {
      connection.ordered_approaches.push_back(approach.key);
    }
    if (ordered.size() == 2) {
      const double angle = std::acos(
          std::clamp(dot(ordered[0].chord, ordered[1].chord), -1.0, 1.0));
      const bool opposite = dot(ordered[0].tangent, ordered[1].tangent) <=
                            -std::cos(rules.straight_tolerance_rad);
      if (!opposite && (angle < rules.minimum_connection_angle_rad ||
                        angle > rules.maximum_connection_angle_rad)) {
        return Out::Fail(
            ErrorKind::kUnsupported,
            "road connected approach angle is outside supported range");
      }
    } else if (ordered.size() >= 3) {
      for (std::size_t index = 0; index < ordered.size(); ++index) {
        const ordered_approach &current = ordered[index];
        const ordered_approach &next =
            ordered[(index + 1) % ordered.size()];
        const double angle = std::acos(
            std::clamp(dot(current.chord, next.chord), -1.0, 1.0));
        if (angle < rules.minimum_connection_angle_rad) {
          return Out::Fail(
              ErrorKind::kUnsupported,
              "road adjacent junction approach angle is outside supported "
              "range");
        }
      }
    }

    if (policy_override != nullptr) {
      connection.applied_policy_override_id = policy_override->id;
      if (policy_override->policy == NodeConnectionPolicy::kForcePassThrough) {
        connection.kind = NodeConnectionKind::kPassThrough;
      } else if (policy_override->policy ==
                 NodeConnectionPolicy::kForceCorner) {
        connection.kind = NodeConnectionKind::kCorner;
      } else {
        connection.kind = NodeConnectionKind::kJunction;
      }
      connection.reason = "explicit policy override";
    } else if (ordered.size() <= 1) {
      connection.kind = NodeConnectionKind::kPassThrough;
      connection.reason = "endpoint";
    } else if (ordered.size() == 2) {
      const bool straight = dot(ordered[0].tangent, ordered[1].tangent) <=
                            -std::cos(rules.straight_tolerance_rad);
      connection.kind = straight ? NodeConnectionKind::kPassThrough
                                 : NodeConnectionKind::kCorner;
      connection.reason =
          straight ? "two aligned approaches" : "two turning approaches";
    } else if (ordered.size() <= rules.maximum_approaches) {
      connection.kind = NodeConnectionKind::kJunction;
      connection.reason = "three or four approaches";
    } else {
      connection.kind = NodeConnectionKind::kUnsupported;
      connection.reason = "more than four approaches";
      return Out::Fail(ErrorKind::kUnsupported,
                       "road node has more than four approaches");
    }

    double minimum_setback = std::numeric_limits<double>::infinity();
    for (const ordered_approach &approach : ordered) {
      const RoadSegment *segment = find_segment(graph, approach.key.segment_id);
      const DerivedSegment *derived = nullptr;
      for (const DerivedSegment &candidate : segments) {
        if (candidate.id == approach.key.segment_id)
          derived = &candidate;
      }
      if (segment == nullptr || derived == nullptr) {
        return Out::Fail(ErrorKind::kInternal, "road approach source is missing");
      }

      double setback = 0.0;
      if (connection.kind == NodeConnectionKind::kCorner) {
        const ordered_approach *other =
            ordered[0].key == approach.key ? &ordered[1] : &ordered[0];
        const double outward_angle = std::acos(
            std::clamp(dot(approach.tangent, other->tangent), -1.0, 1.0));
        const double turn_angle = std::numbers::pi - outward_angle;
        setback = rules.corner_radius_m * std::tan(turn_angle * 0.5);
      } else if (connection.kind == NodeConnectionKind::kJunction) {
        setback = rules.minimum_junction_setback_m;
        for (const ordered_approach &other : ordered) {
          if (other.key == approach.key)
            continue;
          const double sine = std::abs(cross(approach.tangent, other.tangent));
          if (sine <= rules.parallel_sine_tolerance)
            continue;
          const RoadSegment *other_segment =
              find_segment(graph, other.key.segment_id);
          if (other_segment == nullptr) {
            return Out::Fail(ErrorKind::kInternal,
                             "road setback source segment is missing");
          }
          setback = std::max(setback,
                             endpoint_outer_half_width(graph, *other_segment,
                                                       other.key) /
                                 sine);
        }
      }
      if (!is_finite(setback) || setback < 0.0 ||
          setback > derived->length_m + distance_epsilon) {
        return Out::Fail(ErrorKind::kUnsupported,
                         "road approach setback exceeds the segment length");
      }
      const CrossSectionTemplateId endpoint_section =
          endpoint_template_id(graph, *segment, approach.key);
      if (endpoint_section == 0) {
        return Out::Fail(ErrorKind::kValidation,
                         "road approach endpoint section template is missing");
      }
      Result<ResolvedApproach> resolved = resolve_approach(
          graph, *derived, approach.key, endpoint_section, setback);
      if (!resolved.ok)
        return Out::Fail(resolved.error_kind, resolved.error);
      connection.approaches.push_back(std::move(resolved.value));
      minimum_setback = std::min(minimum_setback, setback);
    }
    if (connection.approaches.size() > 1) {
      const CrossSectionTemplateId expected =
          connection.approaches.front().endpoint_template_id;
      if (std::any_of(connection.approaches.begin() + 1,
                      connection.approaches.end(),
                      [expected](const ResolvedApproach &approach) {
                        return approach.endpoint_template_id != expected;
                      })) {
        return Out::Fail(ErrorKind::kUnsupported,
                         "road connected approaches require identical "
                         "endpoint section template IDs");
      }
    }
    if (!std::isfinite(minimum_setback))
      minimum_setback = 0.0;
    const double curve_control_m = minimum_setback * rules.curve_control_factor;
    connection.corner_control_m = curve_control_m;
    connection.junction_corner_control_m = curve_control_m;
    connections.push_back(std::move(connection));
  }
  return Out::Ok(std::move(connections));
}

Result<bool> resolve_connection_geometry(std::vector<ResolvedConnection> &connections,
                                         const std::vector<DerivedSegment> &segments) {
  for (ResolvedConnection &connection : connections) {
    for (ResolvedApproach &approach : connection.approaches) {
      const DerivedSegment *segment = nullptr;
      for (const DerivedSegment &candidate : segments) {
        if (candidate.id == approach.key.segment_id)
          segment = &candidate;
      }
      const SectionEvaluation *section =
          segment == nullptr
              ? nullptr
              : FindSectionAt(*segment, approach.gate_segment_distance_m);
      if (section == nullptr) {
        return Result<bool>::Fail(
            ErrorKind::kInternal,
            "road connection gate has no unique section evaluation");
      }
      approach.gate.boundaries = section->boundaries;
    }

    if (connection.kind == NodeConnectionKind::kPassThrough)
      continue;
    if (connection.kind == NodeConnectionKind::kCorner) {
      if (connection.approaches.size() != 2) {
        return Result<bool>::Fail(ErrorKind::kInternal,
                                  "road corner approach order is invalid");
      }
      const ResolvedApproach *first = approach_of(
          connection, connection.ordered_approaches[0]);
      const ResolvedApproach *second = approach_of(
          connection, connection.ordered_approaches[1]);
      if (first == nullptr || second == nullptr) {
        return Result<bool>::Fail(ErrorKind::kInternal,
                                  "road corner approach is missing");
      }
      const DerivedSegment *first_segment = nullptr;
      const DerivedSegment *second_segment = nullptr;
      for (const DerivedSegment &candidate : segments) {
        if (candidate.id == first->key.segment_id)
          first_segment = &candidate;
        if (candidate.id == second->key.segment_id)
          second_segment = &candidate;
      }
      const SectionEvaluation *first_section =
          first_segment == nullptr
              ? nullptr
              : FindSectionAt(*first_segment, first->gate_segment_distance_m);
      const SectionEvaluation *second_section =
          second_segment == nullptr
              ? nullptr
              : FindSectionAt(*second_segment, second->gate_segment_distance_m);
      if (first_section == nullptr || second_section == nullptr) {
        return Result<bool>::Fail(
            ErrorKind::kInternal,
            "road connection section read model is missing");
      }
      Result<ConnectionGeometry> geometry = internal::generate_connection_geometry(
          connection.node_id, first->gate, second->gate, *first_section,
          *second_section, connection.corner_control_m);
      if (!geometry.ok)
        return Result<bool>::Fail(geometry.error_kind, geometry.error);
      connection.connection_geometry = std::move(geometry.value);
      continue;
    }
    if (connection.kind != NodeConnectionKind::kJunction) {
      return Result<bool>::Fail(ErrorKind::kUnsupported,
                                "road connection decision is unsupported");
    }

    std::vector<ConnectionGate> gates{};
    std::vector<const SectionEvaluation *> sections{};
    for (const ApproachKey &key : connection.ordered_approaches) {
      const ResolvedApproach *approach = approach_of(connection, key);
      const DerivedSegment *segment = nullptr;
      for (const DerivedSegment &candidate : segments) {
        if (candidate.id == key.segment_id)
          segment = &candidate;
      }
      if (approach == nullptr || segment == nullptr) {
        return Result<bool>::Fail(ErrorKind::kInternal,
                                  "road junction approach is missing");
      }
      gates.push_back(approach->gate);
      sections.push_back(FindSectionAt(*segment, approach->gate_segment_distance_m));
    }
    Result<JunctionGeometry> geometry = internal::generate_junction_geometry(
        connection.node_id, connection.ordered_approaches, gates, sections,
        connection.junction_corner_control_m);
    if (!geometry.ok)
      return Result<bool>::Fail(geometry.error_kind, geometry.error);
    connection.junction_geometry = std::move(geometry.value);
  }
  return Result<bool>::Ok(true);
}

} // namespace city::road::generation
