#include "generation.hpp"

#include "../geometry/geometry.hpp"
#include "../geometry/junction.hpp"
#include "../geometry/section.hpp"
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

// An off-centre alignment reaches further one way than the other, so a setback
// needs the larger of the two rather than half the total width.
double endpoint_outer_reach(const SavedRoadGraph &graph,
                            const RoadSegment &segment,
                            const ApproachKey &key) {
  RoadLayoutTemplateId template_id = segment.layout_template;
  if (segment.transition.has_value()) {
    const RoadLayoutTransition *transition =
        find_transition(graph, *segment.transition);
    if (transition == nullptr)
      return 0.0;
    template_id = key.endpoint_role == EndpointRole::kStart
                      ? transition->from_template
                      : transition->to_template;
  }
  const RoadLayoutTemplate *section = find_template(graph, template_id);
  if (section == nullptr)
    return 0.0;
  double width = 0.0;
  for (const RoadLayoutStrip &strip : section->strips)
    width += strip.width_m;
  const double offset = section->alignment_offset_from_left_m;
  return std::max(offset, width - offset);
}

RoadLayoutTemplateId endpoint_template_id(const SavedRoadGraph &graph,
                                            const RoadSegment &segment,
                                            const ApproachKey &key) {
  if (!segment.transition.has_value())
    return segment.layout_template;
  const RoadLayoutTransition *transition =
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

const ResolvedConnection *connection_of(
    const std::vector<ResolvedConnection> &connections, RoadNodeId node_id) {
  const auto found = std::find_if(
      connections.begin(), connections.end(),
      [node_id](const ResolvedConnection &connection) {
        return connection.node_id == node_id;
      });
  return found == connections.end() ? nullptr : &*found;
}

Result<double> lane_lateral(const LaneBand &lane,
                            const SectionEvaluation &section) {
  RoadLayoutTemplate resolved{};
  resolved.strips.push_back(RoadLayoutStrip{lane.surface_strip_id});
  const Result<internal::LaneSectionPosition> position =
      internal::lane_position(resolved, lane, section);
  return position.ok
             ? Result<double>::Ok(position.value.lateral_m)
             : Result<double>::Fail(position.failure_category, position.error);
}

Result<double> boundary_lateral(BoundaryId id,
                                const SectionEvaluation &section) {
  double minimum = std::numeric_limits<double>::infinity();
  double maximum = -std::numeric_limits<double>::infinity();
  for (const SectionBoundarySample &boundary : section.boundaries) {
    if (boundary.boundary_id != id)
      continue;
    minimum = std::min(minimum, boundary.lateral_m);
    maximum = std::max(maximum, boundary.lateral_m);
  }
  if (!std::isfinite(minimum) || !std::isfinite(maximum)) {
    return Result<double>::Fail(CommitFailureCategory::kInternalError,
                                "connected boundary sample is missing");
  }
  return Result<double>::Ok((minimum + maximum) * 0.5);
}

Vec2d endpoint_point(const ResolvedApproach &approach, double lateral_m) {
  const double section_sign =
      approach.key.endpoint_role == EndpointRole::kStart ? 1.0 : -1.0;
  return Vec2d{approach.gate.position.x +
                   approach.gate.lateral.x * lateral_m * section_sign,
               approach.gate.position.y +
                   approach.gate.lateral.y * lateral_m * section_sign};
}

Path connect_g1_endpoint_points(const ResolvedApproach &source,
                                double source_lateral_m,
                                const ResolvedApproach &target,
                                double target_lateral_m) {
  const Vec2d start = endpoint_point(source, source_lateral_m);
  const Vec2d end = endpoint_point(target, target_lateral_m);
  const Vec2d source_motion{-source.tangent.x, -source.tangent.y};
  const Vec2d target_motion{target.tangent.x, target.tangent.y};
  const double control = std::hypot(end.x - start.x, end.y - start.y) / 3.0;
  return MakePath({MakeBezier(
      start, internal::add(start, scale(source_motion, control)),
      subtract(end, scale(target_motion, control)), end)});
}

Path resolve_lane_transition_path(const ResolvedApproach &source,
                                  double source_lateral_m,
                                  const ResolvedApproach &target,
                                  double target_lateral_m) {
  return connect_g1_endpoint_points(source, source_lateral_m, target,
                                    target_lateral_m);
}

Result<Path> resolve_junction_movement_path(
    const ResolvedConnection &connection, const ResolvedApproach &source,
    double source_lateral_m, const ResolvedApproach &target,
    double target_lateral_m) {
  if (connection.kind != NodeConnectionKind::kJunction) {
    return Result<Path>::Fail(
        CommitFailureCategory::kNotImplemented,
        "lane junction movement requires a junction connection");
  }
  return Result<Path>::Ok(connect_g1_endpoint_points(
      source, source_lateral_m, target, target_lateral_m));
}

double minimum_radius(const Path &path) {
  if (path.spans.empty())
    return 0.0;
  const BezierSpan &span = path.spans.front();
  double minimum = std::numeric_limits<double>::infinity();
  for (int index = 0; index <= 24; ++index) {
    const double t = static_cast<double>(index) / 24.0;
    const double u = 1.0 - t;
    const Vec2d first{
        3.0 * u * u * (span.p1.x - span.p0.x) +
            6.0 * u * t * (span.p2.x - span.p1.x) +
            3.0 * t * t * (span.p3.x - span.p2.x),
        3.0 * u * u * (span.p1.y - span.p0.y) +
            6.0 * u * t * (span.p2.y - span.p1.y) +
            3.0 * t * t * (span.p3.y - span.p2.y)};
    const Vec2d second{
        6.0 * u * (span.p2.x - 2.0 * span.p1.x + span.p0.x) +
            6.0 * t * (span.p3.x - 2.0 * span.p2.x + span.p1.x),
        6.0 * u * (span.p2.y - 2.0 * span.p1.y + span.p0.y) +
            6.0 * t * (span.p3.y - 2.0 * span.p2.y + span.p1.y)};
    const double speed = std::hypot(first.x, first.y);
    const double bend = std::abs(cross(first, second));
    if (speed <= distance_epsilon || bend <= distance_epsilon)
      continue;
    minimum = std::min(minimum, speed * speed * speed / bend);
  }
  return std::isfinite(minimum) ? minimum
                                : std::numeric_limits<double>::infinity();
}

Vec2d evaluate_span(const BezierSpan &span, double t) {
  const double u = 1.0 - t;
  return Vec2d{
      u * u * u * span.p0.x + 3.0 * u * u * t * span.p1.x +
          3.0 * u * t * t * span.p2.x + t * t * t * span.p3.x,
      u * u * u * span.p0.y + 3.0 * u * u * t * span.p1.y +
          3.0 * u * t * t * span.p2.y + t * t * t * span.p3.y};
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
                                          RoadLayoutTemplateId endpoint_section,
                                          double auto_setback_m,
                                          double auto_lateral_shift_m) {
  double setback = auto_setback_m;
  double lateral_shift = auto_lateral_shift_m;
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
        CommitFailureCategory::kNotImplemented,
        "road approach override exceeds supported layout range");
  }
  const double distance = key.endpoint_role == EndpointRole::kStart
                             ? setback
                             : segment.length_m - setback;
  const Result<Vec2d> position = EvaluatePath(segment.alignment, distance);
  const Result<Vec2d> path_tangent = tangent_at(segment.alignment, distance);
  if (!position.ok || !path_tangent.ok) {
    return Result<ResolvedApproach>::Fail(
        CommitFailureCategory::kInternalError, "road approach frame could not be evaluated");
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
  approach.auto_lateral_shift_m = auto_lateral_shift_m;
  approach.resolved_lateral_shift_m = lateral_shift;
  approach.gate_segment_distance_m = distance;
  approach.gate = gate_at(key, shifted, tangent, lateral);
  return Result<ResolvedApproach>::Ok(std::move(approach));
}

const ordered_approach *ordered_approach_of(
    const std::vector<ordered_approach> &ordered, RoadSegmentId segment_id,
    EndpointRole endpoint_role) {
  const auto found = std::find_if(
      ordered.begin(), ordered.end(),
      [segment_id, endpoint_role](const ordered_approach &approach) {
        return approach.key.segment_id == segment_id &&
               approach.key.endpoint_role == endpoint_role;
      });
  return found == ordered.end() ? nullptr : &*found;
}

Result<double> auto_lateral_shift_for(
    const SavedRoadGraph &graph, const std::vector<ordered_approach> &ordered,
    const ordered_approach &target) {
  std::optional<double> resolved{};
  for (const LaneConnection &connection : graph.lane_connections) {
    if ((connection.kind != LaneConnectionKind::kContinuation &&
         connection.kind != LaneConnectionKind::kMerge) ||
        connection.target.segment_id != target.key.segment_id ||
        connection.target.endpoint_role != target.key.endpoint_role)
      continue;
    const internal::LaneEndpointLookup source_lookup =
        internal::find_lane_endpoint(graph, connection.source);
    const internal::LaneEndpointLookup target_lookup =
        internal::find_lane_endpoint(graph, connection.target);
    const ordered_approach *source = ordered_approach_of(
        ordered, connection.source.segment_id,
        connection.source.endpoint_role);
    if (source_lookup.lane == nullptr || source_lookup.section == nullptr ||
        target_lookup.lane == nullptr || target_lookup.section == nullptr ||
        source == nullptr) {
      return Result<double>::Fail(
          CommitFailureCategory::kInternalError,
          "lane continuation auto layout input is missing");
    }
    if (dot(source->tangent, target.tangent) >
        -std::cos(rules.straight_tolerance_rad)) {
      continue;
    }
    const Result<double> source_lateral =
        internal::lane_template_lateral(*source_lookup.section,
                                        *source_lookup.lane);
    const Result<double> target_lateral =
        internal::lane_template_lateral(*target_lookup.section,
                                        *target_lookup.lane);
    if (!source_lateral.ok || !target_lateral.ok) {
      return Result<double>::Fail(
          CommitFailureCategory::kInternalError,
          "lane continuation lateral position is missing");
    }
    const double source_sign =
        connection.source.endpoint_role == EndpointRole::kStart ? 1.0 : -1.0;
    const double target_sign =
        connection.target.endpoint_role == EndpointRole::kStart ? 1.0 : -1.0;
    const Vec2d source_axis{-source->tangent.y * source_sign,
                            source->tangent.x * source_sign};
    const Vec2d target_axis{-target.tangent.y * target_sign,
                            target.tangent.x * target_sign};
    const Vec2d target_gate_lateral{-target.tangent.y, target.tangent.x};
    const Vec2d delta = subtract(scale(source_axis, source_lateral.value),
                                 scale(target_axis, target_lateral.value));
    const double shift = dot(delta, target_gate_lateral);
    if (resolved.has_value() &&
        std::abs(*resolved - shift) > distance_epsilon) {
      return Result<double>::Fail(
          CommitFailureCategory::kNotImplemented,
          "lane continuations require conflicting target lateral shifts");
    }
    resolved = shift;
  }
  return Result<double>::Ok(resolved.value_or(0.0));
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
        return Out::Fail(CommitFailureCategory::kInternalError,
                         "road approach read model is missing");
      }
      const Result<Vec2d> tangent =
          inward_tangent(*segment, derived->alignment, key);
      if (!tangent.ok)
        return Out::Fail(tangent.failure_category, tangent.error);
      const RoadNode *node = find_node(graph, key.node_id);
      const RoadNodeId other_id = key.endpoint_role == EndpointRole::kStart
                                      ? segment->node_b
                                      : segment->node_a;
      const RoadNode *other = find_node(graph, other_id);
      if (node == nullptr || other == nullptr) {
        return Out::Fail(CommitFailureCategory::kInternalError,
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
      return Out::Fail(CommitFailureCategory::kNotImplemented,
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
        return Out::Fail(CommitFailureCategory::kInternalError, "road approach source is missing");
      }

      double setback = 0.0;
      if (connection.kind == NodeConnectionKind::kCorner) {
        const ordered_approach *other =
            ordered[0].key == approach.key ? &ordered[1] : &ordered[0];
        const double outward_angle = std::acos(
            std::clamp(dot(approach.tangent, other->tangent), -1.0, 1.0));
        const double turn_angle = std::numbers::pi - outward_angle;
        const RoadSegment *other_segment =
            find_segment(graph, other->key.segment_id);
        if (other_segment == nullptr) {
          return Out::Fail(CommitFailureCategory::kInternalError,
                           "road corner source segment is missing");
        }
        const double section_reach = std::max(
            endpoint_outer_reach(graph, *segment, approach.key),
            endpoint_outer_reach(graph, *other_segment, other->key));
        setback = (rules.corner_radius_m + section_reach) *
                  std::tan(turn_angle * 0.5);
      } else if (connection.kind == NodeConnectionKind::kJunction) {
        setback = rules.minimum_junction_setback_m;
        for (const ordered_approach &other : ordered) {
          if (other.key == approach.key)
            continue;
          const double sine = std::abs(cross(approach.tangent, other.tangent));
          if (sine <= rules.parallel_sine_tolerance)
            continue;
          const double angle = std::acos(std::clamp(
              dot(approach.tangent, other.tangent), -1.0, 1.0));
          const double half_angle_tangent = std::tan(angle * 0.5);
          if (half_angle_tangent <= rules.parallel_sine_tolerance)
            continue;
          const RoadSegment *other_segment =
              find_segment(graph, other.key.segment_id);
          if (other_segment == nullptr) {
            return Out::Fail(CommitFailureCategory::kInternalError,
                             "road setback source segment is missing");
          }
          const double section_clearance_m =
              endpoint_outer_reach(graph, *other_segment, other.key) / sine;
          const double corner_clearance_m =
              rules.corner_radius_m / half_angle_tangent;
          setback = std::max(setback,
                             section_clearance_m + corner_clearance_m);
        }
      }
      if (!is_finite(setback) || setback < 0.0 ||
          setback > derived->length_m + distance_epsilon) {
        return Out::Fail(CommitFailureCategory::kNotImplemented,
                         "road approach setback exceeds the segment length");
      }
      const RoadLayoutTemplateId endpoint_section =
          endpoint_template_id(graph, *segment, approach.key);
      if (endpoint_section == 0) {
        return Out::Fail(CommitFailureCategory::kInvalidInput,
                         "road approach endpoint section template is missing");
      }
      const Result<double> auto_lateral_shift =
          auto_lateral_shift_for(graph, ordered, approach);
      if (!auto_lateral_shift.ok) {
        return Out::Fail(auto_lateral_shift.failure_category,
                         auto_lateral_shift.error);
      }
      Result<ResolvedApproach> resolved = resolve_approach(
          graph, *derived, approach.key, endpoint_section, setback,
          auto_lateral_shift.value);
      if (!resolved.ok)
        return Out::Fail(resolved.failure_category, resolved.error);
      connection.approaches.push_back(std::move(resolved.value));
      minimum_setback = std::min(minimum_setback, setback);
    }
    if (connection.approaches.size() > 1) {
      const RoadLayoutTemplate *expected = find_template(
          graph, connection.approaches.front().endpoint_template_id);
      const bool mixed = expected == nullptr || std::any_of(
          connection.approaches.begin() + 1, connection.approaches.end(),
          [&graph, expected](const ResolvedApproach &approach) {
            const RoadLayoutTemplate *candidate =
                find_template(graph, approach.endpoint_template_id);
            return candidate == nullptr ||
                   !internal::equivalent_section_definition(*expected,
                                                            *candidate);
          });
      const RoadSegmentId first_segment_id =
          connection.approaches.front().key.segment_id;
      const RoadSegmentId second_segment_id =
          connection.approaches.size() == 2
              ? connection.approaches.back().key.segment_id
              : 0;
      const auto connects_approach_pair =
          [first_segment_id,
           second_segment_id](RoadSegmentId source, RoadSegmentId target) {
            return (source == first_segment_id &&
                    target == second_segment_id) ||
                   (source == second_segment_id &&
                    target == first_segment_id);
          };
      const bool has_pair_lane_topology =
          second_segment_id != 0 &&
          std::any_of(graph.lane_connections.begin(),
                      graph.lane_connections.end(),
                      [&connects_approach_pair](const LaneConnection &lane) {
                        return connects_approach_pair(lane.source.segment_id,
                                                      lane.target.segment_id);
                      });
      const bool has_pair_boundary_topology =
          second_segment_id != 0 &&
          std::any_of(graph.boundary_continuations.begin(),
                      graph.boundary_continuations.end(),
                      [&connects_approach_pair](
                          const BoundaryContinuation &boundary) {
                        return connects_approach_pair(
                            boundary.source.segment_id,
                            boundary.target.segment_id);
                      });
      // Junction approaches may intentionally terminate at their gates. An
      // explicit JunctionMovement opts individual lanes into a movement; it is
      // not evidence that every lane/boundary has been mapped. Degree-two
      // connections, however, imply continuity and must not hide a section
      // change at the node.
      if (mixed && connection.kind != NodeConnectionKind::kJunction &&
          !(has_pair_lane_topology && has_pair_boundary_topology)) {
        return Out::Fail(CommitFailureCategory::kNotImplemented,
                         "road degree-two connection lane layouts differ; add "
                         "a section transition before the node");
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
            CommitFailureCategory::kInternalError,
            "road connection gate has no unique section evaluation");
      }
      approach.gate.boundaries = section->boundaries;
    }

    if (connection.kind == NodeConnectionKind::kPassThrough)
      continue;
    if (connection.kind == NodeConnectionKind::kCorner) {
      if (connection.approaches.size() != 2) {
        return Result<bool>::Fail(CommitFailureCategory::kInternalError,
                                  "road corner approach order is invalid");
      }
      const ResolvedApproach *first = approach_of(
          connection, connection.ordered_approaches[0]);
      const ResolvedApproach *second = approach_of(
          connection, connection.ordered_approaches[1]);
      if (first == nullptr || second == nullptr) {
        return Result<bool>::Fail(CommitFailureCategory::kInternalError,
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
            CommitFailureCategory::kInternalError,
            "road connection section read model is missing");
      }
      Result<ConnectionGeometry> geometry = internal::generate_connection_geometry(
          connection.node_id, first->gate, second->gate, *first_section,
          *second_section, connection.corner_control_m);
      if (!geometry.ok)
        return Result<bool>::Fail(geometry.failure_category, geometry.error);
      connection.connection_geometry = std::move(geometry.value);
      continue;
    }
    if (connection.kind != NodeConnectionKind::kJunction) {
      return Result<bool>::Fail(CommitFailureCategory::kNotImplemented,
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
        return Result<bool>::Fail(CommitFailureCategory::kInternalError,
                                  "road junction approach is missing");
      }
      gates.push_back(approach->gate);
      sections.push_back(FindSectionAt(*segment, approach->gate_segment_distance_m));
    }
    Result<JunctionGeometry> geometry = internal::generate_junction_geometry(
        connection.node_id, connection.ordered_approaches, gates, sections,
        connection.junction_corner_control_m);
    if (!geometry.ok)
      return Result<bool>::Fail(geometry.failure_category, geometry.error);
    connection.junction_geometry = std::move(geometry.value);
  }
  return Result<bool>::Ok(true);
}

Result<bool> derive_topology_paths(
    const SavedRoadGraph &graph,
    const std::vector<ResolvedConnection> &connections,
    const std::vector<DerivedSegment> &segments,
    std::vector<DerivedLanePath> &lane_paths,
    std::vector<DerivedBoundaryPath> &boundary_paths,
    std::vector<DerivedSeparationArea> &separation_areas) {
  lane_paths.clear();
  boundary_paths.clear();
  separation_areas.clear();
  for (const LaneConnection &topology : graph.lane_connections) {
    const internal::LaneEndpointLookup source_lookup =
        internal::find_lane_endpoint(graph, topology.source);
    const internal::LaneEndpointLookup target_lookup =
        internal::find_lane_endpoint(graph, topology.target);
    const ResolvedConnection *connection =
        connection_of(connections, source_lookup.node_id);
    const ResolvedApproach *source =
        connection == nullptr ? nullptr
                              : approach_of(*connection,
                                            ApproachKey{source_lookup.node_id,
                                                        topology.source.segment_id,
                                                        topology.source.endpoint_role});
    const ResolvedApproach *target =
        connection == nullptr ? nullptr
                              : approach_of(*connection,
                                            ApproachKey{target_lookup.node_id,
                                                        topology.target.segment_id,
                                                        topology.target.endpoint_role});
    const DerivedSegment *source_segment =
        segment_of(segments, topology.source.segment_id);
    const DerivedSegment *target_segment =
        segment_of(segments, topology.target.segment_id);
    const SectionEvaluation *source_section =
        source_segment == nullptr || source == nullptr
            ? nullptr
            : FindSectionAt(*source_segment,
                            source->gate_segment_distance_m);
    const SectionEvaluation *target_section =
        target_segment == nullptr || target == nullptr
            ? nullptr
            : FindSectionAt(*target_segment,
                            target->gate_segment_distance_m);
    if (source_lookup.lane == nullptr || target_lookup.lane == nullptr ||
        source == nullptr || target == nullptr || source_section == nullptr ||
        target_section == nullptr) {
      return Result<bool>::Fail(
          CommitFailureCategory::kInternalError,
          "lane connection path input is missing from resolved road");
    }
    const Result<double> source_lateral =
        lane_lateral(*source_lookup.lane, *source_section);
    const Result<double> target_lateral =
        lane_lateral(*target_lookup.lane, *target_section);
    if (!source_lateral.ok || !target_lateral.ok) {
      return Result<bool>::Fail(
          CommitFailureCategory::kInternalError,
          "lane connection path cannot resolve a lane center");
    }
    Result<Path> resolved_path =
        topology.kind == LaneConnectionKind::kJunctionMovement
            ? resolve_junction_movement_path(
                  *connection, *source, source_lateral.value, *target,
                  target_lateral.value)
            : Result<Path>::Ok(resolve_lane_transition_path(
                  *source, source_lateral.value, *target,
                  target_lateral.value));
    if (!resolved_path.ok) {
      return Result<bool>::Fail(resolved_path.failure_category,
                                resolved_path.error);
    }
    Path path = std::move(resolved_path.value);
    if (std::hypot(path.spans.front().p3.x - path.spans.front().p0.x,
                   path.spans.front().p3.y - path.spans.front().p0.y) <=
        distance_epsilon) {
      lane_paths.push_back(DerivedLanePath{
          topology.id, Path{}, 0.0,
          std::numeric_limits<double>::infinity()});
      continue;
    }
    const Result<double> length = PathLength(path);
    if (!length.ok) {
      return Result<bool>::Fail(length.failure_category, length.error);
    }
    lane_paths.push_back(DerivedLanePath{topology.id, std::move(path),
                                         length.value, 0.0});
    lane_paths.back().minimum_radius_m =
        minimum_radius(lane_paths.back().centerline);
  }

  for (const BoundaryContinuation &topology :
       graph.boundary_continuations) {
    const internal::BoundaryEndpointLookup source_lookup =
        internal::find_boundary_endpoint(graph, topology.source);
    const internal::BoundaryEndpointLookup target_lookup =
        internal::find_boundary_endpoint(graph, topology.target);
    const ResolvedConnection *connection =
        connection_of(connections, source_lookup.node_id);
    const ResolvedApproach *source =
        connection == nullptr ? nullptr
                              : approach_of(*connection,
                                            ApproachKey{source_lookup.node_id,
                                                        topology.source.segment_id,
                                                        topology.source.endpoint_role});
    const ResolvedApproach *target =
        connection == nullptr ? nullptr
                              : approach_of(*connection,
                                            ApproachKey{target_lookup.node_id,
                                                        topology.target.segment_id,
                                                        topology.target.endpoint_role});
    const DerivedSegment *source_segment =
        segment_of(segments, topology.source.segment_id);
    const DerivedSegment *target_segment =
        segment_of(segments, topology.target.segment_id);
    const SectionEvaluation *source_section =
        source_segment == nullptr || source == nullptr
            ? nullptr
            : FindSectionAt(*source_segment,
                            source->gate_segment_distance_m);
    const SectionEvaluation *target_section =
        target_segment == nullptr || target == nullptr
            ? nullptr
            : FindSectionAt(*target_segment,
                            target->gate_segment_distance_m);
    if (source_lookup.boundary == nullptr ||
        target_lookup.boundary == nullptr || source == nullptr ||
        target == nullptr || source_section == nullptr ||
        target_section == nullptr) {
      return Result<bool>::Fail(
          CommitFailureCategory::kInternalError,
          "boundary continuation path input is missing from resolved road");
    }
    const Result<double> source_lateral =
        boundary_lateral(topology.source.boundary_id, *source_section);
    const Result<double> target_lateral =
        boundary_lateral(topology.target.boundary_id, *target_section);
    if (!source_lateral.ok || !target_lateral.ok) {
      return Result<bool>::Fail(
          CommitFailureCategory::kInternalError,
          "boundary continuation path cannot resolve a boundary");
    }
    Path path = resolve_lane_transition_path(
        *source, source_lateral.value, *target, target_lateral.value);
    if (std::hypot(path.spans.front().p3.x - path.spans.front().p0.x,
                   path.spans.front().p3.y - path.spans.front().p0.y) <=
        distance_epsilon) {
      boundary_paths.push_back(
          DerivedBoundaryPath{topology.id, Path{}, 0.0});
      continue;
    }
    const Result<double> length = PathLength(path);
    if (!length.ok) {
      return Result<bool>::Fail(length.failure_category, length.error);
    }
    boundary_paths.push_back(
        DerivedBoundaryPath{topology.id, std::move(path), length.value});
  }
  for (const LaneConnection &connection : graph.lane_connections) {
    if (connection.kind != LaneConnectionKind::kSplit)
      continue;
    std::vector<const DerivedBoundaryPath *> sides{};
    for (const BoundaryContinuation &boundary :
         graph.boundary_continuations) {
      if (boundary.source.segment_id != connection.source.segment_id ||
          boundary.target.segment_id != connection.target.segment_id)
        continue;
      const auto path = std::find_if(
          boundary_paths.begin(), boundary_paths.end(),
          [&boundary](const DerivedBoundaryPath &candidate) {
            return candidate.continuation_id == boundary.id;
          });
      if (path != boundary_paths.end() && path->path.spans.size() == 1)
        sides.push_back(&*path);
    }
    if (sides.size() != 2)
      continue;
    DerivedSeparationArea area{};
    area.connection_id = connection.id;
    constexpr int samples = 12;
    for (int index = 0; index <= samples; ++index) {
      const Vec2d point = evaluate_span(
          sides[0]->path.spans.front(),
          static_cast<double>(index) / static_cast<double>(samples));
      area.perimeter.push_back(to3(point));
    }
    for (int index = samples; index >= 0; --index) {
      const Vec2d point = evaluate_span(
          sides[1]->path.spans.front(),
          static_cast<double>(index) / static_cast<double>(samples));
      area.perimeter.push_back(to3(point));
    }
    separation_areas.push_back(std::move(area));
  }
  return Result<bool>::Ok(true);
}

} // namespace city::road::generation
