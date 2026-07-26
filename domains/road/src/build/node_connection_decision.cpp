#include "stages.hpp"

#include "stage_support.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>
#include <tuple>

namespace city::road::build {
namespace {

struct ConnectionPolicy {
  double straight_tolerance_rad = 5.0 * std::numbers::pi / 180.0;
  double minimum_connection_angle_rad = 45.0 * std::numbers::pi / 180.0;
  double maximum_connection_angle_rad = 135.0 * std::numbers::pi / 180.0;
  double corner_radius_m = 4.0;
  double minimum_junction_setback_m = 4.0;
  double connection_control_factor = 0.55;
  double junction_control_factor = 0.45;
  double parallel_sine_tolerance = 1e-3;
  std::size_t maximum_approaches = 4;
};

constexpr ConnectionPolicy kPolicy{};

struct OrderedApproach {
  ApproachKey key{};
  Vec2d tangent{};
  Vec2d chord{};
  std::int64_t angle_key = 0;
};

std::int64_t angle_key(Vec2d tangent) {
  constexpr double scale = 1.0e12;
  return static_cast<std::int64_t>(
      std::llround((std::atan2(tangent.y, tangent.x) + std::numbers::pi) * scale));
}

bool approach_key_less(const ApproachKey& a, const ApproachKey& b) {
  return std::tie(a.node_id, a.segment_id, a.endpoint_role) <
         std::tie(b.node_id, b.segment_id, b.endpoint_role);
}

double endpoint_outer_half_width(const SavedRoadGraph& graph,
                                 const RoadSegment& segment,
                                 const ApproachKey& key) {
  CrossSectionTemplateId template_id = segment.section_template;
  if (segment.transition.has_value()) {
    const SectionTransition* transition = FindTransition(graph, *segment.transition);
    if (transition == nullptr) return 0.0;
    template_id = key.endpoint_role == EndpointRole::kStart
                      ? transition->from_template
                      : transition->to_template;
  }
  const CrossSectionTemplate* section = FindTemplate(graph, template_id);
  if (section == nullptr) return 0.0;
  double width = 0.0;
  for (const SurfaceBand& band : section->bands) width += band.width_m;
  for (const BoundaryProfile& boundary : section->boundaries) width += boundary.width_m;
  return width * 0.5;
}

CrossSectionTemplateId endpoint_template_id(const SavedRoadGraph& graph,
                                            const RoadSegment& segment,
                                            const ApproachKey& key) {
  if (!segment.transition.has_value()) return segment.section_template;
  const SectionTransition* transition =
      FindTransition(graph, *segment.transition);
  if (transition == nullptr) return 0;
  return key.endpoint_role == EndpointRole::kStart
             ? transition->from_template
             : transition->to_template;
}

const OrderedApproach* find_ordered(const std::vector<OrderedApproach>& approaches,
                                    const ApproachKey& key) {
  const auto found = std::find_if(
      approaches.begin(), approaches.end(),
      [&key](const OrderedApproach& approach) { return approach.key == key; });
  return found == approaches.end() ? nullptr : &*found;
}

} // namespace

Result<bool> BuildNodeConnectionDecisions(BuildContext& context) {
  context.derived.node_connection_decisions.clear();
  context.derived.setback_calculation_count = 0;
  context.derived.node_connection_decisions.reserve(context.topology.size());

  for (const NodeTopology& topology : context.topology) {
    const NodeConnectionPolicyOverride* policy =
        FindPolicyOverride(context.authoritative, topology.node_id);
    if (topology.endpoints.size() <= 1 && policy == nullptr) {
      continue;
    }
    NodeConnectionDecision decision{};
    decision.node_id = topology.node_id;
    decision.corner_radius_m = kPolicy.corner_radius_m;

    std::vector<OrderedApproach> ordered{};
    ordered.reserve(topology.endpoints.size());
    for (const TopologyEndpoint& endpoint : topology.endpoints) {
      const ApproachKey key{
          topology.node_id, endpoint.segment_id, endpoint.endpoint_role};
      const RoadSegment* segment = FindSegment(context.authoritative, key.segment_id);
      const Path* alignment = FindAlignment(context.derived, key.segment_id);
      if (segment == nullptr || alignment == nullptr) {
        return Result<bool>::Fail(ErrorKind::kInternal, "road approach read model is missing");
      }
      const Result<Vec2d> tangent = InwardTangent(*segment, *alignment, key);
      if (!tangent.ok) return Result<bool>::Fail(tangent.error_kind, tangent.error);
      const RoadNode* node = FindNode(context.authoritative, key.node_id);
      const RoadNodeId other_id =
          key.endpoint_role == EndpointRole::kStart ? segment->node_b
                                                    : segment->node_a;
      const RoadNode* other = FindNode(context.authoritative, other_id);
      if (node == nullptr || other == nullptr) {
        return Result<bool>::Fail(ErrorKind::kInternal,
                                  "road approach chord endpoint is missing");
      }
      const Vec2d chord = Normalize(Subtract(other->position, node->position));
      ordered.push_back(
          OrderedApproach{key, tangent.value, chord, angle_key(tangent.value)});
    }
    std::sort(ordered.begin(), ordered.end(), [](const OrderedApproach& a, const OrderedApproach& b) {
      if (a.angle_key != b.angle_key) return a.angle_key < b.angle_key;
      return approach_key_less(a.key, b.key);
    });
    for (const OrderedApproach& approach : ordered) {
      decision.ordered_approaches.push_back(approach.key);
    }
    for (std::size_t a = 0; a < ordered.size(); ++a) {
      for (std::size_t b = a + 1; b < ordered.size(); ++b) {
        const double angle =
            std::acos(std::clamp(Dot(ordered[a].chord, ordered[b].chord),
                                 -1.0, 1.0));
        const bool opposite =
            Dot(ordered[a].tangent, ordered[b].tangent) <=
            -std::cos(kPolicy.straight_tolerance_rad);
        if (!opposite &&
            (angle < kPolicy.minimum_connection_angle_rad ||
             angle > kPolicy.maximum_connection_angle_rad)) {
          return Result<bool>::Fail(
              ErrorKind::kUnsupported,
              "road connected approach angle is outside supported range");
        }
      }
    }

    if (policy != nullptr) {
      decision.applied_policy_override_id = policy->id;
      if (policy->policy == NodeConnectionPolicy::kForcePassThrough) {
        decision.kind = NodeConnectionKind::kPassThrough;
      } else if (policy->policy == NodeConnectionPolicy::kForceCorner) {
        decision.kind = NodeConnectionKind::kCorner;
      } else {
        decision.kind = NodeConnectionKind::kJunction;
      }
      decision.reason = "explicit policy override";
    } else if (ordered.size() <= 1) {
      decision.kind = NodeConnectionKind::kPassThrough;
      decision.reason = "endpoint";
    } else if (ordered.size() == 2) {
      const bool straight =
          Dot(ordered[0].tangent, ordered[1].tangent) <=
          -std::cos(kPolicy.straight_tolerance_rad);
      decision.kind = straight ? NodeConnectionKind::kPassThrough
                               : NodeConnectionKind::kCorner;
      decision.reason = straight ? "two aligned approaches" : "two turning approaches";
    } else if (ordered.size() <= kPolicy.maximum_approaches) {
      decision.kind = NodeConnectionKind::kJunction;
      decision.reason = "three or four approaches";
    } else {
      decision.kind = NodeConnectionKind::kUnsupported;
      decision.reason = "more than four approaches";
      return Result<bool>::Fail(ErrorKind::kUnsupported,
                                "road node has more than four approaches");
    }

    double minimum_setback = std::numeric_limits<double>::infinity();
    for (const OrderedApproach& approach : ordered) {
      const RoadSegment* segment = FindSegment(context.authoritative, approach.key.segment_id);
      const Path* alignment = FindAlignment(context.derived, approach.key.segment_id);
      if (segment == nullptr || alignment == nullptr) {
        return Result<bool>::Fail(ErrorKind::kInternal, "road approach source is missing");
      }
      const Result<double> length = PathLength(*alignment);
      if (!length.ok) return Result<bool>::Fail(length.error_kind, length.error);

      double setback = 0.0;
      if (decision.kind == NodeConnectionKind::kCorner) {
        const OrderedApproach* other =
            ordered[0].key == approach.key ? &ordered[1] : &ordered[0];
        const double outward_angle =
            std::acos(std::clamp(Dot(approach.tangent, other->tangent), -1.0, 1.0));
        const double turn_angle = std::numbers::pi - outward_angle;
        setback = kPolicy.corner_radius_m * std::tan(turn_angle * 0.5);
      } else if (decision.kind == NodeConnectionKind::kJunction) {
        setback = kPolicy.minimum_junction_setback_m;
        for (const OrderedApproach& other : ordered) {
          if (other.key == approach.key) continue;
          const double sine = std::abs(Cross(approach.tangent, other.tangent));
          if (sine <= kPolicy.parallel_sine_tolerance) continue;
          const RoadSegment* other_segment =
              FindSegment(context.authoritative, other.key.segment_id);
          if (other_segment == nullptr) {
            return Result<bool>::Fail(ErrorKind::kInternal,
                                      "road setback source segment is missing");
          }
          setback = std::max(
              setback,
              endpoint_outer_half_width(context.authoritative, *other_segment, other.key) /
                  sine);
        }
      }
      ++context.derived.setback_calculation_count;
      if (!IsFinite(setback) || setback < 0.0 || setback > length.value + kStationEpsilon) {
        return Result<bool>::Fail(ErrorKind::kUnsupported,
                                  "road approach setback exceeds the segment length");
      }
      const double gate_station =
          approach.key.endpoint_role == EndpointRole::kStart
              ? setback
              : length.value - setback;
      const CrossSectionTemplateId endpoint_section =
          endpoint_template_id(context.authoritative, *segment, approach.key);
      if (endpoint_section == 0) {
        return Result<bool>::Fail(
            ErrorKind::kValidation,
            "road approach endpoint section template is missing");
      }
      decision.approaches.push_back(ApproachConnectionDecision{
          approach.key, endpoint_section, setback, gate_station});
      minimum_setback = std::min(minimum_setback, setback);
    }
    if (decision.approaches.size() > 1) {
      const CrossSectionTemplateId expected =
          decision.approaches.front().endpoint_template_id;
      if (std::any_of(
              decision.approaches.begin() + 1, decision.approaches.end(),
              [expected](const ApproachConnectionDecision& approach) {
                return approach.endpoint_template_id != expected;
              })) {
        return Result<bool>::Fail(
            ErrorKind::kUnsupported,
            "road connected approaches require identical endpoint section template IDs");
      }
    }
    if (!std::isfinite(minimum_setback)) minimum_setback = 0.0;
    decision.corner_control_m = minimum_setback * kPolicy.connection_control_factor;
    decision.junction_corner_control_m =
        minimum_setback * kPolicy.junction_control_factor;
    context.derived.node_connection_decisions.push_back(std::move(decision));
  }
  return Result<bool>::Ok(true);
}

} // namespace city::road::build
