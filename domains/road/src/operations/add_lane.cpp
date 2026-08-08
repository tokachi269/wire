#include "city/road/road.hpp"

#include "../geometry/geometry.hpp"
#include "../geometry/section.hpp"
#include "../lookup.hpp"
#include "operation_plan.hpp"
#include "../geometry/alignment.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <unordered_map>

// Add Lane. Core resolves the outer lane and the boundary it keeps fixed,
// derives the widened cross section, writes one section transition, and plans
// the junction movements the new lane resolves to -- all in one operation.
namespace city::road {
namespace {

using internal::distance;
using internal::find_segment;
using internal::find_template;
using internal::is_finite;
using internal::distance_epsilon;
using internal::PathSplit;
using internal::split_path_at_distance;

[[nodiscard]] EndpointRole endpoint_role_at(const RoadSegment& segment,
                                            RoadNodeId node_id) {
  return segment.node_a == node_id ? EndpointRole::kStart
                                   : EndpointRole::kEnd;
}

[[nodiscard]] bool lane_exits(const LaneBand& lane, EndpointRole role) {
  return (lane.direction == LaneTravelDirection::kAlongSegment &&
          role == EndpointRole::kEnd) ||
         (lane.direction == LaneTravelDirection::kAgainstSegment &&
          role == EndpointRole::kStart);
}

[[nodiscard]] bool lane_enters(const LaneBand& lane, EndpointRole role) {
  return (lane.direction == LaneTravelDirection::kAlongSegment &&
          role == EndpointRole::kStart) ||
         (lane.direction == LaneTravelDirection::kAgainstSegment &&
          role == EndpointRole::kEnd);
}

struct OrderedLaneEndpoint {
  LaneEndpointKey key{};
  double lateral_m = 0.0;
};

[[nodiscard]] Result<std::vector<OrderedLaneEndpoint>> ordered_lane_endpoints(
    const RoadLayoutTemplate& section, RoadSegmentId segment_id,
    EndpointRole role, bool exits) {
  std::vector<OrderedLaneEndpoint> result{};
  const double endpoint_sign = role == EndpointRole::kStart ? 1.0 : -1.0;
  for (const LaneBand& lane : section.lane_bands) {
    if ((exits && !lane_exits(lane, role)) ||
        (!exits && !lane_enters(lane, role))) {
      continue;
    }
    const Result<double> lateral =
        internal::lane_template_lateral(section, lane);
    if (!lateral.ok) {
      return Result<std::vector<OrderedLaneEndpoint>>::Fail(
          lateral.failure_category, lateral.error);
    }
    result.push_back({LaneEndpointKey{segment_id, lane.id, role},
                      lateral.value * endpoint_sign});
  }
  std::sort(result.begin(), result.end(),
            [](const OrderedLaneEndpoint& a, const OrderedLaneEndpoint& b) {
              return std::tie(a.lateral_m, a.key.lane_id) <
                     std::tie(b.lateral_m, b.key.lane_id);
            });
  return Result<std::vector<OrderedLaneEndpoint>>::Ok(std::move(result));
}

struct OrderedBoundaryEndpoint {
  BoundaryEndpointKey key{};
  BoundaryRole role = BoundaryRole::kCurb;
  StripFunction left_function = StripFunction::kCarriageway;
  StripFunction right_function = StripFunction::kCarriageway;
  double lateral_order = 0.0;
};

[[nodiscard]] std::vector<OrderedBoundaryEndpoint> ordered_boundary_endpoints(
    const RoadLayoutTemplate& section, RoadSegmentId segment_id,
    EndpointRole role) {
  std::vector<OrderedBoundaryEndpoint> result{};
  const double endpoint_sign = role == EndpointRole::kStart ? 1.0 : -1.0;
  for (std::size_t index = 0; index < section.boundaries.size(); ++index) {
    const BoundaryProfile& boundary = section.boundaries[index];
    const StripFunction first = section.strips[index].function;
    const StripFunction second = section.strips[index + 1].function;
    result.push_back({BoundaryEndpointKey{segment_id, boundary.boundary_id, role},
                      boundary.role,
                      endpoint_sign > 0.0 ? first : second,
                      endpoint_sign > 0.0 ? second : first,
                      static_cast<double>(index) * endpoint_sign});
  }
  std::sort(result.begin(), result.end(),
            [](const OrderedBoundaryEndpoint& a,
               const OrderedBoundaryEndpoint& b) {
              return std::tie(a.lateral_order, a.key.boundary_id) <
                     std::tie(b.lateral_order, b.key.boundary_id);
            });
  return result;
}

[[nodiscard]] Result<bool> plan_unique_junction_topology(
    const SavedRoadGraph& graph, RoadSegmentId source_segment_id,
    EndpointRole source_role, LaneId added_lane_id,
    BoundaryId added_boundary_id, std::uint64_t& next_id,
    operations::OperationPlan& plan) {
  const RoadSegment* source_segment =
      internal::find_segment(graph, source_segment_id);
  const RoadLayoutTemplate* source_section =
      source_segment == nullptr
          ? nullptr
          : internal::find_endpoint_template(graph, *source_segment,
                                             source_role);
  if (source_segment == nullptr || source_section == nullptr) {
    return Result<bool>::Fail(CommitFailureCategory::kInternalError,
                              "added lane junction source is missing");
  }
  const RoadNodeId node_id = source_role == EndpointRole::kStart
                                 ? source_segment->node_a
                                 : source_segment->node_b;
  std::vector<const RoadSegment*> incident{};
  for (const RoadSegment& segment : graph.segments) {
    if (segment.node_a == node_id || segment.node_b == node_id)
      incident.push_back(&segment);
  }
  if (incident.size() < 2) return Result<bool>::Ok(true);

  const Result<std::vector<OrderedLaneEndpoint>> source_lanes =
      ordered_lane_endpoints(*source_section, source_segment_id, source_role,
                             true);
  if (!source_lanes.ok) {
    return Result<bool>::Fail(source_lanes.failure_category, source_lanes.error);
  }
  if (std::none_of(source_lanes.value.begin(), source_lanes.value.end(),
                   [added_lane_id](const OrderedLaneEndpoint& endpoint) {
                     return endpoint.key.lane_id == added_lane_id;
                   })) {
    return Result<bool>::Ok(true);
  }
  std::vector<OrderedLaneEndpoint> mapped_source_lanes = source_lanes.value;
  std::vector<OrderedBoundaryEndpoint> source_boundaries =
      ordered_boundary_endpoints(*source_section, source_segment_id,
                                 source_role);
  struct Candidate {
    std::vector<OrderedLaneEndpoint> lanes{};
    std::vector<OrderedBoundaryEndpoint> boundaries{};
  };
  std::vector<Candidate> candidates{};
  for (const RoadSegment* candidate_segment : incident) {
    if (candidate_segment->id == source_segment_id) continue;
    const EndpointRole role = endpoint_role_at(*candidate_segment, node_id);
    const RoadLayoutTemplate* section = internal::find_endpoint_template(
        graph, *candidate_segment, role);
    if (section == nullptr) {
      return Result<bool>::Fail(CommitFailureCategory::kInternalError,
                                "junction candidate section is missing");
    }
    Result<std::vector<OrderedLaneEndpoint>> lanes = ordered_lane_endpoints(
        *section, candidate_segment->id, role, false);
    if (!lanes.ok) {
      return Result<bool>::Fail(lanes.failure_category, lanes.error);
    }
    std::vector<OrderedBoundaryEndpoint> boundaries =
        ordered_boundary_endpoints(*section, candidate_segment->id, role);
    if (incident.size() == 2) {
      mapped_source_lanes.erase(
          std::remove_if(mapped_source_lanes.begin(), mapped_source_lanes.end(),
                         [added_lane_id](const OrderedLaneEndpoint& endpoint) {
                           return endpoint.key.lane_id == added_lane_id;
                         }),
          mapped_source_lanes.end());
      source_boundaries.erase(
          std::remove_if(source_boundaries.begin(), source_boundaries.end(),
                         [added_boundary_id](
                             const OrderedBoundaryEndpoint& endpoint) {
                           return endpoint.key.boundary_id == added_boundary_id;
                         }),
          source_boundaries.end());
    }
    if (lanes.value.size() != mapped_source_lanes.size() ||
        boundaries.size() != source_boundaries.size()) {
      continue;
    }
    const bool boundary_roles_match = std::equal(
        source_boundaries.begin(), source_boundaries.end(), boundaries.begin(),
        [](const OrderedBoundaryEndpoint& a,
           const OrderedBoundaryEndpoint& b) {
          return a.role == b.role &&
                 a.left_function == b.left_function &&
                 a.right_function == b.right_function;
        });
    if (boundary_roles_match)
      candidates.push_back({std::move(lanes.value), std::move(boundaries)});
  }
  if (candidates.empty()) return Result<bool>::Ok(true);
  if (candidates.size() > 1) {
    return Result<bool>::Fail(
        CommitFailureCategory::kNotImplemented,
        "added lane junction destination is ambiguous; choose straight, left, "
        "or right");
  }
  const Candidate& target = candidates.front();
  for (std::size_t index = 0; index < mapped_source_lanes.size(); ++index) {
    const LaneEndpointKey& source = mapped_source_lanes[index].key;
    const LaneEndpointKey& destination = target.lanes[index].key;
    const auto exact = std::find_if(
        graph.lane_connections.begin(), graph.lane_connections.end(),
        [&source, &destination](const LaneConnection& connection) {
          return connection.source == source &&
                 connection.target == destination;
        });
    if (exact != graph.lane_connections.end()) continue;
    const bool conflicts = std::any_of(
        graph.lane_connections.begin(), graph.lane_connections.end(),
        [&source, &destination](const LaneConnection& connection) {
          return connection.source == source ||
                 connection.target == destination;
        });
    if (conflicts) {
      return Result<bool>::Fail(
          CommitFailureCategory::kNotImplemented,
          "added lane conflicts with existing junction lane topology");
    }
    plan.add_lane_connections.push_back(LaneConnection{
        next_id++, source, destination,
        incident.size() == 2 ? LaneConnectionKind::kContinuation
                             : LaneConnectionKind::kJunctionMovement});
  }
  for (std::size_t index = 0; index < source_boundaries.size(); ++index) {
    const BoundaryEndpointKey& source = source_boundaries[index].key;
    const BoundaryEndpointKey& destination = target.boundaries[index].key;
    const auto exact = std::find_if(
        graph.boundary_continuations.begin(),
        graph.boundary_continuations.end(),
        [&source, &destination](const BoundaryContinuation& continuation) {
          return continuation.source == source &&
                 continuation.target == destination;
        });
    if (exact != graph.boundary_continuations.end()) continue;
    const bool conflicts = std::any_of(
        graph.boundary_continuations.begin(),
        graph.boundary_continuations.end(),
        [&source, &destination](const BoundaryContinuation& continuation) {
          return continuation.source == source ||
                 continuation.target == destination;
        });
    if (conflicts) {
      return Result<bool>::Fail(
          CommitFailureCategory::kNotImplemented,
          "added lane conflicts with existing junction boundary topology");
    }
    plan.add_boundary_continuations.push_back(BoundaryContinuation{
        next_id++, source, destination,
        BoundaryContinuationKind::kContinuation});
  }
  return Result<bool>::Ok(true);
}

struct OuterLaneSelection {
  std::size_t strip_index = 0;
};

[[nodiscard]] Result<OuterLaneSelection> select_outer_lane_strip(
    const RoadLayoutTemplate& section, LaneTravelDirection direction,
    RoadSide side, CommitFailureCategory empty_category,
    const char* empty_error, const char* missing_strip_error) {
  struct Candidate {
    std::size_t strip_index = 0;
    double center_in_strip_m = 0.0;
  };
  std::vector<Candidate> candidates{};
  for (const LaneBand& lane : section.lane_bands) {
    if (lane.direction != direction) continue;
    const auto strip = std::find_if(
        section.strips.begin(), section.strips.end(),
        [&lane](const RoadLayoutStrip& item) {
          return item.id == lane.surface_strip_id;
        });
    if (strip == section.strips.end()) {
      return Result<OuterLaneSelection>::Fail(
          CommitFailureCategory::kInternalError, missing_strip_error);
    }
    candidates.push_back(Candidate{
        static_cast<std::size_t>(
            std::distance(section.strips.begin(), strip)),
        (lane.lateral_start_m + lane.lateral_end_m) * 0.5});
  }
  if (candidates.empty()) {
    return Result<OuterLaneSelection>::Fail(empty_category, empty_error);
  }
  const auto extremes = std::minmax_element(
      candidates.begin(), candidates.end(),
      [](const Candidate& a, const Candidate& b) {
        return std::tie(a.strip_index, a.center_in_strip_m) <
               std::tie(b.strip_index, b.center_in_strip_m);
      });
  return Result<OuterLaneSelection>::Ok(
      OuterLaneSelection{side == RoadSide::kRight
                             ? extremes.second->strip_index
                             : extremes.first->strip_index});
}

struct LaneSectionIds {
  RoadLayoutStripId strip_id = 0;
  LaneId lane_id = 0;
  BoundaryId boundary_id = 0;
};

[[nodiscard]] std::uint64_t next_section_member_id(
    const SavedRoadGraph& graph) {
  std::uint64_t next_id = 1;
  for (const RoadLayoutTemplate& section : graph.layout_templates) {
    for (const RoadLayoutStrip& strip : section.strips)
      next_id = std::max(next_id, strip.id + 1);
    for (const LaneBand& lane : section.lane_bands)
      next_id = std::max(next_id, lane.id + 1);
    for (const BoundaryProfile& boundary : section.boundaries)
      next_id = std::max(next_id, boundary.boundary_id + 1);
  }
  return next_id;
}

[[nodiscard]] Result<RoadLayoutTemplate> make_extended_lane_section(
    const RoadLayoutTemplate& base, LaneTravelDirection direction,
    RoadSide side, double lane_width_m, LaneSectionIds ids,
    CommitFailureCategory empty_category, const char* empty_error,
    const char* missing_strip_error) {
  const Result<OuterLaneSelection> selected = select_outer_lane_strip(
      base, direction, side, empty_category, empty_error,
      missing_strip_error);
  if (!selected.ok) {
    return Result<RoadLayoutTemplate>::Fail(selected.failure_category,
                                               selected.error);
  }

  RoadLayoutTemplate extended = base;
  RoadLayoutStrip strip = base.strips[selected.value.strip_index];
  strip.id = ids.strip_id;
  strip.width_m = lane_width_m;
  strip.side_marking = {};
  const BoundaryProfile divider{
      ids.boundary_id,
      BoundaryRole::kLaneDivider,
      {ProfilePoint{0.0, 0.0}},
      {},
      AutoMarkingPolicy{true, MarkingRole::kLaneSeparator,
                        builtin_marking_styles::kWhiteDashed,
                        MarkingPlacement::kCenter}};
  // The alignment keeps its distance to the side that is not growing, so every
  // element beyond the insertion keeps its lateral position.
  if (side == RoadSide::kRight) {
    extended.strips.insert(
        extended.strips.begin() + selected.value.strip_index + 1, strip);
    extended.boundaries.insert(
        extended.boundaries.begin() + selected.value.strip_index, divider);
  } else {
    extended.strips.insert(
        extended.strips.begin() + selected.value.strip_index, strip);
    extended.boundaries.insert(
        extended.boundaries.begin() + selected.value.strip_index, divider);
    extended.alignment_offset_from_left_m += lane_width_m;
  }
  extended.lane_bands.push_back(
      LaneBand{ids.lane_id, ids.strip_id, 0.0, lane_width_m, direction});
  return Result<RoadLayoutTemplate>::Ok(std::move(extended));
}

struct LaneSectionExtension {
  RoadLayoutTemplateId source_id = 0;
  LaneTravelDirection direction = LaneTravelDirection::kAlongSegment;
  RoadSide side = RoadSide::kRight;
  RoadLayoutTemplateId target_id = 0;
};

[[nodiscard]] Result<RoadLayoutTemplateId> ensure_extended_lane_section(
    const RoadLayoutTemplate& base, LaneTravelDirection direction,
    RoadSide side, double lane_width_m, LaneSectionIds ids,
    std::vector<LaneSectionExtension>& extensions,
    operations::OperationPlan& plan, std::uint64_t& next_id) {
  const auto existing = std::find_if(
      extensions.begin(), extensions.end(),
      [&base, direction, side](const LaneSectionExtension& extension) {
        return extension.source_id == base.id &&
               extension.direction == direction && extension.side == side;
      });
  if (existing != extensions.end()) {
    return Result<RoadLayoutTemplateId>::Ok(existing->target_id);
  }
  Result<RoadLayoutTemplate> extended = make_extended_lane_section(
      base, direction, side, lane_width_m, ids,
      CommitFailureCategory::kNotImplemented,
      "later section has no lane in the selected direction",
      "lane extension source strip is missing");
  if (!extended.ok) {
    return Result<RoadLayoutTemplateId>::Fail(extended.failure_category,
                                                extended.error);
  }
  extended.value.id = next_id++;
  const RoadLayoutTemplateId id = extended.value.id;
  plan.add_layout_templates.push_back(std::move(extended.value));
  extensions.push_back(LaneSectionExtension{base.id, direction, side, id});
  return Result<RoadLayoutTemplateId>::Ok(id);
}

[[nodiscard]] Result<bool> validate_add_lane_request(
    const AddLaneRequest& request) {
  if (!is_finite(request.transition_start.t) ||
      !is_finite(request.transition_complete.t) ||
      !is_finite(request.continuation_end.t)) {
    return Result<bool>::Fail(CommitFailureCategory::kInvalidInput,
                              "lane transition positions must be finite");
  }
  if (request.transition_start.segment_id == 0 ||
      request.transition_complete.segment_id == 0 ||
      request.continuation_end.segment_id == 0 ||
      request.transition_start.t < 0.0 ||
      request.transition_start.t > 1.0 ||
      request.transition_complete.t < 0.0 ||
      request.transition_complete.t > 1.0 ||
      request.continuation_end.t < 0.0 ||
      request.continuation_end.t > 1.0) {
    return Result<bool>::Fail(
        CommitFailureCategory::kInvalidInput,
        "lane transition positions or continuation endpoint are invalid");
  }
  if (!is_finite(request.lane_width_m) || request.lane_width_m <= 0.0) {
    return Result<bool>::Fail(CommitFailureCategory::kInvalidInput,
                              "lane width must be finite and positive");
  }
  return Result<bool>::Ok(true);
}

struct ResolvedLanePosition {
  std::size_t ref_index = 0;
  double segment_length_m = 0.0;
  double corridor_distance_m = 0.0;
};

[[nodiscard]] Result<ResolvedLanePosition> resolve_lane_position(
    const RoadCorridor& corridor, const DerivedRoad& derived,
    SegmentPosition position) {
  double accumulated = 0.0;
  for (std::size_t index = 0; index < corridor.segments.size(); ++index) {
    const DirectedSegmentRef& ref = corridor.segments[index];
    const DerivedSegment* segment = FindDerivedSegment(derived, ref.segment_id);
    if (segment == nullptr || segment->length_m <= distance_epsilon) {
      return Result<ResolvedLanePosition>::Fail(
          CommitFailureCategory::kInternalError,
          "lane corridor segment geometry is missing");
    }
    if (ref.segment_id == position.segment_id) {
      const double local_distance = position.t * segment->length_m;
      return Result<ResolvedLanePosition>::Ok(ResolvedLanePosition{
          index, segment->length_m,
          accumulated +
              (ref.reversed ? segment->length_m - local_distance
                            : local_distance)});
    }
    accumulated += segment->length_m;
  }
  return Result<ResolvedLanePosition>::Fail(
      CommitFailureCategory::kInvalidInput,
      "lane position is not on the selected corridor");
}
} // namespace

Result<LaneId> RoadState::AddLane(AddLaneRequest request) {
  const Result<bool> valid_request = validate_add_lane_request(request);
  if (!valid_request.ok) {
    return Result<LaneId>::Fail(valid_request.failure_category,
                                valid_request.error);
  }
  const RoadCorridor* corridor =
      FindRoadCorridor(graph_, request.corridor_id);
  if (corridor == nullptr || corridor->segments.empty()) {
    return Result<LaneId>::Fail(CommitFailureCategory::kInvalidInput,
                                "lane transition corridor does not exist");
  }
  const Result<ResolvedLanePosition> start_position =
      resolve_lane_position(*corridor, derived_, request.transition_start);
  const Result<ResolvedLanePosition> complete_position =
      resolve_lane_position(*corridor, derived_, request.transition_complete);
  const Result<ResolvedLanePosition> end_position =
      resolve_lane_position(*corridor, derived_, request.continuation_end);
  if (!start_position.ok || !complete_position.ok || !end_position.ok) {
    const auto& failed = !start_position.ok
                             ? start_position
                             : (!complete_position.ok ? complete_position
                                                      : end_position);
    return Result<LaneId>::Fail(failed.failure_category, failed.error);
  }
  if (complete_position.value.corridor_distance_m -
              start_position.value.corridor_distance_m <=
          distance_epsilon ||
      end_position.value.corridor_distance_m + distance_epsilon <
          complete_position.value.corridor_distance_m) {
    return Result<LaneId>::Fail(
        CommitFailureCategory::kInvalidInput,
        "lane positions must follow start, completion, and end order on the corridor");
  }
  const RoadSegment* segment =
      find_segment(graph_, request.transition_start.segment_id);
  if (segment == nullptr) {
    return Result<LaneId>::Fail(CommitFailureCategory::kInternalError,
                                "lane transition segment is missing");
  }
  const auto selected_ref =
      corridor->segments.begin() +
      static_cast<std::ptrdiff_t>(start_position.value.ref_index);
  const LaneTravelDirection local_direction =
      selected_ref->reversed
          ? (request.direction == LaneTravelDirection::kAlongSegment
                 ? LaneTravelDirection::kAgainstSegment
                 : LaneTravelDirection::kAlongSegment)
          : request.direction;
  const RoadSide local_side =
      selected_ref->reversed
          ? (request.side == RoadSide::kLeft ? RoadSide::kRight
                                             : RoadSide::kLeft)
          : request.side;
  const RoadLayoutTemplate* source =
      find_template(graph_, segment->layout_template);
  if (source == nullptr) {
    return Result<LaneId>::Fail(CommitFailureCategory::kInternalError,
                                "lane transition section is missing");
  }
  if (segment->transition.has_value()) {
    return Result<LaneId>::Fail(
        CommitFailureCategory::kInvalidInput,
        "this road segment already has a section transition; overlapping lane changes are not supported");
  }
  const Result<OuterLaneSelection> selected = select_outer_lane_strip(
      *source, local_direction, local_side,
      CommitFailureCategory::kInvalidInput,
      "selected direction has no lane to extend",
      "lane transition source lane strip is missing");
  if (!selected.ok) {
    return Result<LaneId>::Fail(selected.failure_category, selected.error);
  }
  const std::size_t lane_strip_index = selected.value.strip_index;
  const std::optional<std::size_t> anchor_index =
      local_side == RoadSide::kRight
          ? (lane_strip_index > 0
                 ? std::optional<std::size_t>{lane_strip_index - 1}
                 : std::nullopt)
          : (lane_strip_index < source->boundaries.size()
                 ? std::optional<std::size_t>{lane_strip_index}
                 : std::nullopt);
  const TransitionAnchor anchor =
      anchor_index.has_value()
          ? TransitionAnchor::kBoundary
          : (local_side == RoadSide::kRight ? TransitionAnchor::kLeftEdge
                                             : TransitionAnchor::kRightEdge);
  const BoundaryId anchor_boundary_id =
      anchor_index.has_value()
          ? source->boundaries[*anchor_index].boundary_id
          : 0;

  std::uint64_t next_local_id = next_section_member_id(graph_);
  const LaneSectionIds lane_ids{next_local_id++, next_local_id++,
                                next_local_id++};
  Result<RoadLayoutTemplate> target_result = make_extended_lane_section(
      *source, local_direction, local_side, request.lane_width_m, lane_ids,
      CommitFailureCategory::kInvalidInput,
      "selected direction has no lane to extend",
      "lane transition source lane strip is missing");
  if (!target_result.ok) {
    return Result<LaneId>::Fail(target_result.failure_category,
                                target_result.error);
  }
  RoadLayoutTemplate target = std::move(target_result.value);
  const LaneId added_lane_id = lane_ids.lane_id;

  operations::OperationPlan plan{};
  std::uint64_t next_id = next_id_;
  target.id = next_id++;
  const auto full_ref =
      corridor->segments.begin() +
      static_cast<std::ptrdiff_t>(complete_position.value.ref_index);
  std::vector<LaneSectionExtension> section_extensions{
      LaneSectionExtension{source->id, local_direction, local_side,
                           target.id}};
  const auto continuation_ref =
      corridor->segments.begin() +
      static_cast<std::ptrdiff_t>(end_position.value.ref_index);
  RoadSegmentId terminal_segment_id = continuation_ref->segment_id;
  EndpointRole terminal_role = continuation_ref->reversed
                                   ? EndpointRole::kStart
                                   : EndpointRole::kEnd;
  const bool split_terminal = request.continuation_end.t > distance_epsilon &&
                              request.continuation_end.t <
                                  1.0 - distance_epsilon;
  if (split_terminal) {
    const RoadSegment* terminal =
        find_segment(graph_, continuation_ref->segment_id);
    const Path* terminal_path = terminal == nullptr
                                    ? nullptr
                                    : FindCanonicalAlignment(derived_, terminal->id);
    if (terminal == nullptr || terminal_path == nullptr) {
      return Result<LaneId>::Fail(CommitFailureCategory::kInternalError,
                                  "lane continuation segment is missing");
    }
    if (terminal->transition.has_value()) {
      return Result<LaneId>::Fail(
          CommitFailureCategory::kNotImplemented,
          "lane continuation end cannot split an existing section transition");
    }
    const bool has_owned_state =
        std::any_of(graph_.manual_lines.begin(), graph_.manual_lines.end(),
                    [terminal](const ManualLineMarking& item) {
                      return item.owner_segment_id == terminal->id;
                    }) ||
        std::any_of(graph_.manual_areas.begin(), graph_.manual_areas.end(),
                    [terminal](const ManualAreaMarking& item) {
                      return item.owner_segment_id == terminal->id;
                    }) ||
        std::any_of(graph_.auto_marking_overrides.begin(),
                    graph_.auto_marking_overrides.end(),
                    [terminal](const AutoMarkingOverride& item) {
                      return item.key.owner.kind ==
                                 MarkingOwner::Kind::kRoadSegment &&
                             item.key.owner.segment_id == terminal->id;
                    });
    if (has_owned_state) {
      return Result<LaneId>::Fail(
          CommitFailureCategory::kNotImplemented,
          "lane continuation split crosses segment-owned marking state");
    }
    const Result<PathSplit> split = split_path_at_distance(
        *terminal_path,
        request.continuation_end.t * end_position.value.segment_length_m);
    if (!split.ok) {
      return Result<LaneId>::Fail(split.failure_category, split.error);
    }
    const Result<SegmentShape> first_shape =
        SegmentShapeFromPath(split.value.before);
    const Result<SegmentShape> second_shape =
        SegmentShapeFromPath(split.value.after);
    if (!first_shape.ok || !second_shape.ok) {
      return Result<LaneId>::Fail(
          CommitFailureCategory::kInternalError,
          "lane continuation split shape derivation failed");
    }
    const RoadNodeId split_node = next_id++;
    const RoadSegmentId second_id = next_id++;
    RoadSegment first = *terminal;
    first.node_b = split_node;
    first.shape = first_shape.value;
    plan.replace_segments.push_back(std::move(first));
    plan.add_nodes.push_back(RoadNode{split_node, split.value.point});
    plan.add_segments.push_back(RoadSegment{
        second_id, split_node, terminal->node_b, second_shape.value,
        terminal->layout_template, std::nullopt});
    RoadCorridor replacement = *corridor;
    const std::size_t index = end_position.value.ref_index;
    replacement.segments.erase(
        replacement.segments.begin() + static_cast<std::ptrdiff_t>(index));
    const std::array<DirectedSegmentRef, 2> split_refs =
        continuation_ref->reversed
            ? std::array<DirectedSegmentRef, 2>{
                  DirectedSegmentRef{second_id, true},
                  DirectedSegmentRef{terminal->id, true}}
            : std::array<DirectedSegmentRef, 2>{
                  DirectedSegmentRef{terminal->id, false},
                  DirectedSegmentRef{second_id, false}};
    replacement.segments.insert(
        replacement.segments.begin() + static_cast<std::ptrdiff_t>(index),
        split_refs.begin(), split_refs.end());
    plan.replace_corridors.push_back(std::move(replacement));
    if (continuation_ref->reversed) {
      terminal_segment_id = second_id;
      terminal_role = EndpointRole::kStart;
    } else {
      terminal_segment_id = terminal->id;
      terminal_role = EndpointRole::kEnd;
    }
  }

  auto editable_segment = [&](RoadSegmentId id) -> RoadSegment* {
    const auto replacement = std::find_if(
        plan.replace_segments.begin(), plan.replace_segments.end(),
        [id](const RoadSegment& item) { return item.id == id; });
    if (replacement != plan.replace_segments.end()) return &*replacement;
    const auto addition = std::find_if(
        plan.add_segments.begin(), plan.add_segments.end(),
        [id](const RoadSegment& item) { return item.id == id; });
    if (addition != plan.add_segments.end()) return &*addition;
    const RoadSegment* source_segment = find_segment(graph_, id);
    if (source_segment == nullptr) return nullptr;
    plan.replace_segments.push_back(*source_segment);
    return &plan.replace_segments.back();
  };
  const auto execute_lane_plan = [&]() -> Result<LaneId> {
    plan.next_id_after = next_id;
    SavedRoadGraph planned_graph = graph_;
    std::uint64_t planned_next_id = next_id_;
    const Result<bool> planned =
        operations::Apply(plan, planned_graph, planned_next_id);
    if (!planned.ok) {
      return Result<LaneId>::Fail(planned.failure_category, planned.error);
    }
    const RoadCorridor* planned_corridor =
        FindRoadCorridor(planned_graph, request.corridor_id);
    if (planned_corridor == nullptr || planned_corridor->segments.empty()) {
      return Result<LaneId>::Fail(CommitFailureCategory::kInternalError,
                                  "planned lane corridor is missing");
    }
    const Result<bool> topology = plan_unique_junction_topology(
        planned_graph, terminal_segment_id, terminal_role, added_lane_id,
        lane_ids.boundary_id, next_id, plan);
    if (!topology.ok) {
      return Result<LaneId>::Fail(topology.failure_category, topology.error);
    }
    plan.next_id_after = next_id;
    const Result<bool> executed = Execute(plan);
    if (!executed.ok) {
      return Result<LaneId>::Fail(executed.failure_category, executed.error);
    }
    return Result<LaneId>::Ok(added_lane_id);
  };

  {
    std::vector<double> corridor_starts(corridor->segments.size(), 0.0);
    double corridor_length = 0.0;
    for (std::size_t index = 0; index < corridor->segments.size(); ++index) {
      corridor_starts[index] = corridor_length;
      const DerivedSegment* item =
          FindDerivedSegment(derived_, corridor->segments[index].segment_id);
      if (item == nullptr || item->length_m <= distance_epsilon) {
        return Result<LaneId>::Fail(
            CommitFailureCategory::kInternalError,
            "lane corridor segment geometry is missing");
      }
      corridor_length += item->length_m;
    }
    const double taper_length = complete_position.value.corridor_distance_m -
                                start_position.value.corridor_distance_m;
    auto full_section_id = [&](const RoadLayoutTemplate& base,
                               LaneTravelDirection direction,
                               RoadSide side) -> Result<RoadLayoutTemplateId> {
      return ensure_extended_lane_section(
          base, direction, side, request.lane_width_m, lane_ids,
          section_extensions, plan, next_id);
    };
    auto section_at_fraction = [&](const RoadLayoutTemplate& base,
                                   LaneTravelDirection direction,
                                   RoadSide side,
                                   double fraction) -> Result<RoadLayoutTemplateId> {
      if (fraction <= distance_epsilon)
        return Result<RoadLayoutTemplateId>::Ok(base.id);
      if (fraction >= 1.0 - distance_epsilon)
        return full_section_id(base, direction, side);
      Result<RoadLayoutTemplate> partial = make_extended_lane_section(
          base, direction, side, request.lane_width_m * fraction, lane_ids,
          CommitFailureCategory::kNotImplemented,
          "later section has no lane in the selected direction",
          "lane extension source strip is missing");
      if (!partial.ok) {
        return Result<RoadLayoutTemplateId>::Fail(partial.failure_category,
                                                   partial.error);
      }
      partial.value.id = next_id++;
      const RoadLayoutTemplateId id = partial.value.id;
      plan.add_layout_templates.push_back(std::move(partial.value));
      return Result<RoadLayoutTemplateId>::Ok(id);
    };

    for (std::size_t index = start_position.value.ref_index;
         index <= complete_position.value.ref_index; ++index) {
      const DirectedSegmentRef& ref = corridor->segments[index];
      const RoadSegment* item = find_segment(graph_, ref.segment_id);
      const DerivedSegment* item_derived =
          FindDerivedSegment(derived_, ref.segment_id);
      if (item == nullptr || item_derived == nullptr) {
        return Result<LaneId>::Fail(CommitFailureCategory::kInternalError,
                                    "lane taper segment is missing");
      }
      if (item->transition.has_value()) {
        return Result<LaneId>::Fail(
            CommitFailureCategory::kInvalidInput,
            "lane taper crosses a road segment that already has a section transition");
      }
      const RoadLayoutTemplate* base =
          find_template(graph_, item->layout_template);
      if (base == nullptr) {
        return Result<LaneId>::Fail(CommitFailureCategory::kInternalError,
                                    "lane taper section is missing");
      }
      const LaneTravelDirection direction =
          ref.reversed
              ? (request.direction == LaneTravelDirection::kAlongSegment
                     ? LaneTravelDirection::kAgainstSegment
                     : LaneTravelDirection::kAlongSegment)
              : request.direction;
      const RoadSide side =
          ref.reversed
              ? (request.side == RoadSide::kLeft ? RoadSide::kRight
                                                 : RoadSide::kLeft)
              : request.side;
      const double segment_begin = corridor_starts[index];
      const double segment_end = segment_begin + item_derived->length_m;
      const double overlap_begin = std::max(
          segment_begin, start_position.value.corridor_distance_m);
      const double overlap_end = std::min(
          segment_end, complete_position.value.corridor_distance_m);
      const double begin_fraction =
          (overlap_begin - start_position.value.corridor_distance_m) /
          taper_length;
      const double end_fraction =
          (overlap_end - start_position.value.corridor_distance_m) /
          taper_length;
      const double corridor_begin_t =
          (overlap_begin - segment_begin) / item_derived->length_m;
      const double corridor_end_t =
          (overlap_end - segment_begin) / item_derived->length_m;
      double first_local_t = ref.reversed ? 1.0 - corridor_begin_t
                                          : corridor_begin_t;
      double second_local_t = ref.reversed ? 1.0 - corridor_end_t
                                           : corridor_end_t;
      if (index == start_position.value.ref_index)
        first_local_t = request.transition_start.t;
      if (index == complete_position.value.ref_index)
        second_local_t = request.transition_complete.t;
      if (split_terminal && index == end_position.value.ref_index) {
        const double split_t = request.continuation_end.t;
        const auto remap = [split_t, reversed = ref.reversed](double value) {
          return reversed ? (value - split_t) / (1.0 - split_t)
                          : value / split_t;
        };
        first_local_t = remap(first_local_t);
        second_local_t = remap(second_local_t);
      }
      const double local_start_t = std::min(first_local_t, second_local_t);
      const double local_end_t = std::max(first_local_t, second_local_t);
      const double local_from_fraction = ref.reversed ? end_fraction
                                                      : begin_fraction;
      const double local_to_fraction = ref.reversed ? begin_fraction
                                                    : end_fraction;
      const Result<RoadLayoutTemplateId> from_id = section_at_fraction(
          *base, direction, side, local_from_fraction);
      const Result<RoadLayoutTemplateId> to_id = section_at_fraction(
          *base, direction, side, local_to_fraction);
      if (!from_id.ok || !to_id.ok) {
        const auto& failed = !from_id.ok ? from_id : to_id;
        return Result<LaneId>::Fail(failed.failure_category, failed.error);
      }
      const Result<OuterLaneSelection> lane_selection = select_outer_lane_strip(
          *base, direction, side, CommitFailureCategory::kInvalidInput,
          "selected direction has no lane to extend",
          "lane transition source lane strip is missing");
      if (!lane_selection.ok) {
        return Result<LaneId>::Fail(lane_selection.failure_category,
                                    lane_selection.error);
      }
      const std::size_t strip_index = lane_selection.value.strip_index;
      const std::optional<std::size_t> fixed_index =
          side == RoadSide::kRight
              ? (strip_index > 0
                     ? std::optional<std::size_t>{strip_index - 1}
                     : std::nullopt)
              : (strip_index < base->boundaries.size()
                     ? std::optional<std::size_t>{strip_index}
                     : std::nullopt);
      const TransitionAnchor segment_anchor =
          fixed_index.has_value()
              ? TransitionAnchor::kBoundary
              : (side == RoadSide::kRight ? TransitionAnchor::kLeftEdge
                                          : TransitionAnchor::kRightEdge);
      const BoundaryId fixed_boundary =
          fixed_index.has_value() ? base->boundaries[*fixed_index].boundary_id
                                  : 0;
      const RoadLayoutTransitionId transition_id = next_id++;
      plan.add_transitions.push_back(RoadLayoutTransition{
          transition_id, from_id.value, to_id.value,
          DistanceRef{DistanceRefKind::kRatio, local_start_t},
          DistanceRef{DistanceRefKind::kRatio, local_end_t}, segment_anchor,
          fixed_boundary,
          {RoadLayoutTransitionRule{
              lane_ids.strip_id,
              ref.reversed ? TransitionAction::kTaperOut
                           : TransitionAction::kTaperIn}}});
      const RoadSegmentId lane_segment_id =
          split_terminal && index == end_position.value.ref_index
              ? terminal_segment_id
              : item->id;
      RoadSegment* replacement = editable_segment(lane_segment_id);
      if (replacement == nullptr) {
        return Result<LaneId>::Fail(CommitFailureCategory::kInternalError,
                                    "planned lane taper segment is missing");
      }
      replacement->layout_template = from_id.value;
      replacement->transition = transition_id;
    }

    RoadLayoutTemplateId terminal_template_id = target.id;
    for (auto ref = full_ref + 1; ref <= continuation_ref; ++ref) {
      const RoadSegment* following = find_segment(graph_, ref->segment_id);
      if (following == nullptr) {
        return Result<LaneId>::Fail(
            CommitFailureCategory::kInternalError,
            "lane addition following corridor segment is missing");
      }
      if (following->transition.has_value()) {
        return Result<LaneId>::Fail(
            CommitFailureCategory::kNotImplemented,
            "lane addition conflicts with an existing section transition");
      }
      const RoadLayoutTemplate* following_section =
          find_template(graph_, following->layout_template);
      if (following_section == nullptr) {
        return Result<LaneId>::Fail(
            CommitFailureCategory::kInternalError,
            "lane addition following corridor section is missing");
      }
      const LaneTravelDirection following_direction =
          ref->reversed
              ? (request.direction == LaneTravelDirection::kAlongSegment
                     ? LaneTravelDirection::kAgainstSegment
                     : LaneTravelDirection::kAlongSegment)
              : request.direction;
      const RoadSide following_side =
          ref->reversed
              ? (request.side == RoadSide::kLeft ? RoadSide::kRight
                                                 : RoadSide::kLeft)
              : request.side;
      const Result<RoadLayoutTemplateId> mapped =
          ensure_extended_lane_section(
              *following_section, following_direction, following_side,
              request.lane_width_m, lane_ids, section_extensions, plan,
              next_id);
      if (!mapped.ok) {
        return Result<LaneId>::Fail(mapped.failure_category, mapped.error);
      }
      const std::size_t index = static_cast<std::size_t>(
          std::distance(corridor->segments.begin(), ref));
      const RoadSegmentId lane_segment_id =
          split_terminal && index == end_position.value.ref_index
              ? terminal_segment_id
              : following->id;
      RoadSegment* following_replacement = editable_segment(lane_segment_id);
      if (following_replacement == nullptr) {
        return Result<LaneId>::Fail(
            CommitFailureCategory::kInternalError,
            "planned lane continuation segment is missing");
      }
      following_replacement->layout_template = mapped.value;
      terminal_template_id = mapped.value;
    }
    if (continuation_ref + 1 == corridor->segments.end()) {
      const auto planned_corridor = std::find_if(
          plan.replace_corridors.begin(), plan.replace_corridors.end(),
          [corridor](const RoadCorridor& item) { return item.id == corridor->id; });
      if (planned_corridor != plan.replace_corridors.end()) {
        planned_corridor->layout_template_id = terminal_template_id;
      } else {
        RoadCorridor corridor_replacement = *corridor;
        corridor_replacement.layout_template_id = terminal_template_id;
        plan.replace_corridors.push_back(std::move(corridor_replacement));
      }
    }
    plan.add_layout_templates.push_back(std::move(target));
    return execute_lane_plan();
  }
}
} // namespace city::road
