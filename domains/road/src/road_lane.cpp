#include "city/road/road.hpp"

#include "geometry/geometry.hpp"
#include "geometry/section.hpp"
#include "lookup.hpp"
#include "operations/operation_plan.hpp"
#include "road_path.hpp"

#include <algorithm>
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
using internal::kEpsilon;

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
    const CrossSectionTemplate& section, RoadSegmentId segment_id,
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
    const CrossSectionTemplate& section, RoadSegmentId segment_id,
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
    EndpointRole source_role, LaneId added_lane_id, std::uint64_t& next_id,
    operations::OperationPlan& plan) {
  const RoadSegment* source_segment =
      internal::find_segment(graph, source_segment_id);
  const CrossSectionTemplate* source_section =
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
  if (incident.size() < 3) return Result<bool>::Ok(true);

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
  const std::vector<OrderedBoundaryEndpoint> source_boundaries =
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
    const CrossSectionTemplate* section = internal::find_endpoint_template(
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
    if (lanes.value.size() != source_lanes.value.size() ||
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
  for (std::size_t index = 0; index < source_lanes.value.size(); ++index) {
    const LaneEndpointKey& source = source_lanes.value[index].key;
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
        LaneConnectionKind::kJunctionMovement});
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
    const CrossSectionTemplate& section, LaneTravelDirection direction,
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
        [&lane](const SectionStrip& item) {
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
  SectionStripId strip_id = 0;
  LaneId lane_id = 0;
  BoundaryId boundary_id = 0;
};

[[nodiscard]] std::uint64_t next_section_member_id(
    const SavedRoadGraph& graph) {
  std::uint64_t next_id = 1;
  for (const CrossSectionTemplate& section : graph.section_templates) {
    for (const SectionStrip& strip : section.strips)
      next_id = std::max(next_id, strip.id + 1);
    for (const LaneBand& lane : section.lane_bands)
      next_id = std::max(next_id, lane.id + 1);
    for (const BoundaryProfile& boundary : section.boundaries)
      next_id = std::max(next_id, boundary.boundary_id + 1);
  }
  return next_id;
}

[[nodiscard]] Result<CrossSectionTemplate> make_extended_lane_section(
    const CrossSectionTemplate& base, LaneTravelDirection direction,
    RoadSide side, double lane_width_m, LaneSectionIds ids,
    CommitFailureCategory empty_category, const char* empty_error,
    const char* missing_strip_error) {
  const Result<OuterLaneSelection> selected = select_outer_lane_strip(
      base, direction, side, empty_category, empty_error,
      missing_strip_error);
  if (!selected.ok) {
    return Result<CrossSectionTemplate>::Fail(selected.failure_category,
                                               selected.error);
  }

  CrossSectionTemplate extended = base;
  SectionStrip strip = base.strips[selected.value.strip_index];
  strip.id = ids.strip_id;
  strip.width_m = lane_width_m;
  strip.side_marking = {};
  const BoundaryProfile divider{
      ids.boundary_id, BoundaryRole::kLaneDivider, 0.0, 0.0,
      AutoMarkingPolicy{true, MarkingRole::kLaneSeparator,
                        builtin_marking_styles::kWhiteDashed}};
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
  }
  extended.lane_bands.push_back(
      LaneBand{ids.lane_id, ids.strip_id, 0.0, lane_width_m, direction});
  return Result<CrossSectionTemplate>::Ok(std::move(extended));
}

struct LaneSectionExtension {
  CrossSectionTemplateId source_id = 0;
  LaneTravelDirection direction = LaneTravelDirection::kAlongSegment;
  RoadSide side = RoadSide::kRight;
  CrossSectionTemplateId target_id = 0;
};

[[nodiscard]] Result<CrossSectionTemplateId> ensure_extended_lane_section(
    const CrossSectionTemplate& base, LaneTravelDirection direction,
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
    return Result<CrossSectionTemplateId>::Ok(existing->target_id);
  }
  Result<CrossSectionTemplate> extended = make_extended_lane_section(
      base, direction, side, lane_width_m, ids,
      CommitFailureCategory::kNotImplemented,
      "later section has no lane in the selected direction",
      "lane extension source strip is missing");
  if (!extended.ok) {
    return Result<CrossSectionTemplateId>::Fail(extended.failure_category,
                                                extended.error);
  }
  extended.value.id = next_id++;
  const CrossSectionTemplateId id = extended.value.id;
  plan.add_section_templates.push_back(std::move(extended.value));
  extensions.push_back(LaneSectionExtension{base.id, direction, side, id});
  return Result<CrossSectionTemplateId>::Ok(id);
}

[[nodiscard]] Result<bool> validate_add_lane_request(
    const AddLaneRequest& request) {
  if (!is_finite(request.transition_start.t) ||
      !is_finite(request.transition_complete.t)) {
    return Result<bool>::Fail(CommitFailureCategory::kInvalidInput,
                              "lane transition positions must be finite");
  }
  if (request.transition_start.segment_id == 0 ||
      request.transition_complete.segment_id == 0 ||
      request.continuation_end_node_id == 0 ||
      request.transition_start.t < 0.0 ||
      request.transition_start.t > 1.0 ||
      request.transition_complete.t < 0.0 ||
      request.transition_complete.t > 1.0) {
    return Result<bool>::Fail(
        CommitFailureCategory::kInvalidInput,
        "lane transition positions or continuation endpoint are invalid");
  }
  if (request.transition_start.segment_id !=
      request.transition_complete.segment_id) {
    return Result<bool>::Fail(
        CommitFailureCategory::kNotImplemented,
        "lane transition start and completion must use the same road segment");
  }
  if (!is_finite(request.lane_width_m) || request.lane_width_m <= 0.0) {
    return Result<bool>::Fail(CommitFailureCategory::kInvalidInput,
                              "lane width must be finite and positive");
  }
  return Result<bool>::Ok(true);
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
  const RoadSegment* segment =
      find_segment(graph_, request.transition_start.segment_id);
  if (segment == nullptr) {
    return Result<LaneId>::Fail(CommitFailureCategory::kInternalError,
                                "lane transition segment is missing");
  }
  const auto selected_ref = std::find_if(
      corridor->segments.begin(), corridor->segments.end(),
      [segment](const DirectedSegmentRef& ref) {
        return ref.segment_id == segment->id;
      });
  if (selected_ref == corridor->segments.end()) {
    return Result<LaneId>::Fail(CommitFailureCategory::kInternalError,
                                "lane transition segment is not in its corridor");
  }
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
  const CrossSectionTemplate* source =
      find_template(graph_, segment->section_template);
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
  Result<CrossSectionTemplate> target_result = make_extended_lane_section(
      *source, local_direction, local_side, request.lane_width_m, lane_ids,
      CommitFailureCategory::kInvalidInput,
      "selected direction has no lane to extend",
      "lane transition source lane strip is missing");
  if (!target_result.ok) {
    return Result<LaneId>::Fail(target_result.failure_category,
                                target_result.error);
  }
  CrossSectionTemplate target = std::move(target_result.value);
  const LaneId added_lane_id = lane_ids.lane_id;

  operations::OperationPlan plan{};
  std::uint64_t next_id = next_id_;
  target.id = next_id++;
  const auto full_ref = std::find_if(
      selected_ref, corridor->segments.end(),
      [&request](const DirectedSegmentRef& ref) {
        return ref.segment_id == request.transition_complete.segment_id;
      });
  if (full_ref == corridor->segments.end()) {
    return Result<LaneId>::Fail(
        CommitFailureCategory::kInternalError,
        "lane transition completion segment is not in its corridor");
  }
  std::vector<LaneSectionExtension> section_extensions{
      LaneSectionExtension{source->id, local_direction, local_side,
                           target.id}};
  const auto continuation_ref = std::find_if(
      full_ref, corridor->segments.end(), [&](const DirectedSegmentRef& ref) {
        const RoadSegment* item = find_segment(graph_, ref.segment_id);
        if (item == nullptr) return false;
        const RoadNodeId exit_node = ref.reversed ? item->node_a : item->node_b;
        return exit_node == request.continuation_end_node_id;
      });
  if (continuation_ref == corridor->segments.end()) {
    return Result<LaneId>::Fail(
        CommitFailureCategory::kInvalidInput,
        "lane continuation endpoint is not reachable on the corridor");
  }
  const RoadSegmentId terminal_segment_id = continuation_ref->segment_id;
  const EndpointRole terminal_role = continuation_ref->reversed
                                         ? EndpointRole::kStart
                                         : EndpointRole::kEnd;
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
        next_id, plan);
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
    if (selected_ref != full_ref) {
      return Result<LaneId>::Fail(
          CommitFailureCategory::kNotImplemented,
          "lane transition start and completion must use the same road segment");
    }
    const DerivedSegment* selected_derived =
        FindDerivedSegment(derived_, segment->id);
    if (selected_derived == nullptr || selected_derived->length_m <= kEpsilon) {
      return Result<LaneId>::Fail(
          CommitFailureCategory::kInternalError,
          "lane transition segment geometry is missing");
    }
    const double first_t = request.transition_start.t;
    const double second_t = request.transition_complete.t;
    const double transition_start_t = std::min(first_t, second_t);
    const double transition_complete_t = std::max(first_t, second_t);
    if (transition_start_t < 0.0 || transition_complete_t > 1.0 ||
        transition_complete_t - transition_start_t <= kEpsilon) {
      return Result<LaneId>::Fail(CommitFailureCategory::kInvalidInput,
                                  "lane transition positions are invalid");
    }

    const bool reversed = selected_ref->reversed;
    const bool positions_follow_corridor = reversed
                                               ? first_t > second_t
                                               : first_t < second_t;
    if (!positions_follow_corridor) {
      return Result<LaneId>::Fail(
          CommitFailureCategory::kInvalidInput,
          "lane transition completion must follow its start along the corridor");
    }
    const CrossSectionTemplateId local_from_template_id =
        reversed ? target.id : source->id;
    const CrossSectionTemplateId local_to_template_id =
        reversed ? source->id : target.id;
    const TransitionAction action =
        reversed ? TransitionAction::kTaperOut : TransitionAction::kTaperIn;
    const SectionTransitionId transition_id = next_id++;
    plan.add_transitions.push_back(SectionTransition{
        transition_id, local_from_template_id, local_to_template_id,
        DistanceRef{DistanceRefKind::kRatio, transition_start_t},
        DistanceRef{DistanceRefKind::kRatio, transition_complete_t}, anchor,
        anchor_boundary_id,
        {SectionTransitionRule{lane_ids.strip_id, action}}});
    RoadSegment replacement = *segment;
    replacement.section_template = local_from_template_id;
    replacement.transition = transition_id;
    plan.replace_segments.push_back(std::move(replacement));

    CrossSectionTemplateId terminal_template_id = target.id;
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
      const CrossSectionTemplate* following_section =
          find_template(graph_, following->section_template);
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
      const Result<CrossSectionTemplateId> mapped =
          ensure_extended_lane_section(
              *following_section, following_direction, following_side,
              request.lane_width_m, lane_ids, section_extensions, plan,
              next_id);
      if (!mapped.ok) {
        return Result<LaneId>::Fail(mapped.failure_category, mapped.error);
      }
      RoadSegment following_replacement = *following;
      following_replacement.section_template = mapped.value;
      plan.replace_segments.push_back(std::move(following_replacement));
      terminal_template_id = mapped.value;
    }
    if (continuation_ref + 1 == corridor->segments.end()) {
      RoadCorridor corridor_replacement = *corridor;
      corridor_replacement.section_template_id = terminal_template_id;
      plan.replace_corridors.push_back(std::move(corridor_replacement));
    }
    plan.add_section_templates.push_back(std::move(target));
    return execute_lane_plan();
  }
}
} // namespace city::road
