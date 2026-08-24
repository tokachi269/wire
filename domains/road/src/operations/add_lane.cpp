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
// derives the widened cross section, and writes the section transition. Junction
// movement topology is explicit saved data; the convenience operation does not
// infer a turn destination from approach order.
namespace city::road {
namespace {

using internal::distance;
using internal::find_segment;
using internal::find_template;
using internal::is_finite;
using internal::distance_epsilon;

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

[[nodiscard]] Result<bool> plan_same_identity_degree_two_topology(
    const SavedRoadGraph& graph, RoadSegmentId source_segment_id,
    EndpointRole source_role, LaneId terminating_lane_id,
    BoundaryId terminating_boundary_id, std::uint64_t& next_id,
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
                              "lane continuation source is missing");
  }
  const RoadNodeId node_id = source_role == EndpointRole::kStart
                                 ? source_segment->node_a
                                 : source_segment->node_b;
  const RoadSegment* target_segment = nullptr;
  EndpointRole target_role = EndpointRole::kStart;
  for (const RoadSegment& segment : graph.segments) {
    if (segment.id == source_segment_id) continue;
    if (segment.node_a == node_id || segment.node_b == node_id) {
      if (target_segment != nullptr) return Result<bool>::Ok(true);
      target_segment = &segment;
      target_role = endpoint_role_at(segment, node_id);
    }
  }
  if (target_segment == nullptr) return Result<bool>::Ok(true);
  const RoadLayoutTemplate* target_section =
      internal::find_endpoint_template(graph, *target_segment, target_role);
  if (target_section == nullptr) {
    return Result<bool>::Fail(CommitFailureCategory::kInternalError,
                              "lane continuation target is missing");
  }
  const auto source_lanes =
      ordered_lane_endpoints(*source_section, source_segment_id, source_role,
                             true);
  const auto target_lanes =
      ordered_lane_endpoints(*target_section, target_segment->id, target_role,
                             false);
  if (!source_lanes.ok || !target_lanes.ok) {
    const auto& failed = !source_lanes.ok ? source_lanes : target_lanes;
    return Result<bool>::Fail(failed.failure_category, failed.error);
  }
  const auto lane_endpoint_claimed = [&](const LaneEndpointKey& source,
                                         const LaneEndpointKey& destination) {
    const auto conflicts = [&](const LaneConnection& connection) {
      return connection.source == source || connection.target == destination;
    };
    return std::any_of(graph.lane_connections.begin(),
                       graph.lane_connections.end(), conflicts) ||
           std::any_of(plan.add_lane_connections.begin(),
                       plan.add_lane_connections.end(), conflicts);
  };
  for (const OrderedLaneEndpoint& source : source_lanes.value) {
    if (source.key.lane_id == terminating_lane_id) continue;
    const auto target = std::find_if(
        target_lanes.value.begin(), target_lanes.value.end(),
        [&source](const OrderedLaneEndpoint& candidate) {
          return candidate.key.lane_id == source.key.lane_id;
        });
    if (target == target_lanes.value.end()) continue;
    if (lane_endpoint_claimed(source.key, target->key)) continue;
    plan.add_lane_connections.push_back(
        LaneConnection{next_id++, source.key, target->key,
                       LaneConnectionKind::kContinuation});
  }

  const std::vector<OrderedBoundaryEndpoint> source_boundaries =
      ordered_boundary_endpoints(*source_section, source_segment_id,
                                 source_role);
  const std::vector<OrderedBoundaryEndpoint> target_boundaries =
      ordered_boundary_endpoints(*target_section, target_segment->id,
                                 target_role);
  const auto boundary_endpoint_claimed =
      [&](const BoundaryEndpointKey& source,
          const BoundaryEndpointKey& destination) {
    const auto conflicts = [&](const BoundaryContinuation& continuation) {
      return continuation.source == source ||
             continuation.target == destination;
    };
    return std::any_of(graph.boundary_continuations.begin(),
                       graph.boundary_continuations.end(), conflicts) ||
           std::any_of(plan.add_boundary_continuations.begin(),
                       plan.add_boundary_continuations.end(), conflicts);
  };
  for (const OrderedBoundaryEndpoint& source : source_boundaries) {
    if (source.key.boundary_id == terminating_boundary_id) continue;
    const auto target = std::find_if(
        target_boundaries.begin(), target_boundaries.end(),
        [&source](const OrderedBoundaryEndpoint& candidate) {
          return candidate.key.boundary_id == source.key.boundary_id;
        });
    if (target == target_boundaries.end()) continue;
    if (boundary_endpoint_claimed(source.key, target->key)) continue;
    plan.add_boundary_continuations.push_back(BoundaryContinuation{
        next_id++, source.key, target->key,
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
      !is_finite(request.transition_complete.t)) {
    return Result<bool>::Fail(CommitFailureCategory::kInvalidInput,
                              "lane transition positions must be finite");
  }
  if (request.transition_start.segment_id == 0 ||
      request.transition_complete.segment_id == 0 ||
      request.transition_start.t < 0.0 ||
      request.transition_start.t > 1.0 ||
      request.transition_complete.t < 0.0 ||
      request.transition_complete.t > 1.0) {
    return Result<bool>::Fail(
        CommitFailureCategory::kInvalidInput,
        "lane transition positions are invalid");
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
      "lane position is not on the selected corridor",
      "road_add_lane_position_not_on_selected_corridor");
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
  if (!start_position.ok || !complete_position.ok) {
    const auto& failed = !start_position.ok
                             ? start_position
                             : complete_position;
    return Result<LaneId>::Fail(failed.failure_category, failed.error,
                                failed.reason_code);
  }
  const double operation_delta_m =
      complete_position.value.corridor_distance_m -
      start_position.value.corridor_distance_m;
  if (std::abs(operation_delta_m) <= distance_epsilon) {
    return Result<LaneId>::Fail(
        CommitFailureCategory::kInvalidInput,
        "lane start and completion positions must be different",
        "road_add_lane_positions_identical");
  }
  const bool operation_forward = operation_delta_m > 0.0;
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
        "this road segment already has a section transition; overlapping lane changes are not supported",
        "road_add_lane_transition_conflict");
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
  std::vector<LaneSectionExtension> section_extensions{
      LaneSectionExtension{source->id, local_direction, local_side,
                           target.id}};
  const auto continuation_ref =
      operation_forward ? corridor->segments.end() - 1 : corridor->segments.begin();
  RoadSegmentId terminal_segment_id = continuation_ref->segment_id;
  EndpointRole terminal_role =
      operation_forward
          ? (continuation_ref->reversed ? EndpointRole::kStart
                                        : EndpointRole::kEnd)
          : (continuation_ref->reversed ? EndpointRole::kEnd
                                        : EndpointRole::kStart);

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
    const Result<bool> degree_two_topology =
        plan_same_identity_degree_two_topology(
            graph_, terminal_segment_id, terminal_role, added_lane_id,
            lane_ids.boundary_id, next_id, plan);
    if (!degree_two_topology.ok) {
      return Result<LaneId>::Fail(degree_two_topology.failure_category,
                                  degree_two_topology.error);
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
    const double taper_length = std::abs(operation_delta_m);
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

    const std::size_t taper_begin_index =
        std::min(start_position.value.ref_index, complete_position.value.ref_index);
    const std::size_t taper_end_index =
        std::max(start_position.value.ref_index, complete_position.value.ref_index);
    if (taper_begin_index != taper_end_index) {
      return Result<LaneId>::Fail(
          CommitFailureCategory::kNotImplemented,
          "lane taper must stay within one road segment",
          "road_add_lane_taper_crosses_segment_boundary");
    }
    for (std::size_t index = taper_begin_index;
         index <= taper_end_index; ++index) {
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
            "lane taper crosses a road segment that already has a section transition",
            "road_add_lane_transition_conflict");
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
      const double taper_corridor_begin = std::min(
          start_position.value.corridor_distance_m,
          complete_position.value.corridor_distance_m);
      const double taper_corridor_end = std::max(
          start_position.value.corridor_distance_m,
          complete_position.value.corridor_distance_m);
      const double overlap_begin = std::max(segment_begin, taper_corridor_begin);
      const double overlap_end = std::min(segment_end, taper_corridor_end);
      const auto fraction_at = [&](double corridor_distance_m) {
        return operation_forward
                   ? (corridor_distance_m -
                      start_position.value.corridor_distance_m) /
                         taper_length
                   : (start_position.value.corridor_distance_m -
                      corridor_distance_m) /
                         taper_length;
      };
      const double begin_fraction = fraction_at(overlap_begin);
      const double end_fraction = fraction_at(overlap_end);
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
              local_to_fraction + distance_epsilon >= local_from_fraction
                  ? TransitionAction::kTaperIn
                  : TransitionAction::kTaperOut}}});
      const RoadSegmentId lane_segment_id =
          item->id;
      RoadSegment* replacement = editable_segment(lane_segment_id);
      if (replacement == nullptr) {
        return Result<LaneId>::Fail(CommitFailureCategory::kInternalError,
                                    "planned lane taper segment is missing");
      }
      replacement->layout_template = from_id.value;
      replacement->transition = transition_id;
    }

    RoadLayoutTemplateId terminal_template_id = target.id;
    const bool has_full_segments =
        operation_forward
            ? complete_position.value.ref_index + 1 < corridor->segments.size()
            : complete_position.value.ref_index > 0;
    const std::size_t full_begin_index = operation_forward
                                             ? complete_position.value.ref_index + 1
                                             : 0;
    const std::size_t full_end_index = operation_forward
                                           ? corridor->segments.size() - 1
                                           : complete_position.value.ref_index - 1;
    if (has_full_segments) {
      for (std::size_t full_index = full_begin_index;
           full_index <= full_end_index; ++full_index) {
        const auto ref = corridor->segments.begin() +
                         static_cast<std::ptrdiff_t>(full_index);
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
            following->id;
        RoadSegment* following_replacement = editable_segment(lane_segment_id);
        if (following_replacement == nullptr) {
          return Result<LaneId>::Fail(
              CommitFailureCategory::kInternalError,
              "planned lane continuation segment is missing");
        }
        following_replacement->layout_template = mapped.value;
        terminal_template_id = mapped.value;
      }
    }
    if (operation_forward && continuation_ref + 1 == corridor->segments.end()) {
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
