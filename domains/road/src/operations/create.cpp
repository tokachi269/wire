// Drawing a road: one confirmed path becomes one RoadSegment, a corridor end
// extends by appending, a branch splits the segment it starts from, and a split
// moves the owners that sit past the cut.
#include "city/road/road.hpp"

#include "../geometry/geometry.hpp"
#include "../geometry/section.hpp"
#include "../lookup.hpp"
#include "operation_plan.hpp"
#include "../geometry/alignment.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>

namespace city::road {
namespace {

using internal::add;
using internal::align_first_span_start;
using internal::align_last_span_end;
using internal::distance;
using internal::find_node;
using internal::find_policy_override;
using internal::find_approach_override;
using internal::find_segment;
using internal::node_degree;
using internal::find_template;
using internal::find_transition;
using internal::is_finite;
using internal::distance_epsilon;
using internal::magnitude;
using internal::make_linear_shape;
using internal::manual_area_distance_bounds;
using internal::manual_line_distance_bounds;
using internal::path_end;
using internal::path_start;
using internal::PathSplit;
using internal::scale;
using internal::shift_manual_line_distance;
using internal::split_path_at_distance;
using internal::subtract;

constexpr double kSnapDistancePointToleranceM = 0.6;

double segment_elevation_at(const SavedRoadGraph& graph,
                            const RoadSegment& segment,
                            double segment_distance_m,
                            double segment_length_m) {
  const RoadNode* a = find_node(graph, segment.node_a);
  const RoadNode* b = find_node(graph, segment.node_b);
  if (a == nullptr || b == nullptr ||
      segment_length_m <= internal::distance_epsilon) {
    return a != nullptr ? a->elevation_m : 0.0;
  }
  const double t = std::clamp(segment_distance_m / segment_length_m, 0.0, 1.0);
  return a->elevation_m + (b->elevation_m - a->elevation_m) * t;
}

Result<bool> validate_corner_radius(double corner_radius_m) {
  if (!is_finite(corner_radius_m) || corner_radius_m < 0.0) {
    return Result<bool>::Fail(CommitFailureCategory::kInvalidInput,
                              "road corner radius must be finite and non-negative");
  }
  return Result<bool>::Ok(true);
}

std::optional<LaneEndpointKey> remap_split_lane_endpoint(
    LaneEndpointKey key, RoadSegmentId source_id, RoadSegmentId second_id) {
  if (key.segment_id != source_id) return key;
  if (key.endpoint_role == EndpointRole::kStart) return key;
  key.segment_id = second_id;
  return key;
}

std::optional<BoundaryEndpointKey> remap_split_boundary_endpoint(
    BoundaryEndpointKey key, RoadSegmentId source_id, RoadSegmentId second_id) {
  if (key.segment_id != source_id) return key;
  if (key.endpoint_role == EndpointRole::kStart) return key;
  key.segment_id = second_id;
  return key;
}

template <typename Id>
void append_once(std::vector<Id>& values, Id value) {
  if (std::find(values.begin(), values.end(), value) == values.end()) {
    values.push_back(value);
  }
}

void plan_split_endpoint_topology_remap(const SavedRoadGraph& graph,
                                        RoadSegmentId source_id,
                                        RoadSegmentId second_id,
                                        operations::OperationPlan& plan) {
  // A later split composes its endpoint substitution into the relation already
  // planned by an earlier split, while the saved row is removed only once.
  for (const LaneConnection& connection : graph.lane_connections) {
    const auto planned = std::find_if(
        plan.add_lane_connections.begin(), plan.add_lane_connections.end(),
        [&connection](const LaneConnection& candidate) {
          return candidate.id == connection.id;
        });
    const LaneConnection& current =
        planned == plan.add_lane_connections.end() ? connection : *planned;
    const std::optional<LaneEndpointKey> source =
        remap_split_lane_endpoint(current.source, source_id, second_id);
    const std::optional<LaneEndpointKey> target =
        remap_split_lane_endpoint(current.target, source_id, second_id);
    if (!source.has_value() || !target.has_value()) {
      append_once(plan.remove_lane_connections, connection.id);
      if (planned != plan.add_lane_connections.end()) {
        plan.add_lane_connections.erase(planned);
      }
      continue;
    }
    if (*source == current.source && *target == current.target) continue;
    LaneConnection mapped = current;
    mapped.source = *source;
    mapped.target = *target;
    append_once(plan.remove_lane_connections, connection.id);
    if (planned == plan.add_lane_connections.end()) {
      plan.add_lane_connections.push_back(std::move(mapped));
    } else {
      *planned = std::move(mapped);
    }
  }
  for (const BoundaryContinuation& continuation :
       graph.boundary_continuations) {
    const auto planned = std::find_if(
        plan.add_boundary_continuations.begin(),
        plan.add_boundary_continuations.end(),
        [&continuation](const BoundaryContinuation& candidate) {
          return candidate.id == continuation.id;
        });
    const BoundaryContinuation& current =
        planned == plan.add_boundary_continuations.end() ? continuation
                                                         : *planned;
    const std::optional<BoundaryEndpointKey> source =
        remap_split_boundary_endpoint(current.source, source_id,
                                      second_id);
    const std::optional<BoundaryEndpointKey> target =
        remap_split_boundary_endpoint(current.target, source_id,
                                      second_id);
    if (!source.has_value() || !target.has_value()) {
      append_once(plan.remove_boundary_continuations, continuation.id);
      if (planned != plan.add_boundary_continuations.end()) {
        plan.add_boundary_continuations.erase(planned);
      }
      continue;
    }
    if (*source == current.source && *target == current.target) {
      continue;
    }
    BoundaryContinuation mapped = current;
    mapped.source = *source;
    mapped.target = *target;
    append_once(plan.remove_boundary_continuations, continuation.id);
    if (planned == plan.add_boundary_continuations.end()) {
      plan.add_boundary_continuations.push_back(std::move(mapped));
    } else {
      *planned = std::move(mapped);
    }
  }
}

enum class SplitTransitionPolicy {
  kReject,
  kRemapSegmentLocalRatio,
};

struct PlannedSegmentSplit {
  RoadNodeId split_node_id = 0;
  RoadSegmentId second_segment_id = 0;
  Vec2d point{};
  double elevation_m = 0.0;
};

Result<PlannedSegmentSplit> plan_segment_split(
    const SavedRoadGraph& graph, const DerivedRoad& derived,
    RoadSegmentId source_id, double segment_distance_m,
    SplitTransitionPolicy transition_policy, operations::OperationPlan& plan,
    std::uint64_t& next_id) {
  const RoadSegment* source = find_segment(graph, source_id);
  if (source == nullptr || !is_finite(segment_distance_m)) {
    return Result<PlannedSegmentSplit>::Fail(
        CommitFailureCategory::kInvalidInput, "road split request is invalid");
  }
  if (source->transition.has_value() &&
      transition_policy == SplitTransitionPolicy::kReject) {
    return Result<PlannedSegmentSplit>::Fail(
        CommitFailureCategory::kNotImplemented,
        "splitting a transitioning road segment is unsupported");
  }
  const Path* source_path = FindCanonicalAlignment(derived, source->id);
  if (source_path == nullptr) {
    return Result<PlannedSegmentSplit>::Fail(
        CommitFailureCategory::kInternalError,
        "road split canonical alignment is missing");
  }
  const Result<PathSplit> split =
      split_path_at_distance(*source_path, segment_distance_m);
  if (!split.ok) {
    return Result<PlannedSegmentSplit>::Fail(split.failure_category,
                                             split.error);
  }
  const Result<double> source_length = PathLength(*source_path);
  if (!source_length.ok || source_length.value <= distance_epsilon) {
    return Result<PlannedSegmentSplit>::Fail(
        CommitFailureCategory::kInternalError,
        "road split source length is invalid");
  }

  std::optional<RoadLayoutTransition> remapped_transition{};
  bool transition_moves_to_second = false;
  if (source->transition.has_value()) {
    const RoadLayoutTransition* transition =
        find_transition(graph, *source->transition);
    if (transition == nullptr) {
      return Result<PlannedSegmentSplit>::Fail(
          CommitFailureCategory::kInternalError,
          "transitioning road split data is incomplete");
    }
    if (transition->start.kind != DistanceRefKind::kRatio ||
        transition->end.kind != DistanceRefKind::kRatio) {
      return Result<PlannedSegmentSplit>::Fail(
          CommitFailureCategory::kNotImplemented,
          "only segment-local ratio transitions can be split");
    }
    const double split_t = segment_distance_m / source_length.value;
    const double start_t = transition->start.value;
    const double complete_t = transition->end.value;
    remapped_transition = *transition;
    if (split_t < start_t - distance_epsilon) {
      transition_moves_to_second = true;
      remapped_transition->start.value =
          (start_t - split_t) / (1.0 - split_t);
      remapped_transition->end.value =
          (complete_t - split_t) / (1.0 - split_t);
    } else if (split_t > complete_t + distance_epsilon) {
      remapped_transition->start.value = start_t / split_t;
      remapped_transition->end.value = complete_t / split_t;
    } else {
      return Result<PlannedSegmentSplit>::Fail(
          CommitFailureCategory::kNotImplemented,
          "a road cannot be split inside its section transition; choose a position before or after the lane change");
    }
  }

  for (const ManualLineMarking& marking : graph.manual_lines) {
    if (marking.owner_segment_id != source->id) continue;
    const auto [minimum, maximum] = manual_line_distance_bounds(marking);
    if (minimum < segment_distance_m - distance_epsilon &&
        maximum > segment_distance_m + distance_epsilon) {
      return Result<PlannedSegmentSplit>::Fail(
          CommitFailureCategory::kNotImplemented,
          "road split crosses a manual line marking");
    }
  }
  for (const ManualAreaMarking& marking : graph.manual_areas) {
    if (marking.owner_segment_id != source->id) continue;
    const auto [minimum, maximum] = manual_area_distance_bounds(marking);
    if (minimum < segment_distance_m - distance_epsilon &&
        maximum > segment_distance_m + distance_epsilon) {
      return Result<PlannedSegmentSplit>::Fail(
          CommitFailureCategory::kNotImplemented,
          "road split crosses a manual area marking");
    }
  }

  const Result<SegmentShape> first_shape =
      SegmentShapeFromPath(split.value.before);
  const Result<SegmentShape> second_shape =
      SegmentShapeFromPath(split.value.after);
  if (!first_shape.ok || !second_shape.ok) {
    return Result<PlannedSegmentSplit>::Fail(
        CommitFailureCategory::kInternalError,
        "road split shape derivation failed");
  }
  const RoadCorridor* corridor = FindCorridorForSegment(graph, source->id);
  if (corridor == nullptr) {
    return Result<PlannedSegmentSplit>::Fail(
        CommitFailureCategory::kInternalError,
        "road split source corridor is missing");
  }

  const RoadNodeId split_node = next_id++;
  const RoadSegmentId second_id = next_id++;
  RoadSegment first = *source;
  first.node_b = split_node;
  first.shape = first_shape.value;
  if (remapped_transition.has_value() && transition_moves_to_second) {
    first.transition.reset();
  }
  plan.replace_segments.push_back(std::move(first));
  const double split_elevation =
      segment_elevation_at(graph, *source, segment_distance_m,
                           source_length.value);
  plan.add_nodes.push_back(
      RoadNode{split_node, split.value.point, split_elevation});
  RoadSegment second{second_id, split_node, source->node_b, second_shape.value,
                     source->layout_template, source->transition,
                     source->corner_radius_m};
  if (remapped_transition.has_value()) {
    if (transition_moves_to_second) {
      second.layout_template = remapped_transition->from_template;
    } else {
      second.layout_template = remapped_transition->to_template;
      second.transition.reset();
    }
    plan.remove_transitions.push_back(remapped_transition->id);
    plan.add_transitions.push_back(*remapped_transition);
  }
  plan.add_segments.push_back(std::move(second));

  auto planned_corridor = std::find_if(
      plan.replace_corridors.begin(), plan.replace_corridors.end(),
      [corridor](const RoadCorridor& candidate) {
        return candidate.id == corridor->id;
      });
  if (planned_corridor == plan.replace_corridors.end()) {
    plan.replace_corridors.push_back(*corridor);
    planned_corridor = std::prev(plan.replace_corridors.end());
  }
  const auto source_ref = std::find_if(
      planned_corridor->segments.begin(), planned_corridor->segments.end(),
      [source](const DirectedSegmentRef& ref) {
        return ref.segment_id == source->id;
      });
  if (source_ref == planned_corridor->segments.end()) {
    return Result<PlannedSegmentSplit>::Fail(
        CommitFailureCategory::kInternalError,
        "road split corridor reference is missing");
  }
  const std::size_t source_index = static_cast<std::size_t>(
      std::distance(planned_corridor->segments.begin(), source_ref));
  const bool reversed = source_ref->reversed;
  planned_corridor->segments.erase(
      planned_corridor->segments.begin() +
      static_cast<std::ptrdiff_t>(source_index));
  const std::array<DirectedSegmentRef, 2> replacements =
      reversed
          ? std::array<DirectedSegmentRef, 2>{
                DirectedSegmentRef{second_id, true},
                DirectedSegmentRef{source->id, true}}
          : std::array<DirectedSegmentRef, 2>{
                DirectedSegmentRef{source->id, false},
                DirectedSegmentRef{second_id, false}};
  planned_corridor->segments.insert(
      planned_corridor->segments.begin() +
          static_cast<std::ptrdiff_t>(source_index),
      replacements.begin(), replacements.end());

  const ApproachKey old_end_key{source->node_b, source->id,
                                EndpointRole::kEnd};
  if (const ApproachGeometryOverride* old_end =
          find_approach_override(graph, old_end_key)) {
    ApproachGeometryOverride mapped = *old_end;
    mapped.key =
        ApproachKey{source->node_b, second_id, EndpointRole::kEnd};
    plan.remove_approach_geometry_overrides.push_back(old_end_key);
    plan.add_approach_geometry_overrides.push_back(std::move(mapped));
  }
  for (const ManualLineMarking& marking : graph.manual_lines) {
    if (marking.owner_segment_id != source->id ||
        manual_line_distance_bounds(marking).first + distance_epsilon <
            segment_distance_m) {
      continue;
    }
    ManualLineMarking mapped = marking;
    mapped.owner_segment_id = second_id;
    shift_manual_line_distance(mapped, -segment_distance_m);
    plan.remove_manual_lines.push_back(marking.id);
    plan.add_manual_lines.push_back(std::move(mapped));
  }
  for (const ManualAreaMarking& marking : graph.manual_areas) {
    if (marking.owner_segment_id != source->id ||
        manual_area_distance_bounds(marking).first + distance_epsilon <
            segment_distance_m) {
      continue;
    }
    ManualAreaMarking mapped = marking;
    mapped.owner_segment_id = second_id;
    mapped.frame_origin.x -= segment_distance_m;
    plan.remove_manual_areas.push_back(marking.id);
    plan.add_manual_areas.push_back(std::move(mapped));
  }
  for (const AutoMarkingOverride& override : graph.auto_marking_overrides) {
    if (override.key.owner.kind != MarkingOwner::Kind::kRoadSegment ||
        override.key.owner.segment_id != source->id ||
        !override.key.track.has_value()) {
      continue;
    }
    AutoMarkingOverride mapped = override;
    mapped.key.owner.segment_id = second_id;
    mapped.key.track->segment_id = second_id;
    plan.add_auto_marking_overrides.push_back(std::move(mapped));
  }
  for (const JunctionMarkingOverride& override :
       graph.junction_marking_overrides) {
    // The same override may name both split sources, so continue from its
    // planned mapping instead of rebuilding the other approach from the graph.
    const auto planned = std::find_if(
        plan.add_junction_marking_overrides.begin(),
        plan.add_junction_marking_overrides.end(),
        [&override](const JunctionMarkingOverride& candidate) {
          return candidate.id == override.id;
        });
    const JunctionMarkingOverride& current =
        planned == plan.add_junction_marking_overrides.end() ? override
                                                             : *planned;
    const bool source_end = current.source.approach == old_end_key;
    const bool target_end = current.target.has_value() &&
                            current.target->approach == old_end_key;
    if (!source_end && !target_end) continue;
    JunctionMarkingOverride mapped = current;
    const ApproachKey mapped_key{source->node_b, second_id,
                                 EndpointRole::kEnd};
    if (source_end) mapped.source.approach = mapped_key;
    if (target_end) mapped.target->approach = mapped_key;
    append_once(plan.remove_junction_marking_overrides, override.id);
    if (planned == plan.add_junction_marking_overrides.end()) {
      plan.add_junction_marking_overrides.push_back(std::move(mapped));
    } else {
      *planned = std::move(mapped);
    }
  }
  plan_split_endpoint_topology_remap(graph, source->id, second_id, plan);
  return Result<PlannedSegmentSplit>::Ok(
      {split_node, second_id, split.value.point, split_elevation});
}

Result<RoadNodeId> plan_connection_target_split(
    const SavedRoadGraph& graph, const DerivedRoad& derived,
    const RoadConnectionTarget& target, Vec2d expected_point,
    operations::OperationPlan& plan, std::uint64_t& next_id) {
  const bool names_node = target.node_id != 0;
  const bool names_segment = target.segment_id != 0;
  if (names_node == names_segment || !is_finite(target.segment_distance_m)) {
    return Result<RoadNodeId>::Fail(
        CommitFailureCategory::kInvalidInput,
        "road connection target must name exactly one node or segment");
  }
  if (names_node) {
    const RoadNode* node = find_node(graph, target.node_id);
    if (node == nullptr ||
        distance(expected_point, node->position) > kSnapDistancePointToleranceM) {
      return Result<RoadNodeId>::Fail(
          CommitFailureCategory::kInvalidInput,
          "road input endpoint does not match its explicit target node");
    }
    return Result<RoadNodeId>::Ok(node->id);
  }

  const Result<PlannedSegmentSplit> split = plan_segment_split(
      graph, derived, target.segment_id, target.segment_distance_m,
      SplitTransitionPolicy::kReject, plan, next_id);
  if (!split.ok) {
    return Result<RoadNodeId>::Fail(split.failure_category, split.error);
  }
  if (distance(expected_point, split.value.point) >
      kSnapDistancePointToleranceM) {
    return Result<RoadNodeId>::Fail(
        CommitFailureCategory::kInvalidInput,
        "road input endpoint does not match its explicit target distance");
  }
  return Result<RoadNodeId>::Ok(split.value.split_node_id);
}

} // namespace

Result<RoadSegmentId> RoadState::AddSegment(AddSegmentRequest request) {
  Path alignment = std::move(request.alignment);
  const RoadLayoutTemplateId layout_template = request.layout_template;
  const Result<bool> radius_valid = validate_corner_radius(request.corner_radius_m);
  if (!radius_valid.ok) {
    return Result<RoadSegmentId>::Fail(radius_valid.failure_category,
                                       radius_valid.error);
  }
  const Result<bool> path_valid = ValidatePath(alignment);
  if (!path_valid.ok) {
    return Result<RoadSegmentId>::Fail(path_valid.failure_category,
                                       path_valid.error,
                                       path_valid.reason_code);
  }
  if (find_template(graph_, layout_template) == nullptr) {
    return Result<RoadSegmentId>::Fail(CommitFailureCategory::kInvalidInput, "road segment references a missing section template");
  }
  const double start_elevation =
      request.start_elevation_m.value_or(0.0);
  const double end_elevation = request.end_elevation_m.value_or(start_elevation);
  if (!is_finite(start_elevation) || !is_finite(end_elevation)) {
    return Result<RoadSegmentId>::Fail(
        CommitFailureCategory::kInvalidInput,
        "road segment endpoint elevation is non-finite");
  }
  operations::OperationPlan plan{};
  std::uint64_t next_id = next_id_;
  const RoadNodeId node_a = next_id++;
  const RoadNodeId node_b = next_id++;
  const RoadSegmentId segment_id = next_id++;
  const RoadCorridorId corridor_id = next_id++;
  plan.add_nodes = {RoadNode{node_a, alignment.spans.front().p0,
                             start_elevation},
                    RoadNode{node_b, alignment.spans.back().p3,
                             end_elevation}};
  Result<SegmentShape> shape = SegmentShapeFromPath(alignment);
  if (!shape.ok) {
    return Result<RoadSegmentId>::Fail(shape.failure_category, shape.error);
  }
  if (request.intent.has_value()) {
    shape.value.intent = *request.intent;
  }
  plan.add_segments.push_back(RoadSegment{
      segment_id, node_a, node_b, shape.value, layout_template,
      std::nullopt, request.corner_radius_m});
  plan.add_corridors.push_back(RoadCorridor{
      corridor_id, layout_template,
      {DirectedSegmentRef{segment_id, false}}});
  plan.next_id_after = next_id;
  const Result<bool> executed = Execute(plan);
  if (!executed.ok) return Result<RoadSegmentId>::Fail(executed.failure_category, executed.error);
  return Result<RoadSegmentId>::Ok(segment_id);
}

Result<RoadSegmentId> RoadState::ExtendCorridorFromEnd(
    ExtendCorridorFromEndRequest request) {
  const RoadCorridor* source_corridor =
      FindRoadCorridor(graph_, request.corridor_id);
  const RoadNode* endpoint = find_node(graph_, request.endpoint_node_id);
  if (source_corridor == nullptr || source_corridor->segments.empty() ||
      endpoint == nullptr) {
    return Result<RoadSegmentId>::Fail(
        CommitFailureCategory::kInvalidInput,
        "road extension corridor or endpoint does not exist");
  }
  const DirectedSegmentRef last_ref = source_corridor->segments.back();
  const RoadSegment* source = find_segment(graph_, last_ref.segment_id);
  if (source == nullptr) {
    return Result<RoadSegmentId>::Fail(
        CommitFailureCategory::kInternalError, "road corridor terminal segment is missing");
  }
  const RoadNodeId corridor_end =
      last_ref.reversed ? source->node_a : source->node_b;
  if (corridor_end != endpoint->id) {
    return Result<RoadSegmentId>::Fail(
        CommitFailureCategory::kInvalidInput,
        "road extension node is not the corridor end");
  }
  if (node_degree(graph_, endpoint->id) != 1) {
    return Result<RoadSegmentId>::Fail(
        CommitFailureCategory::kNotImplemented,
        "road extension requires a degree-one endpoint");
  }
  const EndpointRole source_endpoint_role =
      last_ref.reversed ? EndpointRole::kStart : EndpointRole::kEnd;
  const RoadLayoutTemplate* source_endpoint_section =
      internal::find_endpoint_template(graph_, *source, source_endpoint_role);
  const RoadLayoutTemplate* extension_section =
      find_template(graph_, request.layout_template);
  if (source_endpoint_section == nullptr || extension_section == nullptr) {
    return Result<RoadSegmentId>::Fail(
        CommitFailureCategory::kInvalidInput,
        "road extension endpoint section does not exist");
  }
  if (find_policy_override(graph_, endpoint->id) != nullptr) {
    return Result<RoadSegmentId>::Fail(
        CommitFailureCategory::kNotImplemented,
        "road extension endpoint has an explicit connection policy");
  }
  Path extension = std::move(request.extension);
  const Result<bool> extension_valid = ValidatePath(extension);
  if (!extension_valid.ok) {
    return Result<RoadSegmentId>::Fail(extension_valid.failure_category,
                                       extension_valid.error);
  }
  align_first_span_start(extension, endpoint->position);

  if (extension.spans.size() != 1) {
    return Result<RoadSegmentId>::Fail(
        CommitFailureCategory::kNotImplemented,
        "road corridor extension requires one local span");
  }
  Result<SegmentShape> shape = SegmentShapeFromPath(extension);
  if (!shape.ok) {
    return Result<RoadSegmentId>::Fail(shape.failure_category, shape.error);
  }
  if (request.intent.has_value()) {
    shape.value.intent = *request.intent;
  }

  operations::OperationPlan plan{};
  std::uint64_t next_id = next_id_;
  const RoadNodeId end_node = next_id++;
  const RoadSegmentId segment_id = next_id++;
  const double end_elevation =
      request.end_elevation_m.value_or(endpoint->elevation_m);
  if (!is_finite(end_elevation)) {
    return Result<RoadSegmentId>::Fail(
        CommitFailureCategory::kInvalidInput,
        "road extension endpoint elevation is non-finite");
  }

  plan.add_nodes.push_back(RoadNode{end_node, path_end(extension),
                                    end_elevation});
  plan.add_segments.push_back(
      RoadSegment{segment_id, endpoint->id, end_node, shape.value,
                  request.layout_template, std::nullopt,
                  source->corner_radius_m});
  RoadCorridor replacement = *source_corridor;
  replacement.layout_template_id = request.layout_template;
  replacement.segments.push_back(DirectedSegmentRef{segment_id, false});
  plan.replace_corridors.push_back(std::move(replacement));
  plan.next_id_after = next_id;
  const Result<bool> executed = Execute(plan);
  if (!executed.ok) {
    return Result<RoadSegmentId>::Fail(executed.failure_category, executed.error);
  }
  return Result<RoadSegmentId>::Ok(segment_id);
}

Result<RoadSegmentId> RoadState::AddSegmentConnectedTo(AddSegmentConnectedToRequest request) {
  Path alignment = std::move(request.alignment);
  const RoadLayoutTemplateId layout_template = request.layout_template;
  const RoadNodeId connected_node = request.start_node;
  const Result<bool> radius_valid = validate_corner_radius(request.corner_radius_m);
  if (!radius_valid.ok) {
    return Result<RoadSegmentId>::Fail(radius_valid.failure_category,
                                       radius_valid.error);
  }
  const RoadNode* node = find_node(graph_, connected_node);
  if (node == nullptr) {
    return Result<RoadSegmentId>::Fail(CommitFailureCategory::kInvalidInput, "road segment start node does not exist");
  }
  if (find_template(graph_, layout_template) == nullptr) {
    return Result<RoadSegmentId>::Fail(CommitFailureCategory::kInvalidInput,
                                       "connected road references a missing section template");
  }
  const Result<double> length_result = PathLength(alignment);
  if (!length_result.ok) {
    return Result<RoadSegmentId>::Fail(length_result.failure_category, length_result.error);
  }
  if (alignment.spans.size() != 1) {
    return Result<RoadSegmentId>::Fail(
        CommitFailureCategory::kNotImplemented,
        "road branch creation requires one local span");
  }
  if (request.connected_endpoint == EndpointRole::kStart) {
    align_first_span_start(alignment, node->position);
  } else {
    align_last_span_end(alignment, node->position);
  }
  const Result<SegmentShape> shape = SegmentShapeFromPath(alignment);
  if (!shape.ok) return Result<RoadSegmentId>::Fail(shape.failure_category, shape.error);
  operations::OperationPlan plan{};
  std::uint64_t next_id = next_id_;
  const RoadNodeId free_node = next_id++;
  const RoadSegmentId segment_id = next_id++;
  const RoadCorridorId corridor_id = next_id++;
  const bool connects_at_start =
      request.connected_endpoint == EndpointRole::kStart;
  const double free_elevation =
      request.free_endpoint_elevation_m.value_or(node->elevation_m);
  if (!is_finite(free_elevation)) {
    return Result<RoadSegmentId>::Fail(
        CommitFailureCategory::kInvalidInput,
        "connected road free endpoint elevation is non-finite");
  }
  plan.add_nodes.push_back(RoadNode{
      free_node, connects_at_start ? path_end(alignment) : path_start(alignment),
      free_elevation});
  plan.add_segments.push_back(RoadSegment{
      segment_id,
      connects_at_start ? connected_node : free_node,
      connects_at_start ? free_node : connected_node,
      shape.value, layout_template, std::nullopt, request.corner_radius_m});
  plan.add_corridors.push_back(
      RoadCorridor{corridor_id, layout_template,
                   {DirectedSegmentRef{segment_id, false}}});
  plan.next_id_after = next_id;
  const Result<bool> executed = Execute(plan);
  if (!executed.ok) return Result<RoadSegmentId>::Fail(executed.failure_category, executed.error);
  return Result<RoadSegmentId>::Ok(segment_id);
}

Result<RoadSegmentId> RoadState::AddSegmentConnectedToSegment(AddSegmentConnectedToSegmentRequest request) {
  Path alignment = std::move(request.alignment);
  const RoadLayoutTemplateId layout_template = request.layout_template;
  const Result<bool> radius_valid = validate_corner_radius(request.corner_radius_m);
  if (!radius_valid.ok) {
    return Result<RoadSegmentId>::Fail(radius_valid.failure_category,
                                       radius_valid.error);
  }
  const Result<bool> alignment_valid = ValidatePath(alignment);
  if (!alignment_valid.ok) {
    return Result<RoadSegmentId>::Fail(alignment_valid.failure_category, alignment_valid.error);
  }
  if (alignment.spans.size() != 1) {
    return Result<RoadSegmentId>::Fail(
        CommitFailureCategory::kNotImplemented,
        "road branch creation requires one local span");
  }
  const bool connects_at_start =
      request.connected_endpoint == EndpointRole::kStart;
  const Vec2d connected_point =
      connects_at_start ? path_start(alignment) : path_end(alignment);
  operations::OperationPlan plan{};
  std::uint64_t next_id = next_id_;
  const Result<PlannedSegmentSplit> split = plan_segment_split(
      graph_, derived_, request.start_segment, request.segment_distance_m,
      SplitTransitionPolicy::kReject, plan, next_id);
  if (!split.ok) {
    return Result<RoadSegmentId>::Fail(split.failure_category, split.error);
  }
  if (distance(connected_point, split.value.point) >
      kSnapDistancePointToleranceM) {
    return Result<RoadSegmentId>::Fail(CommitFailureCategory::kInvalidInput,
                                       "road segment input endpoint does not match its explicit snap distance");
  }
  if (connects_at_start) {
    align_first_span_start(alignment, split.value.point);
  } else {
    align_last_span_end(alignment, split.value.point);
  }
  const Result<double> branch_length = PathLength(alignment);
  if (!branch_length.ok) return Result<RoadSegmentId>::Fail(branch_length.failure_category, branch_length.error);
  const double branch_free_elevation =
      request.free_endpoint_elevation_m.value_or(split.value.elevation_m);
  if (!is_finite(branch_free_elevation)) {
    return Result<RoadSegmentId>::Fail(
        CommitFailureCategory::kInvalidInput,
        "road branch free endpoint elevation is non-finite");
  }
  const RoadNodeId branch_free_node = next_id++;
  const RoadSegmentId branch_id = next_id++;
  const RoadCorridorId branch_corridor_id = next_id++;
  const Result<SegmentShape> branch_shape = SegmentShapeFromPath(alignment);
  if (!branch_shape.ok) {
    return Result<RoadSegmentId>::Fail(branch_shape.failure_category,
                                       branch_shape.error);
  }
  plan.add_nodes.push_back(RoadNode{
      branch_free_node,
      connects_at_start ? path_end(alignment) : path_start(alignment),
      branch_free_elevation});
  plan.add_segments.push_back(RoadSegment{
      branch_id,
      connects_at_start ? split.value.split_node_id : branch_free_node,
      connects_at_start ? branch_free_node : split.value.split_node_id,
      branch_shape.value, layout_template, std::nullopt,
      request.corner_radius_m});
  plan.add_corridors.push_back(
      RoadCorridor{branch_corridor_id, layout_template,
                   {DirectedSegmentRef{branch_id, false}}});
  plan.next_id_after = next_id;
  const Result<bool> executed = Execute(plan);
  if (!executed.ok) return Result<RoadSegmentId>::Fail(executed.failure_category, executed.error);
  return Result<RoadSegmentId>::Ok(branch_id);
}

Result<RoadSegmentId> RoadState::AddSegmentBetween(
    AddSegmentBetweenRequest request) {
  Path alignment = std::move(request.alignment);
  const Result<bool> radius_valid = validate_corner_radius(request.corner_radius_m);
  if (!radius_valid.ok) {
    return Result<RoadSegmentId>::Fail(radius_valid.failure_category,
                                       radius_valid.error);
  }
  const Result<bool> path_valid = ValidatePath(alignment);
  if (!path_valid.ok) {
    return Result<RoadSegmentId>::Fail(path_valid.failure_category,
                                       path_valid.error);
  }
  if (alignment.spans.size() != 1) {
    return Result<RoadSegmentId>::Fail(
        CommitFailureCategory::kNotImplemented,
        "two-ended road connection requires one local span");
  }
  if (find_template(graph_, request.layout_template) == nullptr) {
    return Result<RoadSegmentId>::Fail(
        CommitFailureCategory::kInvalidInput,
        "two-ended road references a missing section template");
  }
  if (request.start.segment_id != 0 &&
      request.start.segment_id == request.end.segment_id) {
    return Result<RoadSegmentId>::Fail(
        CommitFailureCategory::kNotImplemented,
        "connecting two positions on the same road segment is unsupported");
  }

  operations::OperationPlan plan{};
  std::uint64_t next_id = next_id_;
  const Result<RoadNodeId> start = plan_connection_target_split(
      graph_, derived_, request.start, path_start(alignment), plan, next_id);
  if (!start.ok) {
    return Result<RoadSegmentId>::Fail(start.failure_category, start.error);
  }
  const Result<RoadNodeId> end = plan_connection_target_split(
      graph_, derived_, request.end, path_end(alignment), plan, next_id);
  if (!end.ok) {
    return Result<RoadSegmentId>::Fail(end.failure_category, end.error);
  }
  if (start.value == end.value) {
    return Result<RoadSegmentId>::Fail(
        CommitFailureCategory::kInvalidInput,
        "two-ended road connection targets the same node");
  }
  const RoadNode* start_node = find_node(graph_, start.value);
  const RoadNode* end_node = find_node(graph_, end.value);
  if (start_node != nullptr) {
    align_first_span_start(alignment, start_node->position);
  } else {
    const auto planned = std::find_if(
        plan.add_nodes.begin(), plan.add_nodes.end(),
        [&start](const RoadNode& node) { return node.id == start.value; });
    if (planned == plan.add_nodes.end()) {
      return Result<RoadSegmentId>::Fail(
          CommitFailureCategory::kInternalError,
          "planned start connection node is missing");
    }
    align_first_span_start(alignment, planned->position);
  }
  if (end_node != nullptr) {
    align_last_span_end(alignment, end_node->position);
  } else {
    const auto planned = std::find_if(
        plan.add_nodes.begin(), plan.add_nodes.end(),
        [&end](const RoadNode& node) { return node.id == end.value; });
    if (planned == plan.add_nodes.end()) {
      return Result<RoadSegmentId>::Fail(
          CommitFailureCategory::kInternalError,
          "planned end connection node is missing");
    }
    align_last_span_end(alignment, planned->position);
  }
  const Result<SegmentShape> shape = SegmentShapeFromPath(alignment);
  if (!shape.ok) {
    return Result<RoadSegmentId>::Fail(shape.failure_category, shape.error);
  }
  const RoadSegmentId segment_id = next_id++;
  const RoadCorridorId corridor_id = next_id++;
  plan.add_segments.push_back(RoadSegment{
      segment_id, start.value, end.value, shape.value,
      request.layout_template, std::nullopt, request.corner_radius_m});
  plan.add_corridors.push_back(RoadCorridor{
      corridor_id, request.layout_template,
      {DirectedSegmentRef{segment_id, false}}});
  plan.next_id_after = next_id;
  const Result<bool> executed = Execute(plan);
  if (!executed.ok) {
    return Result<RoadSegmentId>::Fail(executed.failure_category,
                                       executed.error);
  }
  return Result<RoadSegmentId>::Ok(segment_id);
}

Result<RoadSegmentId> RoadState::SplitSegmentAtDistance(
    SplitSegmentAtDistanceRequest request) {
  operations::OperationPlan plan{};
  std::uint64_t next_id = next_id_;
  const Result<PlannedSegmentSplit> split = plan_segment_split(
      graph_, derived_, request.segment_id, request.segment_distance_m,
      SplitTransitionPolicy::kRemapSegmentLocalRatio, plan, next_id);
  if (!split.ok) {
    return Result<RoadSegmentId>::Fail(split.failure_category, split.error);
  }
  plan.next_id_after = next_id;
  const Result<bool> executed = Execute(plan);
  if (!executed.ok) {
    return Result<RoadSegmentId>::Fail(executed.failure_category, executed.error);
  }
  return Result<RoadSegmentId>::Ok(split.value.second_segment_id);
}
} // namespace city::road
