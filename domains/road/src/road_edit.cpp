// Editing a road that already exists: its shape, where its nodes sit, and
// removing one segment together with the state only it owned.
#include "city/road/road.hpp"

#include "geometry/geometry.hpp"
#include "geometry/section.hpp"
#include "lookup.hpp"
#include "operations/operation_plan.hpp"
#include "road_path.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>

namespace city::road {
namespace {

using internal::add;
using internal::align_first_span_start;
using internal::align_last_span_end;
using internal::apply_inherited_arc;
using internal::corridor_terminal_handle;
using internal::distance;
using internal::find_node;
using internal::find_policy_override;
using internal::find_approach_override;
using internal::find_segment;
using internal::node_degree;
using internal::find_template;
using internal::find_transition;
using internal::is_finite;
using internal::kEpsilon;
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

} // namespace

Result<bool> RoadState::EditSegmentShape(EditSegmentShapeRequest request) {
  const RoadSegmentId segment_id = request.segment_id;
  SegmentShape shape = std::move(request.shape);
  const RoadSegment* segment = find_segment(graph_, segment_id);
  if (segment == nullptr) {
    return Result<bool>::Fail(CommitFailureCategory::kInvalidInput, "road segment does not exist");
  }
  const RoadNode* node_a = find_node(graph_, segment->node_a);
  const RoadNode* node_b = find_node(graph_, segment->node_b);
  const Result<Path> alignment = DeriveCanonicalAlignment(node_a->position, node_b->position, shape);
  if (!alignment.ok) return Result<bool>::Fail(alignment.failure_category, alignment.error);
  operations::OperationPlan plan{};
  plan.next_id_after = next_id_;
  RoadSegment replacement = *segment;
  replacement.shape = std::move(shape);
  plan.replace_segments.push_back(std::move(replacement));
  return Execute(plan);
}

Result<bool> RoadState::MoveNode(MoveNodeRequest request) {
  const RoadNodeId node_id = request.node_id;
  const Vec2d position = request.position;
  if (!is_finite(position)) return Result<bool>::Fail(CommitFailureCategory::kInvalidInput, "road node position is non-finite");
  const RoadNode* node = find_node(graph_, node_id);
  if (node == nullptr) return Result<bool>::Fail(CommitFailureCategory::kInvalidInput, "road node does not exist");
  operations::OperationPlan plan{};
  plan.next_id_after = next_id_;
  RoadNode replacement = *node;
  replacement.position = position;
  plan.replace_nodes.push_back(replacement);
  for (const RoadSegment& segment : graph_.segments) {
    if (segment.shape.intent != SegmentShapeIntent::kStraight ||
        (segment.node_a != node_id && segment.node_b != node_id)) {
      continue;
    }
    const RoadNode* node_a = find_node(graph_, segment.node_a);
    const RoadNode* node_b = find_node(graph_, segment.node_b);
    if (node_a == nullptr || node_b == nullptr) {
      return Result<bool>::Fail(CommitFailureCategory::kInternalError,
                                "road segment endpoint node is missing");
    }
    const Vec2d start = segment.node_a == node_id ? position : node_a->position;
    const Vec2d end = segment.node_b == node_id ? position : node_b->position;
    Result<SegmentShape> linear_shape = make_linear_shape(start, end);
    if (!linear_shape.ok) {
      return Result<bool>::Fail(linear_shape.failure_category, linear_shape.error);
    }
    RoadSegment updated = segment;
    updated.shape = std::move(linear_shape.value);
    plan.replace_segments.push_back(std::move(updated));
  }
  return Execute(plan);
}

Result<bool> RoadState::DeleteSegment(DeleteSegmentRequest request) {
  const RoadSegmentId segment_id = request.segment_id;
  const RoadSegment* target = find_segment(graph_, segment_id);
  if (target == nullptr) {
    return Result<bool>::Fail(CommitFailureCategory::kInvalidInput,
                              "road segment does not exist");
  }
  operations::OperationPlan plan{};
  std::uint64_t next_id = next_id_;
  plan.remove_segments.push_back(segment_id);
  const RoadCorridor* corridor = FindCorridorForSegment(graph_, segment_id);
  if (corridor == nullptr) {
    return Result<bool>::Fail(
        CommitFailureCategory::kInternalError, "road segment corridor is missing");
  }
  const auto ref_it =
      std::find_if(corridor->segments.begin(), corridor->segments.end(),
                   [segment_id](const DirectedSegmentRef& ref) {
                     return ref.segment_id == segment_id;
                   });
  if (ref_it == corridor->segments.end()) {
    return Result<bool>::Fail(
        CommitFailureCategory::kInternalError, "road corridor segment reference is missing");
  }
  std::vector<DirectedSegmentRef> before(corridor->segments.begin(), ref_it);
  std::vector<DirectedSegmentRef> after(ref_it + 1,
                                        corridor->segments.end());
  if (before.empty() && after.empty()) {
    plan.remove_corridors.push_back(corridor->id);
  } else if (before.empty() || after.empty()) {
    RoadCorridor replacement = *corridor;
    replacement.segments =
        before.empty() ? std::move(after) : std::move(before);
    plan.replace_corridors.push_back(std::move(replacement));
  } else {
    RoadCorridor start_side = *corridor;
    start_side.segments = std::move(before);
    RoadCorridor end_side{next_id++, corridor->layout_template_id,
                          std::move(after)};
    plan.replace_corridors.push_back(std::move(start_side));
    plan.add_corridors.push_back(std::move(end_side));
  }
  for (const RoadNodeId node_id :
       std::array<RoadNodeId, 2>{target->node_a, target->node_b}) {
    if (node_degree(graph_, node_id) == 1) {
      plan.remove_nodes.push_back(node_id);
    }
  }
  for (const NodeConnectionPolicyOverride& policy :
       graph_.connection_policy_overrides) {
    const std::size_t degree_after =
        node_degree(graph_, policy.node_id) -
        ((target->node_a == policy.node_id ||
          target->node_b == policy.node_id)
             ? 1
             : 0);
    if (degree_after == 0) {
      plan.remove_connection_policy_overrides.push_back(policy.id);
    }
  }
  for (const ManualLineMarking& marking : graph_.manual_lines) {
    if (marking.owner_segment_id == segment_id) {
      plan.remove_manual_lines.push_back(marking.id);
    }
  }
  for (const ManualAreaMarking& marking : graph_.manual_areas) {
    if (marking.owner_segment_id == segment_id) {
      plan.remove_manual_areas.push_back(marking.id);
    }
  }
  const auto removes_node = [&plan](RoadNodeId node_id) {
    return std::find(plan.remove_nodes.begin(), plan.remove_nodes.end(),
                     node_id) != plan.remove_nodes.end();
  };
  for (const ApproachGeometryOverride& override :
       graph_.approach_geometry_overrides) {
    if (override.key.segment_id == segment_id) {
      plan.remove_approach_geometry_overrides.push_back(override.key);
    }
  }
  for (const AutoMarkingOverride& override :
       graph_.auto_marking_overrides) {
    const bool segment_owner =
        override.key.owner.kind == MarkingOwner::Kind::kRoadSegment &&
        override.key.owner.segment_id == segment_id;
    const bool removed_junction_owner =
        override.key.owner.kind == MarkingOwner::Kind::kJunction &&
        removes_node(override.key.owner.node_id);
    const bool approach_owner =
        override.key.approach.has_value() &&
        override.key.approach->segment_id == segment_id;
    const bool track_owner =
        override.key.track.has_value() &&
        override.key.track->segment_id == segment_id;
    if (segment_owner || removed_junction_owner || approach_owner ||
        track_owner) {
      plan.remove_auto_marking_overrides.push_back(override.key);
    }
  }
  for (const JunctionMarkingOverride& override :
       graph_.junction_marking_overrides) {
    const bool removed_node_owner = removes_node(override.node_id);
    const bool source_owner =
        override.source.approach.segment_id == segment_id;
    const bool target_owner =
        override.target.has_value() &&
        override.target->approach.segment_id == segment_id;
    if (removed_node_owner || source_owner || target_owner) {
      plan.remove_junction_marking_overrides.push_back(override.id);
    }
  }
  const std::optional<RoadLayoutTransitionId> transition = target->transition;
  if (transition.has_value() &&
      std::none_of(
          graph_.segments.begin(), graph_.segments.end(),
          [segment_id, transition](const RoadSegment& segment) {
            return segment.id != segment_id &&
                   segment.transition == transition;
          })) {
    plan.remove_transitions.push_back(*transition);
  }
  plan.next_id_after = next_id;
  return Execute(plan);
}
} // namespace city::road
