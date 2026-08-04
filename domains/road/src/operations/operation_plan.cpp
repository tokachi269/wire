#include "operation_plan.hpp"

#include <algorithm>
#include <string_view>

namespace city::road::operations {
namespace {

template <typename Entity, typename Id, typename GetId>
bool contains_id(const std::vector<Entity>& entities, Id id, GetId&& get_id) {
  return std::any_of(entities.begin(), entities.end(), [&](const Entity& entity) { return get_id(entity) == id; });
}

template <typename Entity, typename GetId>
Result<bool> add_entities(std::vector<Entity>& target,
                          const std::vector<Entity>& additions,
                          GetId&& get_id,
                          std::string_view label) {
  for (const Entity& addition : additions) {
    if (contains_id(target, get_id(addition), get_id)) {
      return Result<bool>::Fail(CommitFailureCategory::kInternalError, std::string(label) + " add id already exists");
    }
    target.push_back(addition);
  }
  return Result<bool>::Ok(true);
}

template <typename Entity, typename GetId>
Result<bool> replace_entities(std::vector<Entity>& target,
                              const std::vector<Entity>& replacements,
                              GetId&& get_id,
                              std::string_view label) {
  for (const Entity& replacement : replacements) {
    const auto it = std::find_if(target.begin(), target.end(),
                                 [&](const Entity& entity) { return get_id(entity) == get_id(replacement); });
    if (it == target.end()) {
      return Result<bool>::Fail(CommitFailureCategory::kInternalError, std::string(label) + " replacement id is missing");
    }
    *it = replacement;
  }
  return Result<bool>::Ok(true);
}

template <typename Entity, typename Id, typename GetId>
Result<bool> remove_entities(std::vector<Entity>& target,
                             const std::vector<Id>& removals,
                             GetId&& get_id,
                             std::string_view label) {
  for (const Id id : removals) {
    const auto it = std::find_if(target.begin(), target.end(),
                                 [&](const Entity& entity) { return get_id(entity) == id; });
    if (it == target.end()) {
      return Result<bool>::Fail(CommitFailureCategory::kInternalError, std::string(label) + " removal id is missing");
    }
    target.erase(it);
  }
  return Result<bool>::Ok(true);
}

Result<bool> remove_approach_overrides(std::vector<ApproachGeometryOverride>& target,
                                       const std::vector<ApproachKey>& removals) {
  for (const ApproachKey& key : removals) {
    const auto it = std::find_if(target.begin(), target.end(),
                                 [&](const ApproachGeometryOverride& entity) { return entity.key == key; });
    if (it == target.end()) {
      return Result<bool>::Fail(CommitFailureCategory::kInternalError, "approach geometry override removal key is missing");
    }
    target.erase(it);
  }
  return Result<bool>::Ok(true);
}

Result<bool> remove_auto_marking_overrides(std::vector<AutoMarkingOverride>& target,
                                           const std::vector<AutoMarkingKey>& removals) {
  for (const AutoMarkingKey& key : removals) {
    const auto it = std::find_if(target.begin(), target.end(),
                                 [&](const AutoMarkingOverride& entity) { return entity.key == key; });
    if (it == target.end()) {
      return Result<bool>::Fail(CommitFailureCategory::kInternalError, "auto marking override removal key is missing");
    }
    target.erase(it);
  }
  return Result<bool>::Ok(true);
}

} // namespace

Result<bool> Apply(const OperationPlan& plan, SavedRoadGraph& authoritative, std::uint64_t& next_id) {
  const auto node_id = [](const RoadNode& value) { return value.id; };
  const auto segment_id = [](const RoadSegment& value) { return value.id; };
  const auto corridor_id = [](const RoadCorridor& value) { return value.id; };
  const auto section_id = [](const CrossSectionTemplate& value) { return value.id; };
  const auto transition_id = [](const SectionTransition& value) { return value.id; };
  const auto lane_connection_id = [](const LaneConnection& value) { return value.id; };
  const auto boundary_continuation_id = [](const BoundaryContinuation& value) { return value.id; };
  const auto policy_override_id = [](const NodeConnectionPolicyOverride& value) { return value.id; };
  const auto line_id = [](const ManualLineMarking& value) { return value.id; };
  const auto area_id = [](const ManualAreaMarking& value) { return value.id; };

  Result<bool> result = remove_entities(authoritative.manual_lines, plan.remove_manual_lines, line_id, "manual line");
  if (!result.ok) return result;
  result = remove_entities(authoritative.manual_areas, plan.remove_manual_areas, area_id, "manual area");
  if (!result.ok) return result;
  result = remove_entities(authoritative.connection_policy_overrides, plan.remove_connection_policy_overrides,
                           policy_override_id, "connection policy override");
  if (!result.ok) return result;
  result = remove_approach_overrides(authoritative.approach_geometry_overrides,
                                     plan.remove_approach_geometry_overrides);
  if (!result.ok) return result;
  result = remove_auto_marking_overrides(authoritative.auto_marking_overrides,
                                         plan.remove_auto_marking_overrides);
  if (!result.ok) return result;
  result = remove_entities(authoritative.junction_marking_overrides,
                           plan.remove_junction_marking_overrides,
                           [](const JunctionMarkingOverride& value) { return value.id; },
                           "junction marking override");
  if (!result.ok) return result;
  result = remove_entities(authoritative.transitions, plan.remove_transitions, transition_id, "transition");
  if (!result.ok) return result;
  result = remove_entities(authoritative.segments, plan.remove_segments, segment_id, "segment");
  if (!result.ok) return result;
  result = remove_entities(authoritative.corridors, plan.remove_corridors,
                           corridor_id, "corridor");
  if (!result.ok) return result;
  result = remove_entities(authoritative.nodes, plan.remove_nodes, node_id, "node");
  if (!result.ok) return result;

  result = replace_entities(authoritative.nodes, plan.replace_nodes, node_id, "node");
  if (!result.ok) return result;
  result = replace_entities(authoritative.segments, plan.replace_segments, segment_id, "segment");
  if (!result.ok) return result;
  result = replace_entities(authoritative.corridors, plan.replace_corridors,
                            corridor_id, "corridor");
  if (!result.ok) return result;
  result = replace_entities(authoritative.section_templates, plan.replace_section_templates, section_id, "section");
  if (!result.ok) return result;

  result = add_entities(authoritative.nodes, plan.add_nodes, node_id, "node");
  if (!result.ok) return result;
  result = add_entities(authoritative.segments, plan.add_segments, segment_id, "segment");
  if (!result.ok) return result;
  result = add_entities(authoritative.corridors, plan.add_corridors, corridor_id,
                        "corridor");
  if (!result.ok) return result;
  result = add_entities(authoritative.section_templates, plan.add_section_templates, section_id, "section");
  if (!result.ok) return result;
  result = add_entities(authoritative.transitions, plan.add_transitions, transition_id, "transition");
  if (!result.ok) return result;
  result = add_entities(authoritative.lane_connections,
                        plan.add_lane_connections, lane_connection_id,
                        "lane connection");
  if (!result.ok) return result;
  result = add_entities(authoritative.boundary_continuations,
                        plan.add_boundary_continuations,
                        boundary_continuation_id,
                        "boundary continuation");
  if (!result.ok) return result;
  result = add_entities(authoritative.approach_geometry_overrides, plan.add_approach_geometry_overrides,
                        [](const ApproachGeometryOverride& value) { return value.key; },
                        "approach geometry override");
  if (!result.ok) return result;
  result = add_entities(authoritative.auto_marking_overrides, plan.add_auto_marking_overrides,
                        [](const AutoMarkingOverride& value) { return value.key; },
                        "auto marking override");
  if (!result.ok) return result;
  result = add_entities(authoritative.junction_marking_overrides,
                        plan.add_junction_marking_overrides,
                        [](const JunctionMarkingOverride& value) { return value.id; },
                        "junction marking override");
  if (!result.ok) return result;
  result = add_entities(authoritative.manual_lines, plan.add_manual_lines, line_id, "manual line");
  if (!result.ok) return result;
  result = add_entities(authoritative.manual_areas, plan.add_manual_areas, area_id, "manual area");
  if (!result.ok) return result;

  next_id = plan.next_id_after;
  return Result<bool>::Ok(true);
}

} // namespace city::road::operations
