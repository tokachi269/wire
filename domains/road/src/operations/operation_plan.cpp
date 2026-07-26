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
      return Result<bool>::Fail(ErrorKind::kInternal, std::string(label) + " add id already exists");
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
      return Result<bool>::Fail(ErrorKind::kInternal, std::string(label) + " replacement id is missing");
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
      return Result<bool>::Fail(ErrorKind::kInternal, std::string(label) + " removal id is missing");
    }
    target.erase(it);
  }
  return Result<bool>::Ok(true);
}

} // namespace

Result<bool> Apply(const OperationPlan& plan, SavedRoadGraph& authoritative, std::uint64_t& next_id) {
  const auto node_id = [](const RoadNode& value) { return value.id; };
  const auto segment_id = [](const RoadSegment& value) { return value.id; };
  const auto section_id = [](const CrossSectionTemplate& value) { return value.id; };
  const auto transition_id = [](const SectionTransition& value) { return value.id; };
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
  result = remove_entities(authoritative.transitions, plan.remove_transitions, transition_id, "transition");
  if (!result.ok) return result;
  result = remove_entities(authoritative.segments, plan.remove_segments, segment_id, "segment");
  if (!result.ok) return result;
  result = remove_entities(authoritative.nodes, plan.remove_nodes, node_id, "node");
  if (!result.ok) return result;

  result = replace_entities(authoritative.nodes, plan.replace_nodes, node_id, "node");
  if (!result.ok) return result;
  result = replace_entities(authoritative.segments, plan.replace_segments, segment_id, "segment");
  if (!result.ok) return result;
  result = replace_entities(authoritative.section_templates, plan.replace_section_templates, section_id, "section");
  if (!result.ok) return result;

  result = add_entities(authoritative.nodes, plan.add_nodes, node_id, "node");
  if (!result.ok) return result;
  result = add_entities(authoritative.segments, plan.add_segments, segment_id, "segment");
  if (!result.ok) return result;
  result = add_entities(authoritative.section_templates, plan.add_section_templates, section_id, "section");
  if (!result.ok) return result;
  result = add_entities(authoritative.transitions, plan.add_transitions, transition_id, "transition");
  if (!result.ok) return result;
  result = add_entities(authoritative.connection_policy_overrides, plan.add_connection_policy_overrides,
                        policy_override_id, "connection policy override");
  if (!result.ok) return result;
  result = add_entities(authoritative.manual_lines, plan.add_manual_lines, line_id, "manual line");
  if (!result.ok) return result;
  result = add_entities(authoritative.manual_areas, plan.add_manual_areas, area_id, "manual area");
  if (!result.ok) return result;

  next_id = plan.next_id_after;
  return Result<bool>::Ok(true);
}

} // namespace city::road::operations
