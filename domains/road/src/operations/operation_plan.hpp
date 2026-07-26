#pragma once

#include "city/road/road.hpp"

#include <cstdint>
#include <vector>

namespace city::road::operations {

struct OperationPlan {
  std::uint64_t next_id_after = 0;

  std::vector<RoadNode> add_nodes{};
  std::vector<RoadNode> replace_nodes{};
  std::vector<RoadNodeId> remove_nodes{};

  std::vector<RoadSegment> add_segments{};
  std::vector<RoadSegment> replace_segments{};
  std::vector<RoadSegmentId> remove_segments{};

  std::vector<CrossSectionTemplate> add_section_templates{};
  std::vector<CrossSectionTemplate> replace_section_templates{};

  std::vector<SectionTransition> add_transitions{};
  std::vector<SectionTransitionId> remove_transitions{};

  std::vector<NodeConnectionPolicyOverride> add_connection_policy_overrides{};
  std::vector<NodeConnectionPolicyOverrideId> remove_connection_policy_overrides{};

  std::vector<ManualLineMarking> add_manual_lines{};
  std::vector<ManualMarkingId> remove_manual_lines{};
  std::vector<ManualAreaMarking> add_manual_areas{};
  std::vector<ManualMarkingId> remove_manual_areas{};
};

[[nodiscard]] Result<bool> Apply(const OperationPlan& plan,
                                 SavedRoadGraph& authoritative,
                                 std::uint64_t& next_id);

} // namespace city::road::operations
