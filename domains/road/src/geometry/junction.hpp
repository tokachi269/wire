#pragma once

#include "city/road/road.hpp"

namespace city::road::internal {

// Corner and junction surfaces follow from resolved gates alone. These take the
// gates and the control distances already decided by connection resolution.
[[nodiscard]] Result<ConnectionGeometry>
generate_connection_geometry(RoadNodeId node_id, const ConnectionGate &first,
                             const ConnectionGate &second,
                             const SectionEvaluation &first_section,
                             const SectionEvaluation &second_section,
                             double corner_control_m);

[[nodiscard]] Result<JunctionGeometry>
generate_junction_geometry(RoadNodeId node_id,
                           const std::vector<ApproachKey> &ordered_approaches,
                           const std::vector<ConnectionGate> &gates,
                           const std::vector<const SectionEvaluation *> &sections,
                           double junction_corner_control_m);

} // namespace city::road::internal
