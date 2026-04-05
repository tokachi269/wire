#pragma once

#include "grouped_span_common.hpp"

namespace wire::core::generation::detail {

void apply_seed_topology_projection(const EndpointContinuityDecision& source, ObjectId owner_pole_id,
                                    SupportLayoutSemanticDecision* decision);

} // namespace wire::core::generation::detail
