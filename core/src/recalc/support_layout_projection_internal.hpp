#pragma once

#include "support_layout_materialization.hpp"

namespace wire::core {

void clear_layout_endpoint_authority_projection(LayoutEndpoint* endpoint);

void apply_endpoint_decision_to_layout_endpoint(const LayoutSemantic& decision,
                                                LayoutEndpoint* endpoint);

void apply_grouped_support_placement_to_layout_endpoint(const SupportGroupDecision& group_decision,
                                                        const LoweredSupportGroupPlacement& placement,
                                                        LayoutEndpoint* endpoint);

} // namespace wire::core
