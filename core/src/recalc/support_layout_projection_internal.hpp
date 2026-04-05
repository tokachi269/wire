#pragma once

#include "support_layout_materialization.hpp"

namespace wire::core {

struct SupportGroupDecision;
struct LoweredSupportGroupKey;

void clear_layout_endpoint_authority_projection(SupportLayoutEndpoint* endpoint);

void apply_endpoint_decision_to_layout_endpoint(const EndpointContinuityDecision& decision,
                                                SupportLayoutEndpoint* endpoint);

void apply_grouped_support_placement_to_layout_endpoint(const SupportGroupDecision& group_decision,
                                                        SupportLayoutEndpoint* endpoint);

const SupportGroupDecision* find_layout_support_group_decision_for_endpoint(const SpanSupportLayoutEntry& layout,
                                                                            const SupportLayoutEndpoint& endpoint,
                                                                            LoweredSupportGroupKey* out_key = nullptr);

} // namespace wire::core
