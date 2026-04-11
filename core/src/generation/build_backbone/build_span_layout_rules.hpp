#pragma once

#include "../bundle_spans/bundle_span_context.hpp"

#include <unordered_map>

namespace wire::core::generation::detail {

void apply_seed_topology_projection(const EndpointContinuityDecision& source, ObjectId owner_pole_id,
                                    SupportLayoutSemanticDecision* decision);

void apply_seed_placement_projection(const EndpointContinuityDecision& source,
                                     SupportLayoutSemanticDecision* decision);

[[nodiscard]] int pair_height_rank_from_decision(
    const EditState& edit_state, const std::unordered_map<ObjectId, JunctionRelation>& junction_relations_by_node,
    ObjectId endpoint_node_id, const EndpointContinuityDecision& decision);

[[nodiscard]] SupportLayoutDecisionSeedEndpoint build_seed_endpoint_from_decision(
    const EditState& edit_state, const std::unordered_map<ObjectId, JunctionRelation>& junction_relations_by_node,
    const SegmentLaneAssignment& assignment, ObjectId endpoint_node_id, const Port& port,
    const EndpointContinuityDecision& decision);

} // namespace wire::core::generation::detail
