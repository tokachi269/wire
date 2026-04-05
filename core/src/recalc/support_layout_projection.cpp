#include "support_layout_projection_internal.hpp"

namespace wire::core {

void clear_layout_endpoint_authority_projection(SupportLayoutEndpoint* endpoint) {
  if (endpoint == nullptr) {
    return;
  }
  endpoint->support_authority = {};
  endpoint->has_visual_arm_geometry = false;
  endpoint->visual_arm_mount_world = {};
  endpoint->visual_arm_tip_world = {};
  endpoint->visual_insulator_base_world = {};
}

void apply_endpoint_decision_to_layout_endpoint(const EndpointContinuityDecision& decision,
                                                SupportLayoutEndpoint* endpoint) {
  if (endpoint == nullptr) {
    return;
  }
  endpoint->decision = decision;
  endpoint->decision.owner_pole_id = endpoint->owner_pole_id;
}

void apply_support_layout_decision_seed_endpoint(const SupportLayoutDecisionSeedEndpoint& seed,
                                                 SupportLayoutEndpoint* endpoint) {
  if (endpoint == nullptr) {
    return;
  }
  endpoint->endpoint_node_id = seed.endpoint_node_id;
  endpoint->owner_pole_id = seed.owner_pole_id;
  endpoint->port_id = seed.port_id;
  clear_layout_endpoint_authority_projection(endpoint);
  apply_endpoint_decision_to_layout_endpoint(seed.decision, endpoint);
  endpoint->support_authority = seed.support_authority;
  endpoint->flow_kind = seed.flow_kind;
  endpoint->origin = seed.origin;
  endpoint->port_source = seed.port_source;
  endpoint->side = seed.side;
  endpoint->has_visual_arm_geometry = seed.has_visual_arm_geometry;
  endpoint->visual_arm_mount_world = seed.visual_arm_mount_world;
  endpoint->visual_arm_tip_world = seed.visual_arm_tip_world;
  endpoint->visual_insulator_base_world = seed.visual_insulator_base_world;
  endpoint->automatic_branch_down_offset_m = seed.automatic_branch_down_offset_m;
  endpoint->branch_down_offset_m = seed.branch_down_offset_m;
  endpoint->down_offset_variation = seed.down_offset_variation;
}

void apply_support_layout_decision_seed(const SpanSupportLayoutDecisionSeed& seed, SpanSupportLayoutEntry* layout) {
  if (layout == nullptr) {
    return;
  }
  layout->span_id = seed.span_id;
  layout->requires_decision_seed = true;
  layout->flow_kind = seed.flow_kind;
  layout->pass_mode = seed.pass_mode;
  layout->variation_flow_key = seed.variation_flow_key;
  layout->lowering_kind = seed.lowering_kind;
  apply_support_layout_decision_seed_endpoint(seed.start, &layout->start);
  apply_support_layout_decision_seed_endpoint(seed.end, &layout->end);
  layout->support_group_decisions = seed.support_group_decisions;
}

void apply_grouped_support_placement_to_layout_endpoint(const SupportGroupDecision& group_decision,
                                                        SupportLayoutEndpoint* endpoint) {
  if (endpoint == nullptr) {
    return;
  }
  clear_layout_endpoint_authority_projection(endpoint);
  apply_endpoint_decision_to_layout_endpoint(group_decision.decision, endpoint);
  endpoint->support_authority = group_decision.support_authority;
  endpoint->side = group_decision.side;
  endpoint->origin = group_decision.origin;
  endpoint->automatic_branch_down_offset_m = group_decision.down_offset_m;
  endpoint->branch_down_offset_m = group_decision.down_offset_m;
  endpoint->down_offset_variation = group_decision.down_offset_variation;
  endpoint->support_world = endpoint->endpoint_world;
}

const SupportGroupDecision* find_layout_support_group_decision_for_endpoint(const SpanSupportLayoutEntry& layout,
                                                                            const SupportLayoutEndpoint& endpoint,
                                                                            LoweredSupportGroupKey* out_key) {
  if (endpoint.owner_pole_id == kInvalidObjectId || layout.support_group_decisions.empty() ||
      !UsesAuthoritativeGroupedLoweredSupport(endpoint.decision)) {
    return nullptr;
  }

  const LoweredSupportGroupKey exact_key = LoweredSupportGroupKeyFromDecision(endpoint.decision);
  if (const auto exact_it = layout.support_group_decisions.find(exact_key);
      exact_it != layout.support_group_decisions.end()) {
    if (out_key != nullptr) {
      *out_key = exact_it->first;
    }
    return &exact_it->second;
  }

  return nullptr;
}

} // namespace wire::core
