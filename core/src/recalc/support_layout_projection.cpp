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

void apply_endpoint_decision_to_layout_endpoint(const SupportLayoutSemanticDecision& decision,
                                                SupportLayoutEndpoint* endpoint) {
  if (endpoint == nullptr) {
    return;
  }
  static_cast<SupportLayoutSemanticDecision&>(*endpoint) = decision;
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
  apply_endpoint_decision_to_layout_endpoint(seed, endpoint);
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
  endpoint->default_lower_required = seed.default_lower_required;
  endpoint->same_level_feasible = seed.same_level_feasible;
  endpoint->unresolved_same_level_conflict = seed.unresolved_same_level_conflict;
  endpoint->same_level_reason = seed.same_level_reason;
  endpoint->projected_spacing_topview_m = seed.projected_spacing_topview_m;
  endpoint->required_clearance_m = seed.required_clearance_m;
  endpoint->solver_used_same_level_constraint = seed.solver_used_same_level_constraint;
  endpoint->used_special_case_ports = seed.used_special_case_ports;
  endpoint->order_decision_policy = seed.order_decision_policy;
  endpoint->order_decision_choice = seed.order_decision_choice;
  endpoint->order_decision_choice_reason = seed.order_decision_choice_reason;
  endpoint->chosen_side = seed.chosen_side;
  endpoint->used_junction_pair_side_assignment = seed.used_junction_pair_side_assignment;
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
}

void apply_grouped_support_placement_to_layout_endpoint(const SupportGroupDecision& group_decision,
                                                        const LoweredSupportGroupPlacement& placement,
                                                        SupportLayoutEndpoint* endpoint) {
  if (endpoint == nullptr) {
    return;
  }
  clear_layout_endpoint_authority_projection(endpoint);
  apply_endpoint_decision_to_layout_endpoint(group_decision, endpoint);
  endpoint->support_authority = group_decision.support_authority;
  endpoint->side = group_decision.side;
  endpoint->origin = group_decision.origin;
  endpoint->order_decision_policy = group_decision.order_decision_policy;
  endpoint->order_decision_choice = group_decision.order_decision_choice;
  endpoint->order_decision_choice_reason = group_decision.order_decision_choice_reason;
  endpoint->chosen_side = group_decision.chosen_side;
  endpoint->used_junction_pair_side_assignment = group_decision.used_junction_pair_side_assignment;
  endpoint->automatic_branch_down_offset_m = placement.down_offset_m;
  endpoint->branch_down_offset_m = placement.down_offset_m;
  endpoint->down_offset_variation = placement.down_offset_variation;
  endpoint->support_world = endpoint->endpoint_world;
}

} // namespace wire::core
