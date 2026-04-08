#include "wire/core/core_state.hpp"
#include "backbone_generation_plan_internal.hpp"

namespace wire::core {

using namespace generation::detail;

void CoreState::record_backbone_path_direction_debug(const BackboneGenerationRequestPlan& request_plan) {
  debug_.last_path_direction_debug = request_plan.direction_debug;
  path_direction_debug_records_access().push_back(request_plan.direction_debug);
  if (path_direction_debug_records_access().size() > 128) {
    path_direction_debug_records_access().erase(path_direction_debug_records_access().begin());
  }
}

EditResult<std::unique_ptr<BackboneGenerationPlan>> CoreState::build_backbone_generation_plan(
    const BackboneSpec& spec) const {
  EditResult<std::unique_ptr<BackboneGenerationPlan>> result{};
  EditResult<BackboneGenerationRequestPlan> request_plan_result = build_backbone_generation_request_plan(*this, spec);
  if (!request_plan_result.ok) {
    result.error = request_plan_result.error;
    return result;
  }

  auto plan = std::make_unique<BackboneGenerationPlan>();
  plan->request_plan = std::move(request_plan_result.value);

  EditResult<BackboneSupportChainPlan> support_chain_plan_result = build_backbone_support_chain_plan(plan->request_plan);
  if (!support_chain_plan_result.ok) {
    result.error = support_chain_plan_result.error;
    return result;
  }
  plan->support_chain_plan = std::move(support_chain_plan_result.value);

  EditResult<BackboneTopologyPlan> topology_plan_result =
      build_backbone_topology_plan(plan->request_plan, plan->support_chain_plan);
  if (!topology_plan_result.ok) {
    result.error = topology_plan_result.error;
    return result;
  }
  plan->topology_plan = std::move(topology_plan_result.value);

  EditResult<BackboneOrientationPlan> orientation_plan_result =
      build_backbone_orientation_plan(plan->request_plan, plan->support_chain_plan, plan->topology_plan);
  if (!orientation_plan_result.ok) {
    result.error = orientation_plan_result.error;
    return result;
  }
  plan->orientation_plan = std::move(orientation_plan_result.value);

  result.value = std::move(plan);
  result.ok = true;
  return result;
}

EditResult<bool> CoreState::validate_backbone_generation_plan(const BackboneGenerationPlan& plan) const {
  EditResult<bool> result{};
  if (plan.request_plan.request.path.polyline.size() < 2) {
    result.error = "backbone generation plan is missing path points";
    return result;
  }
  if (plan.request_plan.bundle_plans.empty()) {
    result.error = "backbone generation plan is missing bundle plans";
    return result;
  }
  if (plan.support_chain_plan.ordered_support_node_ids.size() < 2) {
    result.error = "backbone generation plan is missing support chain";
    return result;
  }
  if (plan.topology_plan.generation_backbone.edges.empty()) {
    result.error = "backbone generation plan is missing topology edges";
    return result;
  }
  if (plan.topology_plan.decision_phase.junction_relations_by_node.empty()) {
    result.error = "backbone generation plan is missing topology decision phase";
    return result;
  }
  if (plan.orientation_plan.planned_pole_orientations.empty()) {
    result.error = "backbone generation plan is missing planned orientations";
    return result;
  }
  result.value = true;
  result.ok = true;
  return result;
}

EditResult<GenerateBundleFromPathResult> CoreState::commit_backbone_generation_plan(
    std::unique_ptr<BackboneGenerationPlan> plan) {
  EditResult<GenerateBundleFromPathResult> result{};
  if (!plan) {
    result.error = "backbone generation plan is null";
    return result;
  }
  record_backbone_path_direction_debug(plan->request_plan);
  if (plan->request_plan.active_bundle_plans.empty()) {
    result.ok = true;
    return result;
  }
  EditResult<BackboneCommittedSupportChain> committed_chain_result =
      commit_backbone_support_chain_plan(plan->support_chain_plan);
  if (!committed_chain_result.ok) {
    result.error = committed_chain_result.error;
    return result;
  }

  EditResult<BackboneCommittedGenerationPlan> committed_plan_result = build_committed_backbone_generation_plan(
      plan->topology_plan, plan->orientation_plan, committed_chain_result.value.session_id,
      std::move(committed_chain_result.value.ordered_support_node_ids),
      std::move(committed_chain_result.value.support_node_by_id),
      std::move(committed_chain_result.value.committed_node_id_by_planned_node_id));
  if (!committed_plan_result.ok) {
    result.error = committed_plan_result.error;
    return result;
  }

  result = execute_committed_backbone_generation_plan(plan->request_plan, std::move(committed_plan_result.value),
                                                      std::move(committed_chain_result.value.generated_pole_ids),
                                                      std::move(committed_chain_result.value.change_set));
  return result;
}

} // namespace wire::core
