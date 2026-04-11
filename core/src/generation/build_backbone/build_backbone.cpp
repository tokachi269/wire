#include "wire/core/core_state.hpp"
#include "build_backbone_types.hpp"

namespace wire::core {

using namespace generation::detail;

void CoreState::save_path_direction_debug(const BackboneGenerationRequestPlan& build_request) {
  debug_.last_path_direction_debug = build_request.direction_debug;
  path_direction_debug_records_access().push_back(build_request.direction_debug);
  if (path_direction_debug_records_access().size() > 128) {
    path_direction_debug_records_access().erase(path_direction_debug_records_access().begin());
  }
}

EditResult<std::unique_ptr<BackboneBuildDraft>> CoreState::prepare_backbone_build(
    const BackboneSpec& spec) const {
  EditResult<std::unique_ptr<BackboneBuildDraft>> out{};
  EditResult<BackboneGenerationRequestPlan> build_request_out = build_backbone_generation_request_plan(*this, spec);
  if (!build_request_out.ok) {
    out.error = build_request_out.error;
    return out;
  }

  auto draft = std::make_unique<BackboneBuildDraft>();
  draft->build_request = std::move(build_request_out.value);

  EditResult<BackboneSupportChainPlan> support_chain_out = build_backbone_support_chain_plan(draft->build_request);
  if (!support_chain_out.ok) {
    out.error = support_chain_out.error;
    return out;
  }
  draft->support_chain = std::move(support_chain_out.value);

  EditResult<BackboneTopologyPlan> route_topology_out =
      build_backbone_topology_plan(draft->build_request, draft->support_chain);
  if (!route_topology_out.ok) {
    out.error = route_topology_out.error;
    return out;
  }
  draft->route_topology = std::move(route_topology_out.value);

  EditResult<BackboneOrientationPlan> pole_facing_out =
      build_backbone_orientation_plan(draft->build_request, draft->support_chain, draft->route_topology);
  if (!pole_facing_out.ok) {
    out.error = pole_facing_out.error;
    return out;
  }
  draft->pole_facing = std::move(pole_facing_out.value);

  out.value = std::move(draft);
  out.ok = true;
  return out;
}

EditResult<bool> CoreState::check_backbone_build_input(const BackboneBuildDraft& draft) const {
  EditResult<bool> out{};
  if (draft.build_request.request.path.polyline.size() < 2) {
    out.error = "backbone generation plan is missing path points";
    return out;
  }
  if (draft.build_request.bundle_plans.empty()) {
    out.error = "backbone generation plan is missing bundle plans";
    return out;
  }
  if (draft.support_chain.ordered_support_node_ids.size() < 2) {
    out.error = "backbone generation plan is missing support chain";
    return out;
  }
  if (draft.route_topology.generation_backbone.edges.empty()) {
    out.error = "backbone generation plan is missing topology edges";
    return out;
  }
  if (draft.route_topology.decision_phase.junction_relations_by_node.empty()) {
    out.error = "backbone generation plan is missing topology decision phase";
    return out;
  }
  if (draft.pole_facing.planned_pole_orientations.empty()) {
    out.error = "backbone generation plan is missing planned orientations";
    return out;
  }
  out.value = true;
  out.ok = true;
  return out;
}

EditResult<GenerateBundleFromPathResult> CoreState::build_backbone_from_draft(
    std::unique_ptr<BackboneBuildDraft> draft) {
  EditResult<GenerateBundleFromPathResult> out{};
  if (!draft) {
    out.error = "backbone generation plan is null";
    return out;
  }
  save_path_direction_debug(draft->build_request);
  if (draft->build_request.active_bundle_plans.empty()) {
    out.ok = true;
    return out;
  }
  EditResult<BackboneCommittedSupportChain> support_chain_out = realize_support_chain(draft->support_chain);
  if (!support_chain_out.ok) {
    out.error = support_chain_out.error;
    return out;
  }

  EditResult<BackboneCommittedGenerationPlan> real_node_build_out = remap_backbone_build_to_real_nodes(
      draft->route_topology, draft->pole_facing, support_chain_out.value.session_id,
      std::move(support_chain_out.value.ordered_support_node_ids),
      std::move(support_chain_out.value.support_node_by_id),
      std::move(support_chain_out.value.committed_node_id_by_planned_node_id));
  if (!real_node_build_out.ok) {
    out.error = real_node_build_out.error;
    return out;
  }

  out = build_backbone_from_real_nodes(draft->build_request, std::move(real_node_build_out.value),
                                       std::move(support_chain_out.value.generated_pole_ids),
                                       std::move(support_chain_out.value.change_set));
  return out;
}

} // namespace wire::core
