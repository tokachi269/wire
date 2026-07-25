#pragma once

#include "wire/core/core_state.hpp"

namespace wire::core {

#if defined(WIRE_INTERNAL) || defined(WIRE_TESTING)
// Test-only backdoor for exceptional white-box checks.
// Production code must use CoreState public APIs only.
struct CoreStateTestHook {
  static EditState& edit_state(CoreState& state) { return state.authoritative_.edit_state; }
  static CacheState& cache_state(CoreState& state) { return state.runtime_.cache_state; }
  static RelationIndex& relation_index(CoreState& state) { return state.runtime_.relation_index; }
  static OverrideState& override_state(CoreState& state) { return state.authoritative_.override_state; }
  static IdGenerator& id_generator(CoreState& state) { return state.identity_.id_generator; }
  static bool authoritative_equals(const CoreState& a, const CoreState& b) { return a.authoritative_equals(b); }
  static std::unordered_map<ObjectId, SpanRuntimeState>& span_runtime_states(CoreState& state) {
    return state.runtime_.span_runtime_states;
  }
  static ValidationResult validate(CoreState& state) { return state.Validate(); }
  static std::unordered_map<BundleTemplateId, BundleTemplate>& bundle_templates(CoreState& state) {
    return state.authoritative_.bundle_templates;
  }
  static std::unordered_map<CableTemplateId, CableTemplate>& cable_templates(CoreState& state) {
    return state.authoritative_.cable_templates;
  }
  static std::unordered_map<ModelAssemblyTemplateId, ModelAssemblyTemplate>& model_assembly_templates(CoreState& state) {
    return state.authoritative_.model_assembly_templates;
  }
  static std::vector<SupportNode>& pending_support_nodes(CoreState& state) {
    return state.session_.pending_support_nodes;
  }
  static std::vector<BackboneEdgeOrientation>& last_generation_edge_orientations(CoreState& state) {
    return state.debug_.last_generation_edge_orientations;
  }
  static EditResult<UpdatePlan> make_update_plan(const CoreState& state, UpdateRequest request) {
    return state.make_update_plan(request);
  }
  static EditResult<bool> execute_update_plan(CoreState& state, const UpdatePlan& plan) {
    return state.execute_update_plan(plan);
  }
};
#endif

} // namespace wire::core
