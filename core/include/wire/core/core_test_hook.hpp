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
  static std::unordered_map<ObjectId, SpanRuntimeState>& span_runtime_states(CoreState& state) {
    return state.runtime_.span_runtime_states;
  }
  static void erase_cached_span_support_layout_seed(CoreState& state, ObjectId span_id) {
    state.erase_cached_span_support_layout_seed(span_id);
  }
  static bool rebuild_span_geometry_from_seed(CoreState& state, ObjectId span_id, std::string* error_message) {
    return state.rebuild_span_geometry_from_seed(span_id, error_message);
  }
  static ValidationResult validate(CoreState& state) { return state.Validate(); }
  static std::unordered_map<BundleKind, BundleTemplate>& bundle_templates(CoreState& state) {
    return state.authoritative_.bundle_templates;
  }
  static std::unordered_map<CableTemplateId, CableTemplate>& cable_templates(CoreState& state) {
    return state.authoritative_.cable_templates;
  }
  static std::vector<SupportNode>& last_generation_support_nodes(CoreState& state) {
    return state.debug_.last_generation_support_nodes;
  }
  static std::vector<BackboneEdgeOrientation>& last_generation_edge_orientations(CoreState& state) {
    return state.debug_.last_generation_edge_orientations;
  }
  static std::unordered_map<ObjectId, JunctionRelation>& last_generation_junction_relations(CoreState& state) {
    return state.debug_.last_generation_junction_relations;
  }
};
#endif

} // namespace wire::core
