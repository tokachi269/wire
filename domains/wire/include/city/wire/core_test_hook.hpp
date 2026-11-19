#pragma once

#include "city/wire/core_state.hpp"

namespace city::wire {

#if defined(WIRE_INTERNAL) || defined(WIRE_TESTING)
// Test-only backdoor for exceptional white-box checks.
// Production code must use CoreState public APIs only.
struct CoreStateTestHook {
  static EditState& edit_state(CoreState& state) { return state.authoritative_.edit_state; }
  static CacheState& cache_state(CoreState& state) { return state.runtime_.cache_state; }
  static RelationIndex& relation_index(CoreState& state) { return state.runtime_.relation_index; }
  static OverrideState& override_state(CoreState& state) { return state.authoritative_.override_state; }
  static std::vector<SavedBackboneBundleVariation>& backbone_bundle_variations(
      CoreState& state) {
    return state.authoritative_.backbone_bundle_variations;
  }
  static SavedBackboneGraph& backbone(CoreState& state) {
    return state.authoritative_.backbone;
  }
  static void rebuild_backbone_index(CoreState& state) {
    state.rebuild_backbone_index();
  }
  static IdGenerator& id_generator(CoreState& state) { return state.identity_.id_generator; }
  static bool authoritative_equals(const CoreState& a, const CoreState& b) { return a.authoritative_equals(b); }
  static std::unordered_map<ObjectId, SpanRuntimeState>& span_runtime_states(CoreState& state) {
    return state.runtime_.span_runtime_states;
  }
  static ValidationResult validate(CoreState& state) { return state.Validate(); }
  static EditResult<bool> bind_backbone_row_continuity(
      CoreState& state, ObjectId node_id, ObjectId edge_bundle_a,
      std::size_t lane_a, ObjectId edge_bundle_b, std::size_t lane_b) {
    return state.bind_backbone_row_continuity(
        node_id, edge_bundle_a, lane_a, edge_bundle_b, lane_b);
  }
  static std::unordered_map<BundleTemplateId, BundleTemplate>& bundle_templates(CoreState& state) {
    return state.authoritative_.bundle_templates;
  }
  static std::unordered_map<CableTemplateId, CableTemplate>& cable_templates(CoreState& state) {
    return state.authoritative_.cable_templates;
  }
  static std::unordered_map<ModelAssemblyTemplateId, ModelAssemblyTemplate>& model_assembly_templates(CoreState& state) {
    return state.authoritative_.model_assembly_templates;
  }
  static bool erase_backbone_span_binding(CoreState& state,
                                          ObjectId edge_bundle_id,
                                          std::size_t lane_index) {
    SavedBackboneGraph& graph = state.authoritative_.backbone;
    const std::size_t before = graph.span_bindings.size();
    std::erase_if(
        graph.span_bindings,
        [&](const SavedBackboneSpanBinding& binding) {
          return binding.edge_bundle_id == edge_bundle_id &&
                 binding.lane_index == lane_index;
        });
    if (graph.span_bindings.size() + 1 != before) {
      return false;
    }
    BackboneIndex& index = state.runtime_.backbone_index;
    index.edge_bundle_spans.clear();
    index.span_edge_bundle.clear();
    index.edge_bundle_span_bindings.clear();
    index.span_bindings_by_span.clear();
    for (std::size_t i = 0; i < graph.span_bindings.size(); ++i) {
      const SavedBackboneSpanBinding& binding = graph.span_bindings[i];
      index.edge_bundle_spans[binding.edge_bundle_id].push_back(binding.span_id);
      index.span_edge_bundle[binding.span_id] = binding.edge_bundle_id;
      index.edge_bundle_span_bindings[binding.edge_bundle_id].push_back(i);
      index.span_bindings_by_span[binding.span_id].push_back(i);
    }
    return true;
  }
  static bool erase_backbone_row_continuity(CoreState& state,
                                            ObjectId edge_bundle_id,
                                            std::size_t lane_index) {
    SavedBackboneGraph& graph = state.authoritative_.backbone;
    const std::size_t before = graph.row_continuities.size();
    std::erase_if(
        graph.row_continuities,
        [&](const SavedBackboneRowContinuity& continuity) {
          return (continuity.a.edge_bundle_id == edge_bundle_id &&
                  continuity.a.lane_index == lane_index) ||
                 (continuity.b.edge_bundle_id == edge_bundle_id &&
                  continuity.b.lane_index == lane_index);
        });
    return graph.row_continuities.size() + 1 == before;
  }
  static EditResult<bool> update_backbone_port_binding_frame_exact(
      CoreState& state, ObjectId edge_bundle_id,
      const SavedBackboneRowKey& row_key, std::size_t lane_index,
      double layout_yaw_deg, int support_level, int support_group_id,
      ObjectId port_id, const Vec3d& world_position) {
    return state.update_backbone_port_binding_frame_exact(
        edge_bundle_id, row_key, lane_index, layout_yaw_deg, support_level,
        support_group_id, port_id, world_position);
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

} // namespace city::wire
