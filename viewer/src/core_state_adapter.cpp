#include "core_state_adapter.hpp"

#include "wire/core/core_state.hpp"

namespace viewer_core_state {

wire::core::EditResult<wire::core::GenerateBundleFromPathResult>
GenerateFromBackboneSpec(CoreState& state, const wire::core::BackboneSpec& spec) {
  return state.GenerateFromBackboneSpec(spec);
}

wire::core::EditResult<wire::core::ResolveBranchPickResult>
ResolveBranchPick(CoreState& state, const wire::core::PickResult& pick,
                  const wire::core::ResolveBranchPickOptions& options) {
  return state.ResolveBranchPick(pick, options);
}

wire::core::EditResult<ObjectId> SetPortWorldPositionManual(CoreState& state, ObjectId port_id,
                                                            const wire::core::Vec3d& world_position) {
  return state.SetPortWorldPositionManual(port_id, world_position);
}

wire::core::EditResult<ObjectId> ClearPoleOrientationOverride(CoreState& state, ObjectId pole_id) {
  return state.ClearPoleOrientationOverride(pole_id);
}

wire::core::EditResult<ObjectId> ClearSpanEndpointSocketOverride(CoreState& state, ObjectId span_id,
                                                                 bool is_start_endpoint) {
  return state.ClearSpanEndpointSocketOverride(span_id, is_start_endpoint);
}

wire::core::EditResult<ObjectId> ClearSpanBranchDownOffsetOverride(CoreState& state, ObjectId span_id) {
  return state.ClearSpanBranchDownOffsetOverride(span_id);
}

wire::core::BackboneResult BuildSavedBackboneResult(const CoreState& state) {
  return state.BuildSavedBackboneResult();
}

wire::core::EditResult<bool> UpdateGeometrySettings(CoreState& state, const wire::core::GeometrySettings& settings,
                                                    bool mark_all_spans_dirty) {
  return state.UpdateGeometrySettings(settings, mark_all_spans_dirty);
}

wire::core::EditResult<bool> UpdateLayoutSettings(CoreState& state, const wire::core::LayoutSettings& settings) {
  return state.UpdateLayoutSettings(settings);
}

wire::core::EditResult<bool> ApplyPoleTilt(CoreState& state, const std::vector<ObjectId>& pole_ids, double max_tilt_deg) {
  return state.ApplyPoleTilt(pole_ids, max_tilt_deg);
}

wire::core::EditResult<bool> ResetAllSpanReferenceLengths(CoreState& state, bool mark_all_spans_dirty) {
  return state.ResetAllSpanReferenceLengths(mark_all_spans_dirty);
}

wire::core::EditResult<bool> UpdateCableTemplate(CoreState& state, const wire::core::CableTemplate& cable_template,
                                                 const std::vector<ObjectId>& preferred_visible_span_ids) {
  return state.UpdateCableTemplate(cable_template, preferred_visible_span_ids);
}

wire::core::EditResult<bool> UpdateBundleTemplate(CoreState& state, const wire::core::BundleTemplate& bundle_template) {
  return state.UpdateBundleTemplate(bundle_template);
}

wire::core::EditResult<bool> ApplyBundleRelatedPoleTypeToExistingPoles(CoreState& state,
                                                                       wire::core::BundleKind bundle_template_id) {
  return state.ApplyBundleRelatedPoleTypeToExistingPoles(bundle_template_id);
}

wire::core::EditResult<bool> UpdatePoleTypeDefinition(CoreState& state,
                                                      const wire::core::PoleTypeDefinition& pole_type) {
  return state.UpdatePoleTypeDefinition(pole_type);
}

wire::core::EditResult<bool> UpdateVisualSettings(CoreState& state, const wire::core::VisualSettings& settings,
                                                  bool mark_all_spans_dirty) {
  return state.UpdateVisualSettings(settings, mark_all_spans_dirty);
}

wire::core::CommitResult Commit(CoreState& state, const wire::core::CommitOptions& options) {
  return state.Commit(options);
}

} // namespace viewer_core_state
