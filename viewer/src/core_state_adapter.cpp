#include "core_state_adapter.hpp"

#include "city/wire/core_state.hpp"

namespace viewer_core_state {

city::wire::EditResult<city::wire::GenerateBundleFromPathResult>
GenerateFromBackboneSpec(CoreState& state, const city::wire::BackboneSpec& spec) {
  return state.GenerateFromBackboneSpec(spec);
}

city::wire::EditResult<city::wire::ResolveBranchPickResult>
ResolveBranchPick(CoreState& state, const city::wire::PickResult& pick,
                  const city::wire::ResolveBranchPickOptions& options) {
  return state.ResolveBranchPick(pick, options);
}

city::wire::EditResult<ObjectId> SetPortWorldPositionManual(CoreState& state, ObjectId port_id,
                                                            const city::wire::Vec3d& world_position) {
  return state.SetPortWorldPositionManual(port_id, world_position);
}

city::wire::EditResult<ObjectId> ClearPoleOrientationOverride(CoreState& state, ObjectId pole_id) {
  return state.ClearPoleOrientationOverride(pole_id);
}

city::wire::EditResult<ObjectId> ClearSpanEndpointSocketOverride(CoreState& state, ObjectId span_id,
                                                                 bool is_start_endpoint) {
  return state.ClearSpanEndpointSocketOverride(span_id, is_start_endpoint);
}

city::wire::EditResult<ObjectId> ClearSpanBranchDownOffsetOverride(CoreState& state, ObjectId span_id) {
  return state.ClearSpanBranchDownOffsetOverride(span_id);
}

city::wire::BackboneResult SavedBackboneResult(const CoreState& state) {
  return state.SavedBackboneResult();
}

city::wire::EditResult<bool> UpdateGeometrySettings(CoreState& state, const city::wire::GeometrySettings& settings,
                                                    bool mark_all_spans_dirty) {
  return state.UpdateGeometrySettings(settings, mark_all_spans_dirty);
}

city::wire::EditResult<bool> UpdateLayoutSettings(CoreState& state, const city::wire::LayoutSettings& settings) {
  return state.UpdateLayoutSettings(settings);
}

city::wire::EditResult<bool> ApplyPoleTilt(CoreState& state, const std::vector<ObjectId>& pole_ids, double max_tilt_deg) {
  return state.ApplyPoleTilt(pole_ids, max_tilt_deg);
}

city::wire::EditResult<bool> ResetAllSpanReferenceLengths(CoreState& state, bool mark_all_spans_dirty) {
  return state.ResetAllSpanReferenceLengths(mark_all_spans_dirty);
}

city::wire::EditResult<bool> UpdateCableTemplate(CoreState& state, const city::wire::CableTemplate& cable_template,
                                                 const std::vector<ObjectId>& preferred_visible_span_ids) {
  return state.UpdateCableTemplate(cable_template, preferred_visible_span_ids);
}

city::wire::EditResult<bool> UpdateBundleTemplate(CoreState& state, const city::wire::BundleTemplate& bundle_template) {
  return state.UpdateBundleTemplate(bundle_template);
}

city::wire::EditResult<bool> ApplyBundleRelatedPoleTypeToExistingPoles(CoreState& state,
                                                                       city::wire::BundleKind bundle_template_id) {
  return state.ApplyBundleRelatedPoleTypeToExistingPoles(bundle_template_id);
}

city::wire::EditResult<bool> UpdatePoleTypeDefinition(CoreState& state,
                                                      const city::wire::PoleTypeDefinition& pole_type) {
  return state.UpdatePoleTypeDefinition(pole_type);
}

city::wire::EditResult<bool> UpdateVisualSettings(CoreState& state, const city::wire::VisualSettings& settings,
                                                  bool mark_all_spans_dirty) {
  return state.UpdateVisualSettings(settings, mark_all_spans_dirty);
}

city::wire::ValidationResult ValidateFast(const CoreState& state) {
  return state.ValidateFast();
}

} // namespace viewer_core_state
