#pragma once

#include "app_state.hpp"

namespace viewer_core_state {

[[nodiscard]] inline wire::core::CoreView View(const CoreState& state) {
  return wire::core::CoreView(state);
}

wire::core::EditResult<wire::core::GenerateBundleFromPathResult>
GenerateFromBackboneSpec(CoreState& state, const wire::core::BackboneSpec& spec);

wire::core::EditResult<wire::core::ResolveBranchPickResult>
ResolveBranchPick(CoreState& state, const wire::core::PickResult& pick,
                  const wire::core::ResolveBranchPickOptions& options);

wire::core::EditResult<ObjectId> SetPortWorldPositionManual(CoreState& state, ObjectId port_id,
                                                            const wire::core::Vec3d& world_position);

wire::core::EditResult<ObjectId> ClearPoleOrientationOverride(CoreState& state, ObjectId pole_id);

wire::core::EditResult<ObjectId> ClearSpanEndpointSocketOverride(CoreState& state, ObjectId span_id,
                                                                 bool is_start_endpoint);

wire::core::EditResult<ObjectId> ClearSpanBranchDownOffsetOverride(CoreState& state, ObjectId span_id);

[[nodiscard]] wire::core::BackboneResult SavedBackboneResult(const CoreState& state);

wire::core::EditResult<bool> UpdateGeometrySettings(CoreState& state, const wire::core::GeometrySettings& settings,
                                                    bool mark_all_spans_dirty);

wire::core::EditResult<bool> UpdateLayoutSettings(CoreState& state, const wire::core::LayoutSettings& settings);

wire::core::EditResult<bool> ApplyPoleTilt(CoreState& state, const std::vector<ObjectId>& pole_ids, double max_tilt_deg);

wire::core::EditResult<bool> ResetAllSpanReferenceLengths(CoreState& state, bool mark_all_spans_dirty);

wire::core::EditResult<bool> UpdateCableTemplate(CoreState& state, const wire::core::CableTemplate& cable_template,
                                                 const std::vector<ObjectId>& preferred_visible_span_ids);

wire::core::EditResult<bool> UpdateBundleTemplate(CoreState& state, const wire::core::BundleTemplate& bundle_template);
wire::core::EditResult<bool> ApplyBundleRelatedPoleTypeToExistingPoles(CoreState& state,
                                                                       wire::core::BundleKind bundle_template_id);

wire::core::EditResult<bool> UpdatePoleTypeDefinition(CoreState& state,
                                                      const wire::core::PoleTypeDefinition& pole_type);

wire::core::EditResult<bool> UpdateVisualSettings(CoreState& state, const wire::core::VisualSettings& settings,
                                                  bool mark_all_spans_dirty);
wire::core::EditResult<bool>
UpdateExperimentalSpanMemberPopulationConfig(
    CoreState& state, const wire::core::ExperimentalSpanMemberPopulationConfig& config);
[[nodiscard]] const wire::core::ExperimentalSpanMemberPopulationConfig&
ExperimentalSpanMemberPopulationConfig(const CoreState& state);

[[nodiscard]] wire::core::ValidationResult ValidateFast(const CoreState& state);

} // namespace viewer_core_state
