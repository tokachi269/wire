#pragma once

#include "wire/core/detail_curve.hpp"
#include "wire/core/style_context.hpp"
#include "wire/core/support_layout_types.hpp"

namespace wire::core {

class CoreState;
struct Span;
struct Port;
struct Pole;
struct SpanRuntimeState;

struct ResolvedSpanCurveInputs {
  double basis_length = 0.0;
  BackboneFlowKind flow_kind = BackboneFlowKind::kMain;
  std::uint64_t variation_flow_key = 0;
  HierarchicalVariationSample sag_variation{};
  double effective_sag_ratio = 0.0;
  double endpoint_vertical_attachment_offset_m = 0.0;
  CurvePassMode pass_mode = CurvePassMode::kPassThrough;
  CurveEndpointMode endpoint_mode = CurveEndpointMode::kDirectThrough;
  CableContinuityPolicyHint continuity_preference = CableContinuityPolicyHint::kAuto;
  double bend_stiffness_hint = 1.0;
  double min_bend_radius_hint_m = 0.0;
};

[[nodiscard]] CurvePassMode curve_pass_mode_from_context(ConnectionContext context);

[[nodiscard]] CurveProfileHint detail_curve_profile_hint_from_support_layout(const SpanSupportLayoutEntry& layout);

[[nodiscard]] CurveConstraint make_curve_constraint_from_support_layout(
    const SupportLayoutEndpoint& endpoint, const Pole* owner_pole, double basis_length, double effective_sag_ratio,
    double bend_stiffness_hint, double min_bend_radius_hint_m, CableContinuityPolicyHint continuity_preference,
    CurvePassMode pass_mode, CurveProfileHint profile_hint, ConnectionContext connection_context);

[[nodiscard]] std::uint64_t variation_flow_key_for_span(const SpanRuntimeState* runtime, const Span& span);

[[nodiscard]] ResolvedStyleContext resolve_style_context_for_span(const CoreState& state, const Span& span,
                                                                 StyleObjectKind object_kind = StyleObjectKind::kSpan,
                                                                 std::uint32_t ordinal = 0,
                                                                 bool is_start_endpoint = false);

[[nodiscard]] CableMaterialStyleKind resolve_effective_cable_material_style(const CableTemplate* cable_template,
                                                                            const ResolvedStyleContext& style);

[[nodiscard]] CableAttachmentStyleHint resolve_effective_attachment_style(const CableTemplate* cable_template,
                                                                         const ResolvedStyleContext& style);

[[nodiscard]] ResolvedSpanCurveInputs resolve_span_curve_inputs(const CoreState& state, const Span& span,
                                                                const Port& port_a, const Port& port_b,
                                                                const Pole* pole_a, const Pole* pole_b,
                                                                const Vec3d& a, const Vec3d& b, double distance);

} // namespace wire::core
