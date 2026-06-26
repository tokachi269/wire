#include "detail_curve_input_resolution.hpp"

#include "wire/core/core_state.hpp"
#include "wire/core/core_view.hpp"
#include "wire/core/coord_utils.hpp"
#include "support_layout_materialization.hpp"

#include <algorithm>
#include <cmath>

namespace wire::core {

namespace {

std::uint64_t splitmix64(std::uint64_t x) {
  x += 0x9E3779B97F4A7C15ull;
  x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ull;
  x = (x ^ (x >> 27)) * 0x94D049BB133111EBull;
  return x ^ (x >> 31);
}

std::uint64_t hash_combine(std::uint64_t seed, std::uint64_t value) {
  return splitmix64(seed ^ (value + 0x9E3779B97F4A7C15ull + (seed << 6) + (seed >> 2)));
}

std::uint64_t fallback_variation_flow_key_for_span(const Span& span) {
  std::uint64_t key = hash_combine(span.generation.generation_session_id, static_cast<std::uint64_t>(span.bundle_id));
  key = hash_combine(key, static_cast<std::uint64_t>(span.endpoint_node_a_id));
  key = hash_combine(key, static_cast<std::uint64_t>(span.endpoint_node_b_id));
  key = hash_combine(key, static_cast<std::uint64_t>(span.placement_context));
  return key;
}

ConnectionCategory category_from_span_layer(SpanLayer layer) {
  switch (layer) {
  case SpanLayer::kHighVoltage:
    return ConnectionCategory::kHighVoltage;
  case SpanLayer::kCommunication:
    return ConnectionCategory::kCommunication;
  case SpanLayer::kOptical:
    return ConnectionCategory::kOptical;
  case SpanLayer::kDrop:
    return ConnectionCategory::kDrop;
  case SpanLayer::kLowVoltage:
  default:
    return ConnectionCategory::kLowVoltage;
  }
}

} // namespace

CurvePassMode curve_pass_mode_from_context(ConnectionContext context) {
  switch (context) {
  case ConnectionContext::kBranchAdd:
    return CurvePassMode::kBranch;
  case ConnectionContext::kDropAdd:
    return CurvePassMode::kTerminate;
  default:
    return CurvePassMode::kPassThrough;
  }
}

CurveProfileHint detail_curve_profile_hint_from_support_layout(const SpanLayoutEntry& layout) {
  auto has_valid_departure = [](const SupportLayoutEndpoint& endpoint) {
    Vec3d departure = endpoint.departure_dir;
    return Normalize(&departure);
  };
  const bool grouped_lowered_support =
      UsesAuthoritativeGroupedLoweredSupport(layout.start) ||
      UsesAuthoritativeGroupedLoweredSupport(layout.end);
  if (layout.pass_mode != CurvePassMode::kPassThrough) {
    return CurveProfileHint::kAuto;
  }
  if (grouped_lowered_support &&
      (!layout.start.same_level_feasible || !layout.end.same_level_feasible)) {
    return CurveProfileHint::kGroupedLoweredSupport;
  }
  if (layout.start.endpoint_mode == CurveEndpointMode::kOffsetEndpoint ||
      layout.end.endpoint_mode == CurveEndpointMode::kOffsetEndpoint) {
    return CurveProfileHint::kAuto;
  }
  if (layout.basis_length_m < 10.0 || !has_valid_departure(layout.start) || !has_valid_departure(layout.end)) {
    return CurveProfileHint::kAuto;
  }
  const double support_height_delta_m =
      std::abs(HeightAlongWorldUp(layout.end.support_world) - HeightAlongWorldUp(layout.start.support_world));
  const Vec3d support_delta = layout.end.support_world - layout.start.support_world;
  const Vec3d vertical = ScaleVec(WorldUp(), Dot(support_delta, WorldUp()));
  const double support_horizontal_distance_m =
      std::sqrt(std::max(0.0, LengthSquared(support_delta - vertical)));
  if (support_height_delta_m < 1.8) {
    return CurveProfileHint::kAuto;
  }
  if (support_height_delta_m < support_horizontal_distance_m * 0.16) {
    return CurveProfileHint::kAuto;
  }
  return CurveProfileHint::kCompositeHeightTransition;
}

CurveConstraint make_curve_constraint_from_support_layout(const SupportLayoutEndpoint& endpoint, const Pole* owner_pole,
                                                          double basis_length, double effective_sag_ratio,
                                                          double bend_stiffness_hint, double min_bend_radius_hint_m,
                                                          CableContinuityPolicyHint continuity_preference,
                                                          CurvePassMode pass_mode, CurveProfileHint profile_hint,
                                                          ConnectionContext connection_context) {
  CurveConstraint constraint{};
  constraint.point = endpoint.endpoint_world;
  constraint.tangent_dir = endpoint.departure_dir;
  constraint.tangent_strength = 1.0;
  constraint.tangent_length_hint_m = basis_length * 0.30;
  constraint.support_departure_length_m = endpoint.local_departure_length_m;
  constraint.bend_stiffness_hint = bend_stiffness_hint;
  constraint.min_bend_radius_hint_m = min_bend_radius_hint_m;
  constraint.endpoint_offset = endpoint.endpoint_offset;
  constraint.sag_hint = effective_sag_ratio * 0.5;
  constraint.slack_hint = 0.0;
  constraint.continuity_preference = continuity_preference;
  constraint.pass_mode = pass_mode;
  constraint.endpoint_mode = endpoint.endpoint_mode;
  constraint.profile_hint = profile_hint;
  constraint.corner_pass =
      (connection_context == ConnectionContext::kCornerPass) && owner_pole != nullptr &&
      owner_pole->context.kind == PoleContextKind::kCorner && owner_pole->context.corner_angle_deg > 1e-6;
  constraint.corner_angle_deg = (owner_pole == nullptr) ? 0.0 : owner_pole->context.corner_angle_deg;
  return constraint;
}

std::uint64_t variation_flow_key_for_span(const SpanRuntimeState* runtime, const Span& span) {
  if (runtime != nullptr && runtime->variation_flow_key != 0) {
    return runtime->variation_flow_key;
  }
  return fallback_variation_flow_key_for_span(span);
}

ResolvedStyleContext resolve_style_context_for_span(const CoreState& state, const Span& span, StyleObjectKind object_kind,
                                                    std::uint32_t ordinal, bool is_start_endpoint) {
  const CoreView view = state.view();
  const SpanRuntimeState* runtime = view.find_span_runtime_state(span.id);
  const std::uint64_t variation_flow_key = variation_flow_key_for_span(runtime, span);

  const Bundle* bundle = view.bundles().find(span.bundle_id);
  const BundleTemplate* bundle_template = nullptr;
  if (bundle != nullptr) {
    const auto it = view.bundle_templates().find(bundle->bundle_template_id);
    if (it != view.bundle_templates().end()) {
      bundle_template = &it->second;
    }
  }

  BackboneFlowKind flow_kind = BackboneFlowKind::kMain;
  if (const SpanLayoutEntry* layout = state.span_layout(span.id).entry; layout != nullptr) {
    flow_kind = layout->flow_kind;
  } else {
    const Port* port_a = view.edit_state().ports.find(span.port_a_id);
    const Port* port_b = view.edit_state().ports.find(span.port_b_id);
    if (port_a != nullptr && port_b != nullptr) {
      flow_kind = support_layout_flow_kind_for_span(span, *port_a, *port_b);
    }
  }

  StyleRouteKey route_key{};
  route_key.family_id = variation_flow_key;
  route_key.bundle_template_id = (bundle != nullptr) ? bundle->bundle_template_id : BundleKind::kLowVoltage;
  route_key.category = (bundle_template != nullptr) ? bundle_template->category : category_from_span_layer(span.layer);
  route_key.flow_kind = flow_kind;

  StyleObjectKey object_key{};
  object_key.route = route_key;
  object_key.segment_index = span.generation.generation_order;
  object_key.lane_index = 0;
  object_key.kind = object_kind;
  object_key.ordinal = ordinal;
  object_key.is_start_endpoint = is_start_endpoint;
  return ResolveStyleContext(view.context_profile(), route_key, object_key);
}

CableMaterialStyleKind resolve_effective_cable_material_style(const CableTemplate* cable_template,
                                                              const ResolvedStyleContext& style) {
  const CableMaterialStyleKind template_family =
      (cable_template == nullptr) ? CableMaterialStyleKind::kGeneric : cable_template->material_style;
  if (template_family != CableMaterialStyleKind::kGeneric) {
    return template_family;
  }
  if (style.route.cable_family != CableMaterialStyleKind::kGeneric) {
    return style.route.cable_family;
  }
  if (style.district.cable_family != CableMaterialStyleKind::kGeneric) {
    return style.district.cable_family;
  }
  return CableMaterialStyleKind::kGeneric;
}

CableAttachmentStyleHint resolve_effective_attachment_style(const CableTemplate* cable_template,
                                                            const ResolvedStyleContext& style) {
  const CableAttachmentStyleHint template_family =
      (cable_template == nullptr) ? CableAttachmentStyleHint::kAuto : cable_template->attachment_style;
  if (template_family != CableAttachmentStyleHint::kAuto) {
    return template_family;
  }
  if (style.route.attachment_family != CableAttachmentStyleHint::kAuto) {
    return style.route.attachment_family;
  }
  if (style.district.attachment_family != CableAttachmentStyleHint::kAuto) {
    return style.district.attachment_family;
  }
  return CableAttachmentStyleHint::kDirectThrough;
}

ResolvedSpanCurveInputs resolve_span_curve_inputs(const CoreState& state, const Span& span, const Port& port_a,
                                                  const Port& port_b, const Pole* pole_a, const Pole* pole_b,
                                                  const Vec3d& a, const Vec3d& b, double distance) {
  const CoreView view = state.view();
  const Bundle* bundle = view.bundles().find(span.bundle_id);
  const BundleTemplate* bundle_template = nullptr;
  if (bundle != nullptr) {
    const auto it_bundle_template = view.bundle_templates().find(bundle->bundle_template_id);
    if (it_bundle_template != view.bundle_templates().end()) {
      bundle_template = &it_bundle_template->second;
    }
  }
  const CableTemplate* cable_template = nullptr;
  if (bundle_template != nullptr) {
    const auto it_cable = view.cable_templates().find(bundle_template->cable_template_id);
    if (it_cable != view.cable_templates().end()) {
      cable_template = &it_cable->second;
    }
  }

  const double sag_ratio = (cable_template == nullptr) ? view.geometry_settings().sag_factor
                                                       : (cable_template->sag_factor + cable_template->slack_factor);
  const bool use_reference_length = true;
  const ResolvedStyleContext style = resolve_style_context_for_span(state, span, StyleObjectKind::kSpan, 0, false);

  const SpanLayoutEntry* existing_layout = state.span_layout(span.id).entry;

  ResolvedSpanCurveInputs inputs{};
  inputs.basis_length =
      (use_reference_length && span.reference_length_m > 1e-9) ? span.reference_length_m : distance;
  if (existing_layout != nullptr) {
    inputs.flow_kind = existing_layout->flow_kind;
  } else {
    inputs.flow_kind = support_layout_flow_kind_for_span(span, port_a, port_b);
  }
  inputs.pass_mode = curve_pass_mode_from_context(span.placement_context);
  inputs.endpoint_mode =
      curve_endpoint_mode_for_attachment_style(resolve_effective_attachment_style(cable_template, style), bundle, bundle_template);
  inputs.continuity_preference =
      (cable_template == nullptr) ? CableContinuityPolicyHint::kAuto : cable_template->continuity_policy;
  inputs.bend_stiffness_hint = (cable_template == nullptr) ? 1.0 : cable_template->bend_stiffness;
  inputs.min_bend_radius_hint_m = (cable_template == nullptr) ? 0.0 : cable_template->min_bend_radius_m;
  inputs.endpoint_vertical_attachment_offset_m =
      (cable_template != nullptr && cable_template->requires_insulator)
          ? std::max(0.0, cable_template->insulator_attachment_height_m)
          : 0.0;

  const SpanRuntimeState* runtime = view.find_span_runtime_state(span.id);
  inputs.variation_flow_key = variation_flow_key_for_span(runtime, span);
  VariationContext sag_variation_context{};
  sag_variation_context.world_position = {(a.x + b.x) * 0.5, (a.y + b.y) * 0.5, (a.z + b.z) * 0.5};
  sag_variation_context.flow_key = inputs.variation_flow_key;
  sag_variation_context.pole_id = (pole_a == nullptr) ? kInvalidObjectId : pole_a->id;
  sag_variation_context.secondary_pole_id = (pole_b == nullptr) ? kInvalidObjectId : pole_b->id;
  sag_variation_context.local_key = static_cast<std::uint64_t>(span.id);
  inputs.sag_variation = EvaluateHierarchicalVariation(view.variation_settings(), sag_variation_context);
  const double sag_multiplier =
      std::max(0.0, 1.0 + inputs.sag_variation.final_value * view.variation_settings().sag_variation_scale);
  inputs.effective_sag_ratio =
      (view.geometry_settings().sag_enabled && inputs.basis_length > 1e-9) ? (sag_ratio * sag_multiplier) : 0.0;
  return inputs;
}

} // namespace wire::core
