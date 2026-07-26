#include "derive_span_layout.hpp"

#include "city/wire/coord_utils.hpp"

#include "out.hpp"
#include "model_assembly.hpp"

#include <algorithm>
#include <optional>

namespace city::wire::generation::backbone {

EditResult<Vec3d> resolve_span_layout_endpoint(const CoreState& state, const EditState& edit_state,
                                                const EndpointLayoutRule& rule,
                                                const FixturePlacementPlanByPort* fixture_plan) {
  EditResult<Vec3d> out{};
  const Port* port = edit_state.ports.find(rule.port_id);
  if (port == nullptr) {
    out.error = "backbone internal: backbone layout: endpoint port not found";
    return out;
  }
  if (rule.source_projection.valid()) {
    const std::optional<Vec3d> projection = source_edge_projection_world(state, rule.source_projection);
    if (!projection.has_value()) {
      out.error = "backbone internal: backbone layout: source edge projection missing";
      return out;
    }
    out.value = *projection;
    out.ok = true;
    return out;
  }
  if (fixture_plan != nullptr) {
    const auto plan_it = fixture_plan->find(rule.port_id);
    if (plan_it != fixture_plan->end()) {
      out.value = plan_it->second.wire_endpoint;
      out.ok = true;
      return out;
    }
  }
  return resolve_model_assembly_wire_socket(state, *port, endpoint_down_offset(rule));
}

EditResult<SpanLayoutEntry> derive_span_layout(const SpanLayoutRule& rule,
                                                const span_layout_endpoint_resolver& endpoint_resolver,
                                                std::uint64_t source_version) {
  EditResult<SpanLayoutEntry> out{};
  const EditResult<Vec3d> start_world = endpoint_resolver(rule.start);
  if (!start_world.ok) {
    out.error = start_world.error;
    return out;
  }
  const EditResult<Vec3d> end_world = endpoint_resolver(rule.end);
  if (!end_world.ok) {
    out.error = end_world.error;
    return out;
  }

  SpanLayoutEntry entry{};
  entry.span_id = rule.span_id;
  entry.flow_kind = rule.flow_kind;
  entry.pass_mode = rule.pass_mode;
  entry.variation_flow_key = rule.variation_flow_key;
  entry.lowering_kind = rule.lowering_kind;
  ApplyEndpointLayoutRule(entry.start, rule.start, start_world.value);
  ApplyEndpointLayoutRule(entry.end, rule.end, end_world.value);
  const auto append_group_key = [&](const LayoutEndpoint& endpoint) {
    if (!UsesAuthoritativeGroupedLoweredSupport(endpoint)) {
      return;
    }
    const LoweredSupportGroupKey key = LoweredSupportGroupKeyFromDecision(endpoint);
    if (std::find(entry.lowered_support_group_keys.begin(), entry.lowered_support_group_keys.end(), key) ==
        entry.lowered_support_group_keys.end()) {
      entry.lowered_support_group_keys.push_back(key);
    }
  };
  append_group_key(entry.start);
  append_group_key(entry.end);
  entry.basis_length_m = Length(entry.end.endpoint_world - entry.start.endpoint_world);
  entry.effective_sag_ratio = 0.0;
  entry.continuity_preference = CableContinuityPolicyHint::kAuto;
  entry.bend_stiffness_hint = 1.0;
  entry.source_version = source_version;
  out.value = std::move(entry);
  out.ok = true;
  return out;
}

} // namespace city::wire::generation::backbone
