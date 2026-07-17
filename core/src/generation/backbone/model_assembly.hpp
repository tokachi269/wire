#pragma once

#include "wire/core/core_state.hpp"

#include <unordered_map>
#include <vector>

namespace wire::core::generation::backbone {

struct ResolvedEndpointPlacement {
  Transformd fixture_root{};
  Vec3d wire_endpoint{};
};

struct FixturePlacementPlan {
  double down_offset_m = 0.0;
  Transformd endpoint_fixture_root{};
  Vec3d wire_endpoint{};
};

using FixturePlacementPlanByPort = std::unordered_map<ObjectId, FixturePlacementPlan>;

[[nodiscard]] double endpoint_down_offset(const EndpointLayoutRule& endpoint);

// Resolves the endpoint-fixture root and its exported wire point from one
// placement chain. A row fixture may contribute position through its explicit
// endpoint mount socket; endpoint orientation remains the Port PoleFrame.
[[nodiscard]] EditResult<ResolvedEndpointPlacement> resolve_endpoint_placement(
    const CoreState& state, const Port& port, double down_offset_m = 0.0);

// Resolves the generic endpoint fixture attached to a backbone Port. Ports without
// an endpoint assembly keep their authoritative position unchanged.
[[nodiscard]] EditResult<Vec3d> resolve_model_assembly_wire_socket(const CoreState& state,
                                                                   const Port& port,
                                                                   double down_offset_m = 0.0);

[[nodiscard]] EditResult<FixturePlacementPlanByPort> fixture_placement_plan_from_rules(
    const CoreState& state, const std::vector<SpanLayoutRule>& rules);

[[nodiscard]] EditResult<FixturePlacementPlanByPort> fixture_placement_plan_from_cache(
    const CoreState& state);

// Produces generic model instances from Pole, saved row, and Port ownership.
// The result is derived runtime output and never creates topology objects.
[[nodiscard]] EditResult<VisualModelInstanceCache> materialize_model_assemblies(
    const CoreState& state, const FixturePlacementPlanByPort* fixture_plan = nullptr);

[[nodiscard]] EditResult<VisualModelInstanceCache> materialize_model_assemblies(
    const CoreState& state, const std::vector<SpanLayoutRule>& rules);

} // namespace wire::core::generation::backbone
