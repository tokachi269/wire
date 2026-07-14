#pragma once

#include "wire/core/core_state.hpp"

namespace wire::core::generation::backbone {

// Resolves the generic endpoint fixture attached to a backbone Port. Ports without
// an endpoint assembly keep their authoritative position unchanged.
[[nodiscard]] EditResult<Vec3d> resolve_model_assembly_wire_socket(const CoreState& state,
                                                                   const Port& port);

// Produces generic model instances from Pole, saved row, and Port ownership.
// The result is derived runtime output and never creates topology objects.
[[nodiscard]] EditResult<VisualModelInstanceCache> materialize_model_assemblies(
    const CoreState& state);

} // namespace wire::core::generation::backbone
