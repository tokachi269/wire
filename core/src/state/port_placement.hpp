#pragma once

#include "wire/core/core_state.hpp"

namespace wire::core::state_internal {

[[nodiscard]] const PortPlacementBand* FindPortPlacementBandById(
    const PoleTypeDefinition& pole_type, int placement_band_id);

[[nodiscard]] const PortPlacementBand* FindPortPlacementBandForPort(
    const CoreState& state, const PoleTypeDefinition& pole_type, const Port& port);

} // namespace wire::core::state_internal
