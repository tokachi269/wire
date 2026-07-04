#pragma once

#include "wire/core/core_state.hpp"

namespace wire::core::state_internal {

[[nodiscard]] const PortPlacementBand* FindPortPlacementBandById(
    const PoleTypeDefinition& pole_type, int placement_band_id);

[[nodiscard]] const PortPlacementBand* FindPortPlacementBandForPort(
    const CoreState& state, const PoleTypeDefinition& pole_type, const Port& port);

[[nodiscard]] SlotSide inner_side_for_turn(double turn_sign);
[[nodiscard]] double apply_corner_side_scale(
    double local_y, SlotSide slot_side, double turn_sign, double side_scale);

} // namespace wire::core::state_internal
