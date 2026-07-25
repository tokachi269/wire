#include "port_placement.hpp"

#include "wire/core/core_state.hpp"
#include "wire/core/support/numeric_tolerances.hpp"
#include "wire/core/core_view.hpp"

namespace wire::core::state_internal {

const PortPlacementBand* FindPortPlacementBandById(
    const PoleTypeDefinition& pole_type, int placement_band_id) {
  for (const PortPlacementBand& band : pole_type.port_bands) {
    if (band.band_id == placement_band_id) {
      return &band;
    }
  }
  return nullptr;
}

const PortPlacementBand* FindPortPlacementBandForPort(
    const CoreState& state, const PoleTypeDefinition& pole_type, const Port& port) {
  if (const SavedBackbonePortBinding* binding = state.view().backbone_port_binding_for_port(port.id);
      binding != nullptr) {
    return FindPortPlacementBandById(pole_type, binding->placement_band_id);
  }

  const PortPlacementBand* best = nullptr;
  for (const PortPlacementBand& band : pole_type.port_bands) {
    if (!band.enabled || band.category != port.category || band.layer != port.template_layer ||
        band.side != port.template_side || band.role != port.template_role) {
      continue;
    }
    if (best == nullptr || band.priority > best->priority ||
        (band.priority == best->priority && band.band_id < best->band_id)) {
      best = &band;
    }
  }
  return best;
}

SlotSide inner_side_for_turn(double turn_sign) {
  if (turn_sign > kLengthToleranceM) {
    return SlotSide::kLeft;
  }
  if (turn_sign < -kLengthToleranceM) {
    return SlotSide::kRight;
  }
  return SlotSide::kCenter;
}

double apply_corner_side_scale(
    double local_y, SlotSide slot_side, double turn_sign, double side_scale) {
  if (slot_side == SlotSide::kCenter) {
    return local_y;
  }
  const double inner_scale = 1.0 + (side_scale - 1.0) * 0.35;
  const SlotSide inner_side = inner_side_for_turn(turn_sign);
  if (inner_side == SlotSide::kCenter) {
    return local_y * side_scale;
  }
  if (slot_side == inner_side) {
    return local_y * inner_scale;
  }
  return local_y * side_scale;
}

} // namespace wire::core::state_internal
