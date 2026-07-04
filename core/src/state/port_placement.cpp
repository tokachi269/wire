#include "port_placement.hpp"

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

} // namespace wire::core::state_internal
