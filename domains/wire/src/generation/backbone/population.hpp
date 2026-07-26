#pragma once

#include "city/wire/core_state.hpp"
#include "city/wire/coord_utils.hpp"

namespace city::wire::generation::backbone {

struct CablePopulationEndpoint {
  bool valid = false;
  std::string failure_reason{};
  PoleTypeId pole_type_id = kInvalidPoleTypeId;
  int band_id = 0;
  PoleFrame frame{};
  Vec3d original_local{};
  Vec3d endpoint_offset_world{};
  double lateral_min_m = 0.0;
  double lateral_max_m = 0.0;
  double height_min_m = 0.0;
  double height_max_m = 0.0;
};

struct CablePopulationInput {
  CableSectionKey key{};
  CablePopulationRule rule{};
  CablePopulationEndpoint endpoint_a{};
  CablePopulationEndpoint endpoint_b{};
  // Carrier endpoints of the base span. Wrap sections attach to these instead of solving a
  // band placement of their own.
  Vec3d endpoint_a_world{};
  Vec3d endpoint_b_world{};
  std::vector<Vec3d> occupied_a_local{};
  std::vector<Vec3d> occupied_b_local{};
};

struct CablePopulationOutput {
  std::vector<CableSectionLayout> sections{};
  CablePopulationDiagnostic diagnostic{};
};

[[nodiscard]] bool has_duplicate_band_ids(const PoleTypeDefinition& pole_type);
[[nodiscard]] EditResult<CablePopulationOutput> populate_cable_sections(
    const CablePopulationInput& input);

struct CablePopulation {
  std::vector<CableSectionLayout> sections{};
  std::vector<CablePopulationDiagnostic> diagnostics{};
};

[[nodiscard]] CablePopulation make_cable_population(
    const CoreState& state, const std::vector<SpanLayoutEntry>& layouts);

} // namespace city::wire::generation::backbone
