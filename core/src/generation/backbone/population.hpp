#pragma once

#include "wire/core/core_state.hpp"
#include "wire/core/coord_utils.hpp"

namespace wire::core::generation::backbone {

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
  CableInstanceKey key{};
  CablePopulationRule rule{};
  CablePopulationEndpoint endpoint_a{};
  CablePopulationEndpoint endpoint_b{};
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

} // namespace wire::core::generation::backbone
