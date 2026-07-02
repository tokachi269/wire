#pragma once

#include "wire/core/core_state.hpp"
#include "wire/core/coord_utils.hpp"

namespace wire::core::generation::backbone {

struct PhysicalLinePopulationEndpoint {
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

struct PhysicalLinePopulationInput {
  PhysicalLineInstanceKey key{};
  ExperimentalPhysicalLineRule rule{};
  std::uint64_t explicit_seed = 1;
  PhysicalLinePopulationEndpoint endpoint_a{};
  PhysicalLinePopulationEndpoint endpoint_b{};
  std::vector<ExperimentalPlacementReserve> reserves{};
  std::vector<Vec3d> occupied_a_local{};
  std::vector<Vec3d> occupied_b_local{};
};

struct PhysicalLinePopulationOutput {
  std::vector<PhysicalLineInstance> instances{};
  PhysicalLinePopulationDiagnostic diagnostic{};
};

[[nodiscard]] bool has_duplicate_band_ids(const PoleTypeDefinition& pole_type);
[[nodiscard]] EditResult<PhysicalLinePopulationOutput> populate_physical_lines(
    const PhysicalLinePopulationInput& input);
void append_experimental_physical_lines(const CoreState& state, const std::vector<SpanLayoutEntry>& layouts,
                                        VisualCurvePartCache* output);

} // namespace wire::core::generation::backbone
