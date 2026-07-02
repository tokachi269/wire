#pragma once

#include "wire/core/core_state.hpp"
#include "wire/core/coord_utils.hpp"

namespace wire::core::generation::backbone {

struct SpanMemberPopulationEndpoint {
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

struct SpanMemberPopulationInput {
  SpanMemberKey key{};
  ExperimentalSpanMemberRule rule{};
  std::uint64_t explicit_seed = 1;
  SpanMemberPopulationEndpoint endpoint_a{};
  SpanMemberPopulationEndpoint endpoint_b{};
  std::vector<ExperimentalPlacementReserve> reserves{};
  std::vector<Vec3d> occupied_a_local{};
  std::vector<Vec3d> occupied_b_local{};
};

struct SpanMemberPopulationOutput {
  std::vector<SpanMemberLayout> members{};
  SpanMemberPopulationDiagnostic diagnostic{};
};

[[nodiscard]] bool has_duplicate_band_ids(const PoleTypeDefinition& pole_type);
[[nodiscard]] EditResult<SpanMemberPopulationOutput> populate_span_members(
    const SpanMemberPopulationInput& input);

struct ExperimentalSpanMemberPopulation {
  std::vector<SpanMemberLayout> members{};
  std::vector<SpanMemberPopulationDiagnostic> diagnostics{};
};

[[nodiscard]] ExperimentalSpanMemberPopulation make_experimental_span_members(
    const CoreState& state, const std::vector<SpanLayoutEntry>& layouts);

} // namespace wire::core::generation::backbone
