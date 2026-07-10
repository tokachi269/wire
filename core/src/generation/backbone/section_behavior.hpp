#pragma once

#include "population.hpp"

namespace wire::core::generation::backbone {

struct behavior_curve_part_input {
  const CoreState& state;
  const CableSectionLayout& section;
  const DetailCurve& carrier;
  ObjectId edge_id = kInvalidObjectId;
  ObjectId bundle_id = kInvalidObjectId;
  BundleTemplateId bundle_template_id = kInvalidBundleTemplateId;
  std::size_t lane_index = 0;
  CableRunId cable_run_id = 0;
  std::uint64_t source_version = 0;
};

[[nodiscard]] std::string validate_population_rule_behavior(const CablePopulationRule& rule);
[[nodiscard]] std::vector<CableSectionLayout> make_behavior_sections(const CablePopulationInput& input,
                                                                       int requested_count);
[[nodiscard]] bool participates_in_node_patch(const CableSectionLayout& section);
[[nodiscard]] EditResult<VisualCurvePart> build_behavior_curve_part(const behavior_curve_part_input& input);

} // namespace wire::core::generation::backbone
