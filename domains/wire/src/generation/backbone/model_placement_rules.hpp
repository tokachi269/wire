#pragma once

#include "city/wire/core_state.hpp"

#include <cstdint>
#include <vector>

namespace city::wire::generation::backbone {

enum class ModelPlacementRuleKind : std::uint8_t {
  kAtRow,
  kAtEndpoint,
  kInterval,
};

enum class ModelPlacementOrientationPolicy : std::uint8_t {
  kWorldUp,
  kAlignTangent,
};

struct ModelPlacementRule {
  ModelPlacementRuleKind kind = ModelPlacementRuleKind::kAtEndpoint;
  ModelAssemblyTemplateId assembly_id = kInvalidModelAssemblyTemplateId;
  double spacing_m = 0.0;
  double phase_m = 0.0;
  ModelPlacementOrientationPolicy orientation = ModelPlacementOrientationPolicy::kWorldUp;
};

struct SpanAnchorFrame {
  Transformd transform{};
  double t = 0.0;
};

[[nodiscard]] std::vector<ModelPlacementRule> placement_rules_from_bundle_template(
    const BundleTemplate& bundle_template);

[[nodiscard]] std::vector<SpanAnchorFrame> interval_anchor_frames(const Vec3d& start,
                                                                  const Vec3d& end,
                                                                  double spacing_m,
                                                                  double phase_m,
                                                                  ModelPlacementOrientationPolicy orientation);

} // namespace city::wire::generation::backbone
