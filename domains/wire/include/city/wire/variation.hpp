#pragma once

#include <cstdint>

#include "city/wire/id.hpp"
#include "city/wire/types.hpp"

namespace city::wire {

struct VariationSettings {
  bool enabled = true;
  std::uint64_t global_seed = 1;
  double world_cell_size_m = 40.0;
  double world_bias_scale = 0.35;
  double flow_bias_scale = 0.40;
  double pole_delta_scale = 0.18;
  double local_jitter_scale = 0.07;
  double sag_variation_scale = 0.12;
  double branch_down_offset_variation_scale = 0.0;
};

struct VariationContext {
  Vec3d world_position{};
  std::uint64_t flow_key = 0;
  ObjectId pole_id = kInvalidObjectId;
  ObjectId secondary_pole_id = kInvalidObjectId;
  std::uint64_t local_key = 0;
};

struct HierarchicalVariationSample {
  double world_bias = 0.0;
  double flow_bias = 0.0;
  double pole_delta = 0.0;
  double local_jitter = 0.0;
  double final_value = 0.0;
  std::uint64_t flow_key = 0;
  ObjectId pole_id = kInvalidObjectId;
  ObjectId secondary_pole_id = kInvalidObjectId;
  std::uint64_t local_key = 0;
};

[[nodiscard]] HierarchicalVariationSample EvaluateHierarchicalVariation(const VariationSettings& settings,
                                                                        const VariationContext& context);

} // namespace city::wire
