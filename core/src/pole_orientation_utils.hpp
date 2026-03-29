#pragma once

#include <cstddef>
#include <vector>

#include "wire/core/entities.hpp"
#include "wire/core/types.hpp"

namespace wire::core {

struct SharpCornerOrientationDebug {
  bool applied = false;
  double theta_deg = 0.0;
  Vec3d bisector_dir{};
  Vec3d side_dir{};
};

struct AutoPoleTransformResult {
  Transformd transform{};
  SharpCornerOrientationDebug sharp{};
};

AutoPoleTransformResult compute_auto_pole_transform(const std::vector<Vec3d>& points, std::size_t index,
                                                    const Vec3d* preferred_side_dir = nullptr,
                                                    const Vec3d* base_rotation_euler_deg = nullptr);
Vec3d side_axis_from_yaw_deg(double yaw_deg);
void apply_sharp_debug_to_context(PoleContextInfo* context, const SharpCornerOrientationDebug& sharp);

} // namespace wire::core
