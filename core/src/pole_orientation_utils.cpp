#include "pole_orientation_utils.hpp"

#include <algorithm>
#include <cmath>

#include "wire/core/coord_utils.hpp"

namespace wire::core {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kSharpCornerInteriorAngleMaxDeg = 75.0;

double dot_xy(const Vec3d& a, const Vec3d& b) { return a.x * b.x + a.y * b.y; }

SharpCornerOrientationDebug compute_sharp_corner_orientation(const std::vector<Vec3d>& points, std::size_t index,
                                                             double sharp_threshold_deg,
                                                             const Vec3d* preferred_side_dir_xy) {
  SharpCornerOrientationDebug out{};
  if (points.size() < 3 || index == 0 || index + 1 >= points.size()) {
    return out;
  }

  Vec3d u0{
      points[index - 1].x - points[index].x,
      points[index - 1].y - points[index].y,
      0.0,
  };
  Vec3d u1{
      points[index + 1].x - points[index].x,
      points[index + 1].y - points[index].y,
      0.0,
  };
  if (!NormalizeXY(&u0) || !NormalizeXY(&u1)) {
    return out;
  }

  double d = dot_xy(u0, u1);
  d = std::clamp(d, -1.0, 1.0);
  out.theta_deg = std::acos(d) * (180.0 / kPi);
  if (out.theta_deg <= 1e-6 || out.theta_deg > sharp_threshold_deg + 1e-6) {
    return out;
  }

  Vec3d bisector{
      u0.x + u1.x,
      u0.y + u1.y,
      0.0,
  };
  if (!NormalizeXY(&bisector)) {
    return out;
  }
  out.bisector_dir = bisector;

  Vec3d side1 = ComputeLateralAxis(bisector);
  if (!NormalizeXY(&side1)) {
    return out;
  }
  Vec3d side2{-side1.x, -side1.y, 0.0};
  Vec3d selected = side1;
  Vec3d inward{};
  bool has_inward = false;

  Vec3d t_in{
      points[index].x - points[index - 1].x,
      points[index].y - points[index - 1].y,
      0.0,
  };
  Vec3d t_out{
      points[index + 1].x - points[index].x,
      points[index + 1].y - points[index].y,
      0.0,
  };
  if (!NormalizeXY(&t_in) || !NormalizeXY(&t_out)) {
    return out;
  }
  const double turn = t_in.x * t_out.y - t_in.y * t_out.x;
  if (std::abs(turn) > 1e-9) {
    inward = (turn > 0.0) ? ComputeLateralAxis(t_in) : Vec3d{-ComputeLateralAxis(t_in).x, -ComputeLateralAxis(t_in).y, 0.0};
    if (NormalizeXY(&inward)) {
      has_inward = true;
      const double inward_proj_1 = dot_xy(side1, inward);
      const double inward_proj_2 = dot_xy(side2, inward);
      selected = (inward_proj_1 <= inward_proj_2) ? side1 : side2;
    }
  }

  if (preferred_side_dir_xy != nullptr) {
    Vec3d preferred = *preferred_side_dir_xy;
    if (NormalizeXY(&preferred)) {
      auto side_score = [&](const Vec3d& side) -> double {
        const double inward_penalty = has_inward ? std::max(0.0, dot_xy(side, inward)) : 0.0;
        const double continuity_penalty = 0.5 * (1.0 - dot_xy(side, preferred));
        return inward_penalty * 4.0 + continuity_penalty;
      };
      const Vec3d alternate{-selected.x, -selected.y, 0.0};
      if (side_score(alternate) + 1e-9 < side_score(selected)) {
        selected = alternate;
      }
    }
  }

  out.side_dir = selected;
  out.applied = true;
  return out;
}

} // namespace

AutoPoleTransformResult compute_auto_pole_transform(const std::vector<Vec3d>& points, std::size_t index,
                                                    const Vec3d* preferred_side_dir_xy) {
  AutoPoleTransformResult out{};
  out.transform.position = points[index];

  Vec3d tangent{};
  if (points.size() >= 2) {
    if (index == 0) {
      tangent = points[1] - points[0];
    } else if (index + 1 >= points.size()) {
      tangent = points[index] - points[index - 1];
    } else {
      tangent = points[index + 1] - points[index - 1];
    }
  }

  const double len2 = tangent.x * tangent.x + tangent.y * tangent.y + tangent.z * tangent.z;
  if (len2 > 1e-12) {
    double yaw_deg = std::atan2(tangent.y, tangent.x) * (180.0 / kPi);
    out.sharp =
        compute_sharp_corner_orientation(points, index, kSharpCornerInteriorAngleMaxDeg, preferred_side_dir_xy);
    if (out.sharp.applied) {
      yaw_deg = std::atan2(out.sharp.side_dir.y, out.sharp.side_dir.x) * (180.0 / kPi) - 90.0;
    }
    out.transform.rotation_euler_deg.z = NormalizeYawDeg(yaw_deg);
  }
  return out;
}

Vec3d side_axis_from_yaw_deg(double yaw_deg) {
  return ComputeLateralAxis(RotateAroundWorldUpDeg(WorldForward(), yaw_deg));
}

void apply_sharp_debug_to_context(PoleContextInfo* context, const SharpCornerOrientationDebug& sharp) {
  if (context == nullptr) {
    return;
  }
  context->sharp_orientation_applied = sharp.applied;
  context->sharp_theta_deg = sharp.theta_deg;
  context->sharp_bisector_dir = sharp.bisector_dir;
  context->sharp_side_dir = sharp.side_dir;
}

} // namespace wire::core
