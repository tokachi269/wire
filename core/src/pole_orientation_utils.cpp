#include "pole_orientation_utils.hpp"

#include <algorithm>
#include <cmath>

#include "wire/core/coord_utils.hpp"

namespace wire::core {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kSharpCornerInteriorAngleMaxDeg = 74.0;

Vec3d project_onto_plane(const Vec3d& value, const Vec3d& plane_up) {
  return value - ScaleVec(plane_up, Dot(value, plane_up));
}

bool normalize_on_plane(Vec3d* value, const Vec3d& plane_up) {
  if (value == nullptr) {
    return false;
  }
  *value = project_onto_plane(*value, plane_up);
  return Normalize(value);
}

Vec3d plane_lateral_axis(const Vec3d& plane_up, const Vec3d& forward) {
  Vec3d lateral = Cross(plane_up, forward);
  if (!Normalize(&lateral)) {
    return {};
  }
  return lateral;
}

double signed_turn_about_axis(const Vec3d& axis, const Vec3d& a, const Vec3d& b) {
  return Dot(Cross(a, b), axis);
}

double yaw_deg_for_direction_in_tilted_frame(const Vec3d& world_dir, const Vec3d* base_rotation_euler_deg,
                                             bool align_lateral_axis) {
  double yaw_deg = std::atan2(world_dir.y, world_dir.x) * (180.0 / kPi);
  if (base_rotation_euler_deg != nullptr) {
    Transformd base{};
    base.rotation_euler_deg = *base_rotation_euler_deg;
    base.rotation_euler_deg.z = 0.0;
    const PoleFrame frame = BuildPoleFrame(base, 0.0);
    const Vec3d local_dir{
        Dot(world_dir, frame.forward),
        Dot(world_dir, frame.lateral),
        Dot(world_dir, frame.up),
    };
    yaw_deg = std::atan2(local_dir.y, local_dir.x) * (180.0 / kPi);
  }
  if (align_lateral_axis) {
    yaw_deg -= 90.0;
  }
  return NormalizeYawDeg(yaw_deg);
}

SharpCornerOrientationDebug compute_sharp_corner_orientation(const std::vector<Vec3d>& points, std::size_t index,
                                                             double sharp_threshold_deg,
                                                             const Vec3d* preferred_side_dir,
                                                             const Vec3d& plane_up) {
  SharpCornerOrientationDebug out{};
  if (points.size() < 3 || index == 0 || index + 1 >= points.size()) {
    return out;
  }

  Vec3d u0 = points[index - 1] - points[index];
  Vec3d u1 = points[index + 1] - points[index];
  if (!normalize_on_plane(&u0, plane_up) || !normalize_on_plane(&u1, plane_up)) {
    return out;
  }

  double d = Dot(u0, u1);
  d = std::clamp(d, -1.0, 1.0);
  out.theta_deg = std::acos(d) * (180.0 / kPi);
  if (out.theta_deg <= 1e-6 || out.theta_deg > sharp_threshold_deg + 1e-6) {
    return out;
  }

  Vec3d bisector = u0 + u1;
  if (!normalize_on_plane(&bisector, plane_up)) {
    return out;
  }
  out.bisector_dir = bisector;

  Vec3d side1 = plane_lateral_axis(plane_up, bisector);
  if (!Normalize(&side1)) {
    return out;
  }
  Vec3d side2 = ScaleVec(side1, -1.0);
  Vec3d selected = side1;
  Vec3d inward{};
  bool has_inward = false;

  Vec3d t_in = points[index] - points[index - 1];
  Vec3d t_out = points[index + 1] - points[index];
  if (!normalize_on_plane(&t_in, plane_up) || !normalize_on_plane(&t_out, plane_up)) {
    return out;
  }
  const double turn = signed_turn_about_axis(plane_up, t_in, t_out);
  if (std::abs(turn) > 1e-9) {
    inward = plane_lateral_axis(plane_up, t_in);
    if (turn < 0.0) {
      inward = ScaleVec(inward, -1.0);
    }
    if (Normalize(&inward)) {
      has_inward = true;
      const double inward_proj_1 = Dot(side1, inward);
      const double inward_proj_2 = Dot(side2, inward);
      selected = (inward_proj_1 <= inward_proj_2) ? side1 : side2;
    }
  }

  if (preferred_side_dir != nullptr) {
    Vec3d preferred = *preferred_side_dir;
    if (normalize_on_plane(&preferred, plane_up)) {
      auto side_score = [&](const Vec3d& side) -> double {
        const double inward_penalty = has_inward ? std::max(0.0, Dot(side, inward)) : 0.0;
        const double continuity_penalty = 0.5 * (1.0 - Dot(side, preferred));
        return inward_penalty * 4.0 + continuity_penalty;
      };
      const Vec3d alternate = ScaleVec(selected, -1.0);
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
                                                    const Vec3d* preferred_side_dir,
                                                    const Vec3d* base_rotation_euler_deg) {
  AutoPoleTransformResult out{};
  out.transform.position = points[index];
  if (base_rotation_euler_deg != nullptr) {
    out.transform.rotation_euler_deg.x = base_rotation_euler_deg->x;
    out.transform.rotation_euler_deg.y = base_rotation_euler_deg->y;
  }

  Vec3d plane_up = WorldUp();
  if (base_rotation_euler_deg != nullptr) {
    Vec3d base_euler = *base_rotation_euler_deg;
    base_euler.z = 0.0;
    plane_up = RotateEulerXYZDeg(WorldUp(), base_euler);
    Normalize(&plane_up);
  }

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

  if (normalize_on_plane(&tangent, plane_up)) {
    double yaw_deg = yaw_deg_for_direction_in_tilted_frame(tangent, base_rotation_euler_deg, false);
    out.sharp =
        compute_sharp_corner_orientation(points, index, kSharpCornerInteriorAngleMaxDeg, preferred_side_dir, plane_up);
    if (out.sharp.applied) {
      yaw_deg = yaw_deg_for_direction_in_tilted_frame(out.sharp.side_dir, base_rotation_euler_deg, true);
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
