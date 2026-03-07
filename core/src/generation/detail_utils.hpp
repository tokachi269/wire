#pragma once

#include "wire/core/core_state.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace wire::core::generation::detail {

constexpr double kPi = 3.14159265358979323846;
constexpr double kZeroLengthEps = 1e-9;

template <typename TValue>
void append_unique(std::vector<TValue>& dst, const std::vector<TValue>& src) {
  for (const TValue& value : src) {
    if (std::find(dst.begin(), dst.end(), value) == dst.end()) {
      dst.push_back(value);
    }
  }
}

inline void append_change_set(ChangeSet& dst, const ChangeSet& src) {
  append_unique(dst.created_ids, src.created_ids);
  append_unique(dst.updated_ids, src.updated_ids);
  append_unique(dst.deleted_ids, src.deleted_ids);
  append_unique(dst.dirty_span_ids, src.dirty_span_ids);
}

inline double normalize_yaw_deg(double yaw_deg) {
  double out = std::fmod(yaw_deg, 360.0);
  if (out <= -180.0) {
    out += 360.0;
  } else if (out > 180.0) {
    out -= 360.0;
  }
  return out;
}

inline bool normalize_xy(Vec3d* v) {
  if (v == nullptr) {
    return false;
  }
  const double len = std::sqrt(v->x * v->x + v->y * v->y);
  if (len <= 1e-9) {
    return false;
  }
  v->x /= len;
  v->y /= len;
  v->z = 0.0;
  return true;
}

inline double dot_xy(const Vec3d& a, const Vec3d& b) { return a.x * b.x + a.y * b.y; }

inline Vec3d rotate_xy_by_yaw_deg_local(const Vec3d& local, double yaw_deg) {
  const double rad = yaw_deg * (kPi / 180.0);
  const double c = std::cos(rad);
  const double s = std::sin(rad);
  return {local.x * c - local.y * s, local.x * s + local.y * c, local.z};
}

inline Vec3d rotate_x_deg_local(const Vec3d& v, double deg) {
  const double rad = deg * (kPi / 180.0);
  const double c = std::cos(rad);
  const double s = std::sin(rad);
  return {v.x, v.y * c - v.z * s, v.y * s + v.z * c};
}

inline Vec3d rotate_y_deg_local(const Vec3d& v, double deg) {
  const double rad = deg * (kPi / 180.0);
  const double c = std::cos(rad);
  const double s = std::sin(rad);
  return {v.x * c + v.z * s, v.y, -v.x * s + v.z * c};
}

inline Vec3d rotate_euler_xyz_deg_local(const Vec3d& local, const Vec3d& euler_deg) {
  const Vec3d rx = rotate_x_deg_local(local, euler_deg.x);
  const Vec3d ry = rotate_y_deg_local(rx, euler_deg.y);
  return rotate_xy_by_yaw_deg_local(ry, euler_deg.z);
}

inline Vec3d local_to_world_on_pole_local(const Transformd& tf, double yaw_deg, const Vec3d& local) {
  Vec3d euler = tf.rotation_euler_deg;
  euler.z = yaw_deg;
  return tf.position + rotate_euler_xyz_deg_local(local, euler);
}

inline double orient2d_xy_local(const Vec3d& a, const Vec3d& b, const Vec3d& c) {
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

inline bool xy_bbox_overlap_local(const Vec3d& a, const Vec3d& b, const Vec3d& c, const Vec3d& d) {
  const double min_ax = std::min(a.x, b.x);
  const double max_ax = std::max(a.x, b.x);
  const double min_ay = std::min(a.y, b.y);
  const double max_ay = std::max(a.y, b.y);
  const double min_cx = std::min(c.x, d.x);
  const double max_cx = std::max(c.x, d.x);
  const double min_cy = std::min(c.y, d.y);
  const double max_cy = std::max(c.y, d.y);
  return !(max_ax < min_cx || max_cx < min_ax || max_ay < min_cy || max_cy < min_ay);
}

inline bool segments_intersect_xy_strict_local(const Vec3d& a, const Vec3d& b, const Vec3d& c, const Vec3d& d) {
  if (!xy_bbox_overlap_local(a, b, c, d)) {
    return false;
  }
  constexpr double kIntersectEps = 1e-9;
  const double o1 = orient2d_xy_local(a, b, c);
  const double o2 = orient2d_xy_local(a, b, d);
  const double o3 = orient2d_xy_local(c, d, a);
  const double o4 = orient2d_xy_local(c, d, b);
  const bool ab_straddle_cd =
      ((o1 > kIntersectEps && o2 < -kIntersectEps) || (o1 < -kIntersectEps && o2 > kIntersectEps));
  const bool cd_straddle_ab =
      ((o3 > kIntersectEps && o4 < -kIntersectEps) || (o3 < -kIntersectEps && o4 > kIntersectEps));
  return ab_straddle_cd && cd_straddle_ab;
}

inline bool line_intersection_xy_local(const Vec3d& p, const Vec3d& r, const Vec3d& q, const Vec3d& s, Vec3d* out) {
  if (out == nullptr) {
    return false;
  }
  const double denom = r.x * s.y - r.y * s.x;
  if (std::abs(denom) <= 1e-9) {
    return false;
  }
  const Vec3d qp = q - p;
  const double t = (qp.x * s.y - qp.y * s.x) / denom;
  *out = {p.x + r.x * t, p.y + r.y * t, 0.0};
  return std::isfinite(out->x) && std::isfinite(out->y);
}

} // namespace wire::core::generation::detail
