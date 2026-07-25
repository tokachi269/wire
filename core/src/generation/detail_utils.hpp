#pragma once

#include "../collection_utils.hpp"
#include "wire/core/coord_utils.hpp"
#include "wire/core/support/numeric_tolerances.hpp"
#include "wire/core/core_edit_types.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace wire::core::generation::detail {

constexpr double kPi = 3.14159265358979323846;

using wire::core::detail::append_unique;

inline void append_change_set(ChangeSet& dst, const ChangeSet& src) {
  append_unique(dst.created_ids, src.created_ids);
  append_unique(dst.updated_ids, src.updated_ids);
  append_unique(dst.deleted_ids, src.deleted_ids);
}

inline bool normalize_xy(Vec3d* v) { return wire::core::NormalizeXY(v); }

inline double dot_xy(const Vec3d& a, const Vec3d& b) { return a.x * b.x + a.y * b.y; }

inline Vec3d local_to_world_on_pole_local(const Transformd& tf, double yaw_deg, const Vec3d& local) {
  return wire::core::LocalPointToWorld(wire::core::BuildPoleFrame(tf, yaw_deg), local);
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
  const double o1 = orient2d_xy_local(a, b, c);
  const double o2 = orient2d_xy_local(a, b, d);
  const double o3 = orient2d_xy_local(c, d, a);
  const double o4 = orient2d_xy_local(c, d, b);
  const bool ab_straddle_cd =
      ((o1 > kIntersectionTolerance && o2 < -kIntersectionTolerance) ||
       (o1 < -kIntersectionTolerance && o2 > kIntersectionTolerance));
  const bool cd_straddle_ab =
      ((o3 > kIntersectionTolerance && o4 < -kIntersectionTolerance) ||
       (o3 < -kIntersectionTolerance && o4 > kIntersectionTolerance));
  return ab_straddle_cd && cd_straddle_ab;
}

inline bool line_intersection_xy_local(const Vec3d& p, const Vec3d& r, const Vec3d& q, const Vec3d& s, Vec3d* out) {
  if (out == nullptr) {
    return false;
  }
  const double denom = r.x * s.y - r.y * s.x;
  if (std::abs(denom) <= kIntersectionTolerance) {
    return false;
  }
  const Vec3d qp = q - p;
  const double t = (qp.x * s.y - qp.y * s.x) / denom;
  *out = {p.x + r.x * t, p.y + r.y * t, 0.0};
  return std::isfinite(out->x) && std::isfinite(out->y);
}

} // namespace wire::core::generation::detail
