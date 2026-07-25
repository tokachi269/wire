#pragma once

#include <algorithm>
#include <cstddef>
#include <cmath>
#include <vector>

#include "wire/core/types.hpp"
#include "wire/core/numeric_tolerances.hpp"

namespace wire::core {

struct PoleFrame {
  Vec3d origin{};
  Vec3d forward{1.0, 0.0, 0.0};
  Vec3d lateral{0.0, 1.0, 0.0};
  Vec3d up{0.0, 0.0, 1.0};
  Vec3d euler_deg{};
};

inline Vec3d WorldUp() { return {0.0, 0.0, 1.0}; }

inline Vec3d WorldForward() { return {1.0, 0.0, 0.0}; }

inline Vec3d WorldLateral() { return {0.0, 1.0, 0.0}; }

inline Vec3d ScaleVec(const Vec3d& v, double scale) { return {v.x * scale, v.y * scale, v.z * scale}; }

inline double Dot(const Vec3d& a, const Vec3d& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

inline Vec3d Cross(const Vec3d& a, const Vec3d& b) {
  return {
      a.y * b.z - a.z * b.y,
      a.z * b.x - a.x * b.z,
      a.x * b.y - a.y * b.x,
  };
}

inline double LengthSquared(const Vec3d& v) { return Dot(v, v); }

inline double Length(const Vec3d& v) { return std::sqrt(LengthSquared(v)); }

inline double DistanceSquared(const Vec3d& a, const Vec3d& b) { return LengthSquared(a - b); }

inline double DistanceSquaredXY(const Vec3d& a, const Vec3d& b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return dx * dx + dy * dy;
}

inline double DistanceSquaredToSegment(const Vec3d& point, const Vec3d& a, const Vec3d& b) {
  const Vec3d ab = b - a;
  const double len2 = LengthSquared(ab);
  if (len2 <= kLengthSquaredToleranceM2) {
    return DistanceSquared(point, a);
  }
  const double t = std::clamp(Dot(point - a, ab) / len2, 0.0, 1.0);
  return DistanceSquared(point, a + ScaleVec(ab, t));
}

inline bool Normalize(Vec3d* v) {
  if (v == nullptr) {
    return false;
  }
  const double len2 = LengthSquared(*v);
  if (len2 <= kLengthSquaredToleranceM2) {
    return false;
  }
  const double inv_len = 1.0 / std::sqrt(len2);
  v->x *= inv_len;
  v->y *= inv_len;
  v->z *= inv_len;
  return true;
}

inline bool NormalizeXY(Vec3d* v) {
  if (!Normalize(v)) {
    return false;
  }
  v->z = 0.0;
  return Normalize(v);
}

inline Vec3d HorizontalNormalizedOr(Vec3d v, Vec3d fallback = WorldForward()) {
  v.z = 0.0;
  if (Normalize(&v)) {
    return v;
  }
  fallback.z = 0.0;
  return Normalize(&fallback) ? fallback : WorldForward();
}

inline double NormalizeYawDeg(double yaw_deg) {
  double out = std::fmod(yaw_deg, 360.0);
  if (out <= -180.0) {
    out += 360.0;
  } else if (out > 180.0) {
    out -= 360.0;
  }
  return out;
}

inline double YawDegFromXY(Vec3d forward) {
  forward = HorizontalNormalizedOr(forward);
  return std::atan2(forward.y, forward.x) * (180.0 / 3.14159265358979323846);
}

inline double PolylineLength(const std::vector<Vec3d>& polyline) {
  double total = 0.0;
  for (std::size_t i = 0; i + 1 < polyline.size(); ++i) {
    total += Length(polyline[i + 1] - polyline[i]);
  }
  return total;
}

inline Vec3d ComputeLateralAxis(const Vec3d& forward) {
  Vec3d lateral = Cross(WorldUp(), forward);
  if (!Normalize(&lateral)) {
    return {};
  }
  return lateral;
}

inline double HeightAlongWorldUp(const Vec3d& world) { return Dot(world, WorldUp()); }

inline void SetHeightAlongWorldUp(Vec3d* world, double height) {
  if (world == nullptr) {
    return;
  }
  world->z = height;
}

inline void OffsetAlongWorldUp(Vec3d* world, double offset) {
  if (world == nullptr) {
    return;
  }
  world->z += offset;
}

inline Vec3d RotateAroundWorldUpDeg(const Vec3d& local, double yaw_deg) {
  constexpr double kPi = 3.14159265358979323846;
  const double rad = yaw_deg * (kPi / 180.0);
  const double c = std::cos(rad);
  const double s = std::sin(rad);
  return {local.x * c - local.y * s, local.x * s + local.y * c, local.z};
}

inline Vec3d RotateAroundAxisDeg(const Vec3d& value, Vec3d axis, double angle_deg) {
  if (!Normalize(&axis)) {
    return value;
  }
  constexpr double kPi = 3.14159265358979323846;
  const double angle = angle_deg * (kPi / 180.0);
  const double cosine = std::cos(angle);
  const double sine = std::sin(angle);
  return ScaleVec(value, cosine) + ScaleVec(Cross(axis, value), sine) +
         ScaleVec(axis, Dot(axis, value) * (1.0 - cosine));
}

inline Vec3d RotateXDeg(const Vec3d& v, double deg) {
  constexpr double kPi = 3.14159265358979323846;
  const double rad = deg * (kPi / 180.0);
  const double c = std::cos(rad);
  const double s = std::sin(rad);
  return {v.x, v.y * c - v.z * s, v.y * s + v.z * c};
}

inline Vec3d RotateYDeg(const Vec3d& v, double deg) {
  constexpr double kPi = 3.14159265358979323846;
  const double rad = deg * (kPi / 180.0);
  const double c = std::cos(rad);
  const double s = std::sin(rad);
  return {v.x * c + v.z * s, v.y, -v.x * s + v.z * c};
}

inline Vec3d RotateEulerXYZDeg(const Vec3d& local, const Vec3d& euler_deg) {
  const Vec3d rx = RotateXDeg(local, euler_deg.x);
  const Vec3d ry = RotateYDeg(rx, euler_deg.y);
  return RotateAroundWorldUpDeg(ry, euler_deg.z);
}

inline Vec3d InverseRotateEulerXYZDeg(const Vec3d& world_delta, const Vec3d& euler_deg) {
  const Vec3d rz = RotateAroundWorldUpDeg(world_delta, -euler_deg.z);
  const Vec3d ry = RotateYDeg(rz, -euler_deg.y);
  return RotateXDeg(ry, -euler_deg.x);
}

inline PoleFrame BuildPoleFrame(const Transformd& transform, double layout_yaw_deg) {
  PoleFrame frame{};
  frame.origin = transform.position;
  frame.euler_deg = transform.rotation_euler_deg;
  // Tilt is an instance-owned physical direction. Layout yaw may vary per port,
  // so it only rotates the local placement axes around the already tilted pole.
  frame.forward = RotateEulerXYZDeg(WorldForward(), transform.rotation_euler_deg);
  frame.lateral = RotateEulerXYZDeg(WorldLateral(), transform.rotation_euler_deg);
  frame.up = RotateEulerXYZDeg(WorldUp(), transform.rotation_euler_deg);
  const double layout_delta_deg = layout_yaw_deg - transform.rotation_euler_deg.z;
  frame.forward = RotateAroundAxisDeg(frame.forward, frame.up, layout_delta_deg);
  frame.lateral = RotateAroundAxisDeg(frame.lateral, frame.up, layout_delta_deg);
  frame.euler_deg.z = layout_yaw_deg;
  Normalize(&frame.forward);
  Normalize(&frame.lateral);
  Normalize(&frame.up);
  return frame;
}

inline Vec3d LocalPointToWorld(const PoleFrame& frame, const Vec3d& local) {
  return frame.origin + ScaleVec(frame.forward, local.x) + ScaleVec(frame.lateral, local.y) +
         ScaleVec(frame.up, local.z);
}

inline Vec3d WorldPointToLocal(const PoleFrame& frame, const Vec3d& world) {
  const Vec3d delta = world - frame.origin;
  return {
      Dot(delta, frame.forward),
      Dot(delta, frame.lateral),
      Dot(delta, frame.up),
  };
}

} // namespace wire::core
