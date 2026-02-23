#pragma once

namespace wire::core {

struct Vec3d {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

inline Vec3d operator+(const Vec3d& a, const Vec3d& b) {
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}

inline Vec3d operator-(const Vec3d& a, const Vec3d& b) {
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}

struct Transformd {
  Vec3d position{};
  Vec3d rotation_euler_deg{};
  Vec3d scale{1.0, 1.0, 1.0};
};

struct Frame3d {
  Vec3d origin{};
  Vec3d forward{1.0, 0.0, 0.0};
  Vec3d right{0.0, 1.0, 0.0};
  Vec3d up{0.0, 0.0, 1.0};
};

struct AABBd {
  Vec3d min{};
  Vec3d max{};
};

}  // namespace wire::core
