#include "wire/core/variation.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace wire::core {
namespace {

constexpr std::uint64_t kWorldSalt = 0x9E3779B97F4A7C15ull;
constexpr std::uint64_t kFlowSalt = 0xBF58476D1CE4E5B9ull;
constexpr std::uint64_t kPoleSalt = 0x94D049BB133111EBull;
constexpr std::uint64_t kLocalSalt = 0xD6E8FEB86659FD93ull;

std::uint64_t splitmix64(std::uint64_t x) {
  x += 0x9E3779B97F4A7C15ull;
  x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ull;
  x = (x ^ (x >> 27)) * 0x94D049BB133111EBull;
  return x ^ (x >> 31);
}

std::uint64_t hash_combine(std::uint64_t seed, std::uint64_t value) {
  return splitmix64(seed ^ (value + 0x9E3779B97F4A7C15ull + (seed << 6) + (seed >> 2)));
}

std::uint64_t mix_double_bits(double value) {
  static_assert(sizeof(double) == sizeof(std::uint64_t));
  std::uint64_t bits = 0;
  std::memcpy(&bits, &value, sizeof(double));
  return splitmix64(bits);
}

double unit_from_hash(std::uint64_t seed) {
  const std::uint64_t bits = splitmix64(seed);
  constexpr double kInv = 1.0 / static_cast<double>(1ull << 53);
  const double u = static_cast<double>((bits >> 11) & ((1ull << 53) - 1ull)) * kInv;
  return u * 2.0 - 1.0;
}

double smoothstep(double t) {
  t = std::max(0.0, std::min(1.0, t));
  return t * t * (3.0 - 2.0 * t);
}

double lerp(double a, double b, double t) {
  return a + (b - a) * t;
}

double world_noise_2d(std::uint64_t global_seed, double x, double y, double cell_size_m) {
  const double safe_cell = std::max(1e-3, cell_size_m);
  const double sx = x / safe_cell;
  const double sy = y / safe_cell;
  const double ix = std::floor(sx);
  const double iy = std::floor(sy);
  const double fx = smoothstep(sx - ix);
  const double fy = smoothstep(sy - iy);

  auto corner = [&](double gx, double gy) {
    std::uint64_t key = hash_combine(global_seed ^ kWorldSalt, mix_double_bits(gx));
    key = hash_combine(key, mix_double_bits(gy));
    return unit_from_hash(key);
  };

  const double v00 = corner(ix, iy);
  const double v10 = corner(ix + 1.0, iy);
  const double v01 = corner(ix, iy + 1.0);
  const double v11 = corner(ix + 1.0, iy + 1.0);
  const double vx0 = lerp(v00, v10, fx);
  const double vx1 = lerp(v01, v11, fx);
  return lerp(vx0, vx1, fy);
}

double pole_noise(std::uint64_t global_seed, ObjectId pole_a, ObjectId pole_b) {
  if (pole_a == kInvalidObjectId && pole_b == kInvalidObjectId) {
    return 0.0;
  }
  auto sample = [&](ObjectId pole_id) -> double {
    if (pole_id == kInvalidObjectId) {
      return 0.0;
    }
    return unit_from_hash(hash_combine(global_seed ^ kPoleSalt, static_cast<std::uint64_t>(pole_id)));
  };
  if (pole_a != kInvalidObjectId && pole_b != kInvalidObjectId) {
    return 0.5 * (sample(pole_a) + sample(pole_b));
  }
  return sample((pole_a != kInvalidObjectId) ? pole_a : pole_b);
}

} // namespace

HierarchicalVariationSample EvaluateHierarchicalVariation(const VariationSettings& settings,
                                                          const VariationContext& context) {
  HierarchicalVariationSample sample{};
  sample.flow_key = context.flow_key;
  sample.pole_id = context.pole_id;
  sample.secondary_pole_id = context.secondary_pole_id;
  sample.local_key = context.local_key;
  if (!settings.enabled) {
    return sample;
  }

  sample.world_bias = world_noise_2d(settings.global_seed, context.world_position.x, context.world_position.y,
                                     settings.world_cell_size_m) *
                      settings.world_bias_scale;
  sample.flow_bias =
      unit_from_hash(hash_combine(settings.global_seed ^ kFlowSalt, context.flow_key)) * settings.flow_bias_scale;
  sample.pole_delta = pole_noise(settings.global_seed, context.pole_id, context.secondary_pole_id) *
                      settings.pole_delta_scale;
  sample.local_jitter =
      unit_from_hash(hash_combine(settings.global_seed ^ kLocalSalt, context.local_key)) * settings.local_jitter_scale;
  sample.final_value =
      std::max(-1.0, std::min(1.0, sample.world_bias + sample.flow_bias + sample.pole_delta + sample.local_jitter));
  return sample;
}

} // namespace wire::core
