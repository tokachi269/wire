#pragma once

namespace wire::core {

// Squared length tolerance in m^2 for deciding that a vector/segment has no usable direction.
inline constexpr double kLengthSquaredToleranceM2 = 1e-12;

// Length tolerance in meters for zero-length, positive-length, and distance comparisons.
inline constexpr double kLengthToleranceM = 1e-9;

// Strict length tolerance in meters for persisted/template coordinate equality.
inline constexpr double kStrictLengthToleranceM = 1e-12;

// Loose geometry tolerance in meters for merged intervals, band bounds, and visual fit margins.
inline constexpr double kGeometryToleranceM = 1e-6;

// Angle tolerance in degrees for threshold comparisons.
inline constexpr double kAngleToleranceDeg = 1e-6;

// Strict angle tolerance in degrees for persisted yaw/frame equality.
inline constexpr double kStrictAngleToleranceDeg = 1e-9;

// Unitless tolerance for dot products, normalized parameters, signs, and ratios.
inline constexpr double kUnitlessTolerance = 1e-9;

// Loose unitless tolerance for progress, alignment, and near-zero blend weights.
inline constexpr double kLooseUnitlessTolerance = 1e-6;

// Normalized route parameter tolerance.
inline constexpr double kParameterTolerance = 1e-6;

} // namespace wire::core
