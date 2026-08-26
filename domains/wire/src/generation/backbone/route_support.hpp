#pragma once

#include "city/wire/entities.hpp"

#include <algorithm>
#include <cmath>

namespace city::wire::generation::backbone::route_support {

// Initial visual-evaluation values. They describe the derived non-HV support
// representation and are not engineering-standard dimensions.
constexpr double kDirectReachM = 0.18;
constexpr double kSupportedSlotStartM = 0.28;
constexpr double kSupportedSlotSpacingM = 0.12;
constexpr double kSupportedSlotJitterM = 0.012;
constexpr double kRowMergeDistanceM = 0.32;
constexpr double kMinimumSupportReachM = 0.34;
constexpr double kSupportReachMarginM = 0.08;

inline bool is_high_voltage(ConnectionCategory category) {
  return category == ConnectionCategory::kHighVoltage;
}

inline bool is_supported(ConnectionCategory category, double lateral_m) {
  return !is_high_voltage(category) && std::abs(lateral_m) > kDirectReachM;
}

inline int compatibility_family(ConnectionCategory category) {
  switch (category) {
  case ConnectionCategory::kLowVoltage:
    return 1;
  case ConnectionCategory::kCommunication:
  case ConnectionCategory::kOptical:
    return 2;
  case ConnectionCategory::kDrop:
    return 3;
  case ConnectionCategory::kHighVoltage:
    return 0;
  }
  return 0;
}

inline double support_reach(double lateral_m) {
  return std::max(kMinimumSupportReachM, std::abs(lateral_m) + kSupportReachMarginM);
}

} // namespace city::wire::generation::backbone::route_support
