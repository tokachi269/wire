#pragma once

#include "wire/core/entities.hpp"

namespace wire::core::generation::detail {

inline double BranchDownOffsetForCategory(ConnectionCategory category) {
  switch (category) {
  case ConnectionCategory::kHighVoltage:
    return 0.275;
  case ConnectionCategory::kCommunication:
    return 0.30;
  case ConnectionCategory::kOptical:
    return 0.24;
  case ConnectionCategory::kLowVoltage:
  case ConnectionCategory::kDrop:
  default:
    return 0.35;
  }
}

inline SpanKind DefaultSpanKindForCategory(ConnectionCategory category) {
  switch (category) {
  case ConnectionCategory::kDrop:
    return SpanKind::kService;
  case ConnectionCategory::kHighVoltage:
  case ConnectionCategory::kLowVoltage:
  case ConnectionCategory::kCommunication:
  case ConnectionCategory::kOptical:
  default:
    return SpanKind::kDistribution;
  }
}

inline int TemplateLayerForCategory(ConnectionCategory category) {
  switch (category) {
  case ConnectionCategory::kHighVoltage:
    return 2;
  case ConnectionCategory::kDrop:
    return 0;
  case ConnectionCategory::kLowVoltage:
  case ConnectionCategory::kCommunication:
  case ConnectionCategory::kOptical:
  default:
    return 1;
  }
}

} // namespace wire::core::generation::detail
