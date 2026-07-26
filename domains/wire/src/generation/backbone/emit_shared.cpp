#include "emit_shared.hpp"

#include "city/wire/coord_utils.hpp"
#include "city/wire/support/numeric_tolerances.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace city::wire::generation::backbone {

PortKind PortKindForCategory(ConnectionCategory category) {
  switch (category) {
  case ConnectionCategory::kCommunication:
  case ConnectionCategory::kOptical:
    return PortKind::kCommunication;
  case ConnectionCategory::kHighVoltage:
  case ConnectionCategory::kLowVoltage:
  case ConnectionCategory::kDrop:
  default:
    return PortKind::kPower;
  }
}

PortLayer PortLayerForSpanLayer(SpanLayer layer) {
  switch (layer) {
  case SpanLayer::kHighVoltage:
    return PortLayer::kHighVoltage;
  case SpanLayer::kLowVoltage:
    return PortLayer::kLowVoltage;
  case SpanLayer::kCommunication:
    return PortLayer::kCommunication;
  case SpanLayer::kOptical:
    return PortLayer::kOptical;
  case SpanLayer::kDrop:
    return PortLayer::kDrop;
  case SpanLayer::kUnknown:
  default:
    return PortLayer::kUnknown;
  }
}

SpanKind SpanKindForCategory(ConnectionCategory category) {
  return category == ConnectionCategory::kDrop ? SpanKind::kService : SpanKind::kDistribution;
}

int TemplateLayerForSpanLayer(SpanLayer layer) {
  switch (layer) {
  case SpanLayer::kHighVoltage:
    return 2;
  case SpanLayer::kDrop:
    return 0;
  case SpanLayer::kLowVoltage:
  case SpanLayer::kCommunication:
  case SpanLayer::kOptical:
  case SpanLayer::kUnknown:
  default:
    return 1;
  }
}

double LaneOffset(std::size_t lane_index, int count, double spacing_m) {
  return (static_cast<double>(lane_index) - (static_cast<double>(count - 1) * 0.5)) * spacing_m;
}

double PortLayoutYawDeg(const Vec3d& row_axis) {
  // R5 fallback: upstream row-axis inconsistency candidate. Supported rows should validate non-zero axes earlier.
  const Vec3d axis = HorizontalNormalizedOr(row_axis);
  return YawDegFromXY(ScaleVec(ComputeLateralAxis(axis), -1.0));
}

EditResult<std::vector<PortPlacementBand>> SelectPortPlacementBands(const PoleTypeDefinition& pole_type,
                                                                    ConnectionCategory category, SpanLayer layer,
                                                                    int lane_count) {
  EditResult<std::vector<PortPlacementBand>> out{};
  if (lane_count <= 0) {
    out.error = "backbone unsupported: bundle count resolved to zero";
    return out;
  }
  const int target_rank = TemplateLayerForSpanLayer(layer);
  std::vector<PortPlacementBand> candidates{};
  for (const PortPlacementBand& band : pole_type.port_bands) {
    if (band.enabled && band.category == category && band.layer == target_rank) {
      candidates.push_back(band);
    }
  }
  if (candidates.empty()) {
    out.error = "backbone unsupported: port band missing";
    return out;
  }
  std::sort(candidates.begin(), candidates.end(), [](const PortPlacementBand& a, const PortPlacementBand& b) {
    if (a.priority != b.priority) {
      return a.priority > b.priority;
    }
    return a.band_id < b.band_id;
  });
  std::vector<PortPlacementBand> distinct{};
  for (const PortPlacementBand& candidate : candidates) {
    const bool same_position = std::any_of(distinct.begin(), distinct.end(), [&](const PortPlacementBand& selected) {
      return std::abs(selected.lateral_center_m - candidate.lateral_center_m) <= kStrictLengthToleranceM;
    });
    if (!same_position) {
      distinct.push_back(candidate);
    }
    if (distinct.size() == static_cast<std::size_t>(lane_count)) {
      break;
    }
  }
  if (lane_count > 1 && distinct.size() == static_cast<std::size_t>(lane_count)) {
    std::sort(distinct.begin(), distinct.end(), [](const PortPlacementBand& a, const PortPlacementBand& b) {
      if (std::abs(a.lateral_center_m - b.lateral_center_m) > kStrictLengthToleranceM) {
        return a.lateral_center_m < b.lateral_center_m;
      }
      return a.band_id < b.band_id;
    });
    out.value = std::move(distinct);
  } else {
    out.value.assign(static_cast<std::size_t>(lane_count), candidates.front());
  }
  out.ok = true;
  return out;
}

Vec3d PortLocalPosition(const Vec3d& row_axis, const PortPlacementBand& band, double lane_offset_m,
                        double lateral_offset_m, const Vec3d& shift) {
  // R5 fallback: upstream row-axis inconsistency candidate. This mirrors PortLayoutYawDeg's row-axis contract.
  const Vec3d axis = HorizontalNormalizedOr(row_axis);
  const Vec3d forward_axis = ScaleVec(ComputeLateralAxis(axis), -1.0);
  const double offset = band.lateral_center_m + lane_offset_m + lateral_offset_m;
  return {Dot(shift, forward_axis), offset + Dot(shift, axis), band.height_center_m + shift.z};
}

Vec3d PortWorldPosition(const Pole& pole, const Vec3d& row_axis, const PortPlacementBand& band,
                        double lane_offset_m, double lateral_offset_m, const Vec3d& shift) {
  // R5 fallback: upstream row-axis inconsistency candidate. This path should not own row-axis validation.
  const Vec3d axis = HorizontalNormalizedOr(row_axis);
  const double layout_yaw_deg = PortLayoutYawDeg(axis);
  return LocalPointToWorld(BuildPoleFrame(pole.world_transform, layout_yaw_deg),
                           PortLocalPosition(axis, band, lane_offset_m, lateral_offset_m, shift));
}

Vec3d PortWorldPositionForLayoutYaw(const Pole& pole, double layout_yaw_deg, const PortPlacementBand& band,
                                    double lane_offset_m, double lateral_offset_m, double height_offset_m) {
  return LocalPointToWorld(BuildPoleFrame(pole.world_transform, layout_yaw_deg),
                           {0.0, band.lateral_center_m + lane_offset_m + lateral_offset_m,
                            band.height_center_m + height_offset_m});
}

void ApplyPortBandTemplateFields(Port* port, const PortPlacementBand& band) {
  if (port == nullptr) {
    return;
  }
  port->template_layer = band.layer;
  port->template_side = band.side;
  port->template_role = band.role;
}

} // namespace city::wire::generation::backbone
