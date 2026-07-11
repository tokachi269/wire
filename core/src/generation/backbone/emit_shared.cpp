#include "emit_shared.hpp"

#include "wire/core/coord_utils.hpp"

namespace wire::core::generation::backbone {

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
  const Vec3d axis = HorizontalNormalizedOr(row_axis);
  return YawDegFromXY(ScaleVec(ComputeLateralAxis(axis), -1.0));
}

EditResult<PortPlacementBand> SelectPortPlacementBand(const PoleTypeDefinition& pole_type, ConnectionCategory category,
                                                      SpanLayer layer) {
  EditResult<PortPlacementBand> out{};
  const int target_rank = TemplateLayerForSpanLayer(layer);
  const PortPlacementBand* best = nullptr;
  for (const PortPlacementBand& band : pole_type.port_bands) {
    if (!band.enabled || band.category != category || band.layer != target_rank) {
      continue;
    }
    if (best == nullptr || band.priority > best->priority ||
        (band.priority == best->priority && band.band_id < best->band_id)) {
      best = &band;
    }
  }
  if (best == nullptr) {
    out.error = "backbone unsupported: port band missing";
    return out;
  }
  out.value = *best;
  out.ok = true;
  return out;
}

Vec3d PortLocalPosition(const Vec3d& row_axis, const PortPlacementBand& band, std::size_t lane_index, int lane_count,
                        double spacing_m, double lateral_offset_m, const Vec3d& shift) {
  const Vec3d axis = HorizontalNormalizedOr(row_axis);
  const Vec3d forward_axis = ScaleVec(ComputeLateralAxis(axis), -1.0);
  const double offset = LaneOffset(lane_index, lane_count, spacing_m) + lateral_offset_m;
  return {Dot(shift, forward_axis), offset + Dot(shift, axis), band.height_center_m + shift.z};
}

Vec3d PortWorldPosition(const Pole& pole, const Vec3d& row_axis, const PortPlacementBand& band,
                        std::size_t lane_index, int lane_count, double spacing_m, double lateral_offset_m,
                        const Vec3d& shift) {
  const Vec3d axis = HorizontalNormalizedOr(row_axis);
  const double layout_yaw_deg = PortLayoutYawDeg(axis);
  return LocalPointToWorld(BuildPoleFrame(pole.world_transform, layout_yaw_deg),
                           PortLocalPosition(axis, band, lane_index, lane_count, spacing_m, lateral_offset_m, shift));
}

void ApplyPortBandTemplateFields(Port* port, const PortPlacementBand& band) {
  if (port == nullptr) {
    return;
  }
  port->template_layer = band.layer;
  port->template_side = band.side;
  port->template_role = band.role;
}

} // namespace wire::core::generation::backbone
