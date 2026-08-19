#pragma once

#include "city/wire/core_state.hpp"

#include <vector>

namespace city::wire::generation::backbone {

PortKind PortKindForCategory(ConnectionCategory category);
PortLayer PortLayerForSpanLayer(SpanLayer layer);
SpanKind SpanKindForCategory(ConnectionCategory category);
int TemplateLayerForSpanLayer(SpanLayer layer);

double LaneOffset(std::size_t lane_index, int count, double spacing_m);
double PortLayoutYawDeg(const Vec3d& row_axis);
EditResult<bool> PermutableLaneMirror(const Vec3d& row_direction_a,
                                      const Vec3d& row_direction_b,
                                      const Vec3d& span_forward);

EditResult<std::vector<PortPlacementBand>> SelectPortPlacementBands(const PoleTypeDefinition& pole_type,
                                                                    ConnectionCategory category, SpanLayer layer,
                                                                    int lane_count);

Vec3d PortLocalPosition(const Vec3d& row_axis, const PortPlacementBand& band, double lane_offset_m,
                        double lateral_offset_m, const Vec3d& shift);

Vec3d PortWorldPosition(const Pole& pole, const Vec3d& row_axis, const PortPlacementBand& band,
                        double lane_offset_m, double lateral_offset_m, const Vec3d& shift);

Vec3d PortWorldPositionForLayoutYaw(const Pole& pole, double layout_yaw_deg, const PortPlacementBand& band,
                                    double lane_offset_m, double lateral_offset_m, double height_offset_m);

void ApplyPortBandTemplateFields(Port* port, const PortPlacementBand& band);

} // namespace city::wire::generation::backbone
