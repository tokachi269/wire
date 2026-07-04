#pragma once

#include "wire/core/core_state.hpp"

namespace wire::core::generation::backbone {

PortKind PortKindForCategory(ConnectionCategory category);
PortLayer PortLayerForSpanLayer(SpanLayer layer);
SpanKind SpanKindForCategory(ConnectionCategory category);
int TemplateLayerForSpanLayer(SpanLayer layer);

double LaneOffset(std::size_t lane_index, int count, double spacing_m);
double BundleGroupOffset(std::size_t bundle_index, std::size_t bundle_count, int lane_count, double spacing_m);
double PortLayoutYawDeg(const Vec3d& row_axis);

EditResult<PortPlacementBand> SelectPortPlacementBand(const PoleTypeDefinition& pole_type, ConnectionCategory category,
                                                      SpanLayer layer);

Vec3d PortLocalPosition(const Vec3d& row_axis, const PortPlacementBand& band, std::size_t lane_index, int lane_count,
                        double spacing_m, double group_offset_m, double lateral_offset_m, const Vec3d& shift);

Vec3d PortWorldPosition(const Pole& pole, const Vec3d& row_axis, const PortPlacementBand& band,
                        std::size_t lane_index, int lane_count, double spacing_m, double group_offset_m,
                        double lateral_offset_m, const Vec3d& shift);

void ApplyPortBandTemplateFields(Port* port, const PortPlacementBand& band);

} // namespace wire::core::generation::backbone
