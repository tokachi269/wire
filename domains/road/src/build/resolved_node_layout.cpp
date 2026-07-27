#include "stages.hpp"

#include "stage_support.hpp"

namespace city::road::build {
namespace {

Result<double> segment_length(const DerivedRoad& derived, const ApproachKey& key) {
  const Path* alignment = FindAlignment(derived, key.segment_id);
  if (alignment == nullptr) {
    return Result<double>::Fail(ErrorKind::kInternal, "road resolved layout alignment is missing");
  }
  return PathLength(*alignment);
}

double station_from_setback(const ApproachKey& key, double length_m, double setback_m) {
  return key.endpoint_role == EndpointRole::kStart ? setback_m : length_m - setback_m;
}

} // namespace

Result<bool> BuildResolvedNodeLayouts(BuildContext& context) {
  context.derived.resolved_node_layouts.clear();
  for (const AutoNodeLayout& auto_layout : context.derived.auto_node_layouts) {
    ResolvedNodeLayout layout{};
    layout.node_id = auto_layout.node_id;
    layout.kind = auto_layout.kind;
    layout.ordered_approaches = auto_layout.ordered_approaches;
    for (const AutoApproachLayout& auto_approach : auto_layout.approaches) {
      double setback = auto_approach.setback_m;
      double lateral_shift = auto_approach.lateral_shift_m;
      if (const ApproachGeometryOverride* override =
              FindApproachGeometryOverride(context.authoritative, auto_approach.key)) {
        if (override->setback_m.has_value) setback = override->setback_m.value;
        if (override->lateral_shift_m.has_value) lateral_shift = override->lateral_shift_m.value;
      }
      const Result<double> length = segment_length(context.derived, auto_approach.key);
      if (!length.ok) return Result<bool>::Fail(length.error_kind, length.error);
      if (!IsFinite(setback) || setback < 0.0 || setback > length.value + kStationEpsilon ||
          !IsFinite(lateral_shift)) {
        return Result<bool>::Fail(ErrorKind::kUnsupported, "road approach override exceeds supported layout range");
      }
      const double station = station_from_setback(auto_approach.key, length.value, setback);
      const Path* alignment = FindAlignment(context.derived, auto_approach.key.segment_id);
      if (alignment == nullptr) {
        return Result<bool>::Fail(ErrorKind::kInternal, "road resolved layout source alignment is missing");
      }
      const Result<Vec2d> position = EvaluatePath(*alignment, station);
      const Result<Vec2d> path_tangent = TangentAt(*alignment, station);
      if (!position.ok || !path_tangent.ok) {
        return Result<bool>::Fail(ErrorKind::kInternal, "road resolved node layout frame could not be evaluated");
      }
      const Vec2d tangent = auto_approach.key.endpoint_role == EndpointRole::kStart
                                ? path_tangent.value
                                : Scale(path_tangent.value, -1.0);
      const Vec2d lateral{-tangent.y, tangent.x};
      const Vec2d shifted = Add(position.value, Scale(lateral, lateral_shift));
      ConnectionGate gate{};
      gate.approach = auto_approach.key;
      gate.segment_id = auto_approach.key.segment_id;
      gate.node_id = auto_approach.key.node_id;
      gate.position = To3(shifted);
      gate.tangent = To3(tangent);
      gate.lateral = To3(lateral);
      gate.normal = Vec3d{0.0, 0.0, 1.0};
      layout.approaches.push_back(ResolvedApproachLayout{
          auto_approach.key,
          To3(shifted),
          To3(tangent),
          To3(lateral),
          Vec3d{0.0, 0.0, 1.0},
          setback,
          lateral_shift,
          station,
          gate,
      });
    }
    context.derived.resolved_node_layouts.push_back(std::move(layout));
  }
  return Result<bool>::Ok(true);
}

} // namespace city::road::build
