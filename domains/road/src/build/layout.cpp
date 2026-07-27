#include "pipeline.hpp"

#include "geometry.hpp"
#include "read.hpp"

namespace city::road::build {
namespace {

Result<double> segment_length(const DerivedRoad &out, const ApproachKey &key) {
  const Path *alignment = find_alignment(out, key.segment_id);
  if (alignment == nullptr) {
    return Result<double>::Fail(ErrorKind::kInternal,
                                "road resolved layout alignment is missing");
  }
  return PathLength(*alignment);
}

double station_from_setback(const ApproachKey &key, double length_m,
                            double setback_m) {
  return key.endpoint_role == EndpointRole::kStart ? setback_m
                                                   : length_m - setback_m;
}

} // namespace

Result<bool> resolve_layouts(pipeline &pipe) {
  pipe.out.layouts.clear();
  for (const AutoNodeLayout &auto_layout : pipe.out.auto_layouts) {
    ResolvedNodeLayout layout{};
    layout.node_id = auto_layout.node_id;
    layout.kind = auto_layout.kind;
    layout.ordered_approaches = auto_layout.ordered_approaches;
    for (const AutoApproachLayout &auto_approach : auto_layout.approaches) {
      double setback = auto_approach.setback_m;
      double lateral_shift = auto_approach.lateral_shift_m;
      if (const ApproachGeometryOverride *override =
              find_approach_override(pipe.source, auto_approach.key)) {
        if (override->setback_m.has_value)
          setback = override->setback_m.value;
        if (override->lateral_shift_m.has_value)
          lateral_shift = override->lateral_shift_m.value;
      }
      const Result<double> length = segment_length(pipe.out, auto_approach.key);
      if (!length.ok)
        return Result<bool>::Fail(length.error_kind, length.error);
      if (!is_finite(setback) || setback < 0.0 ||
          setback > length.value + station_epsilon ||
          !is_finite(lateral_shift)) {
        return Result<bool>::Fail(
            ErrorKind::kUnsupported,
            "road approach override exceeds supported layout range");
      }
      const double station =
          station_from_setback(auto_approach.key, length.value, setback);
      const Path *alignment =
          find_alignment(pipe.out, auto_approach.key.segment_id);
      if (alignment == nullptr) {
        return Result<bool>::Fail(
            ErrorKind::kInternal,
            "road resolved layout source alignment is missing");
      }
      const Result<Vec2d> position = EvaluatePath(*alignment, station);
      const Result<Vec2d> path_tangent = tangent_at(*alignment, station);
      if (!position.ok || !path_tangent.ok) {
        return Result<bool>::Fail(
            ErrorKind::kInternal,
            "road resolved node layout frame could not be evaluated");
      }
      const Vec2d tangent =
          auto_approach.key.endpoint_role == EndpointRole::kStart
              ? path_tangent.value
              : scale(path_tangent.value, -1.0);
      const Vec2d lateral{-tangent.y, tangent.x};
      const Vec2d shifted = add(position.value, scale(lateral, lateral_shift));
      ConnectionGate gate{};
      gate.approach = auto_approach.key;
      gate.segment_id = auto_approach.key.segment_id;
      gate.node_id = auto_approach.key.node_id;
      gate.position = to3(shifted);
      gate.tangent = to3(tangent);
      gate.lateral = to3(lateral);
      gate.normal = Vec3d{0.0, 0.0, 1.0};
      layout.approaches.push_back(ResolvedApproachLayout{
          auto_approach.key,
          to3(shifted),
          to3(tangent),
          to3(lateral),
          Vec3d{0.0, 0.0, 1.0},
          setback,
          lateral_shift,
          station,
          gate,
      });
    }
    pipe.out.layouts.push_back(std::move(layout));
  }
  return Result<bool>::Ok(true);
}

} // namespace city::road::build
