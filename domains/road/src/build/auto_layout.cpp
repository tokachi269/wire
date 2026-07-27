#include "pipeline.hpp"

#include "geometry.hpp"
#include "read.hpp"

namespace city::road::build {

Result<bool> make_auto_layouts(pipeline &pipe) {
  pipe.out.auto_layouts.clear();
  for (const NodeConnectionDecision &decision : pipe.out.decisions) {
    if (decision.ordered_approaches.size() <= 1)
      continue;
    AutoNodeLayout layout{};
    layout.node_id = decision.node_id;
    layout.kind = decision.kind;
    layout.ordered_approaches = decision.ordered_approaches;
    for (const ApproachKey &key : decision.ordered_approaches) {
      const ApproachConnectionDecision *approach =
          find_approach_connection(decision, key);
      const Path *alignment = find_alignment(pipe.out, key.segment_id);
      if (approach == nullptr || alignment == nullptr) {
        return Result<bool>::Fail(ErrorKind::kInternal,
                                  "road auto node layout input is missing");
      }
      const Result<Vec2d> position =
          EvaluatePath(*alignment, approach->gate_station_m);
      const Result<Vec2d> path_tangent =
          tangent_at(*alignment, approach->gate_station_m);
      if (!position.ok || !path_tangent.ok) {
        return Result<bool>::Fail(
            ErrorKind::kInternal,
            "road auto node layout frame could not be evaluated");
      }
      const Vec2d tangent = key.endpoint_role == EndpointRole::kStart
                                ? path_tangent.value
                                : scale(path_tangent.value, -1.0);
      const Vec2d lateral{-tangent.y, tangent.x};
      ConnectionGate gate{};
      gate.approach = key;
      gate.segment_id = key.segment_id;
      gate.node_id = key.node_id;
      gate.position = to3(position.value);
      gate.tangent = to3(tangent);
      gate.lateral = to3(lateral);
      gate.normal = Vec3d{0.0, 0.0, 1.0};
      layout.approaches.push_back(AutoApproachLayout{
          key,
          to3(position.value),
          to3(tangent),
          to3(lateral),
          Vec3d{0.0, 0.0, 1.0},
          approach->setback_m,
          0.0,
          approach->gate_station_m,
          gate,
      });
    }
    pipe.out.auto_layouts.push_back(std::move(layout));
  }
  return Result<bool>::Ok(true);
}

} // namespace city::road::build
