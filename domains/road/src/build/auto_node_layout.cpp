#include "stages.hpp"

#include "stage_support.hpp"

namespace city::road::build {

Result<bool> BuildAutoNodeLayouts(BuildContext& context) {
  context.derived.auto_node_layouts.clear();
  for (const NodeConnectionDecision& decision :
       context.derived.node_connection_decisions) {
    if (decision.ordered_approaches.size() <= 1) continue;
    AutoNodeLayout layout{};
    layout.node_id = decision.node_id;
    layout.kind = decision.kind;
    layout.ordered_approaches = decision.ordered_approaches;
    for (const ApproachKey& key : decision.ordered_approaches) {
      const ApproachConnectionDecision* approach = FindApproachDecision(decision, key);
      const Path* alignment = FindAlignment(context.derived, key.segment_id);
      if (approach == nullptr || alignment == nullptr) {
        return Result<bool>::Fail(ErrorKind::kInternal, "road auto node layout input is missing");
      }
      const Result<Vec2d> position = EvaluatePath(*alignment, approach->gate_station_m);
      const Result<Vec2d> path_tangent = TangentAt(*alignment, approach->gate_station_m);
      if (!position.ok || !path_tangent.ok) {
        return Result<bool>::Fail(ErrorKind::kInternal, "road auto node layout frame could not be evaluated");
      }
      const Vec2d tangent = key.endpoint_role == EndpointRole::kStart
                                ? path_tangent.value
                                : Scale(path_tangent.value, -1.0);
      const Vec2d lateral{-tangent.y, tangent.x};
      ConnectionGate gate{};
      gate.approach = key;
      gate.segment_id = key.segment_id;
      gate.node_id = key.node_id;
      gate.position = To3(position.value);
      gate.tangent = To3(tangent);
      gate.lateral = To3(lateral);
      gate.normal = Vec3d{0.0, 0.0, 1.0};
      layout.approaches.push_back(AutoApproachLayout{
          key,
          To3(position.value),
          To3(tangent),
          To3(lateral),
          Vec3d{0.0, 0.0, 1.0},
          approach->setback_m,
          0.0,
          approach->gate_station_m,
          gate,
      });
    }
    context.derived.auto_node_layouts.push_back(std::move(layout));
  }
  return Result<bool>::Ok(true);
}

} // namespace city::road::build
