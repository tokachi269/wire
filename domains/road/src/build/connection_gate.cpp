#include "stages.hpp"

#include "stage_support.hpp"

namespace city::road::build {

Result<bool> BuildConnectionGates(BuildContext& context) {
  context.derived.connection_gates.clear();
  for (const NodeConnectionDecision& decision :
       context.derived.node_connection_decisions) {
    for (const ApproachKey& key : decision.ordered_approaches) {
      const ApproachConnectionDecision* approach =
          FindApproachDecision(decision, key);
      const RoadSegment* segment =
          FindSegment(context.authoritative, key.segment_id);
      const Path* alignment = FindAlignment(context.derived, key.segment_id);
      if (approach == nullptr || segment == nullptr || alignment == nullptr) {
        return Result<bool>::Fail(ErrorKind::kInternal,
                                  "road connection gate input is missing");
      }
      const Result<Vec2d> position =
          EvaluatePath(*alignment, approach->gate_station_m);
      const Result<Vec2d> path_tangent =
          TangentAt(*alignment, approach->gate_station_m);
      if (!position.ok || !path_tangent.ok) {
        return Result<bool>::Fail(ErrorKind::kInternal,
                                  "road connection gate frame could not be evaluated");
      }
      const Vec2d tangent =
          key.endpoint_role == EndpointRole::kStart
              ? path_tangent.value
              : Scale(path_tangent.value, -1.0);
      const Vec2d lateral{-tangent.y, tangent.x};
      const SectionEvaluation* section = FindSectionEvaluation(
          context.derived, key.segment_id, approach->gate_station_m);
      if (section == nullptr) {
        return Result<bool>::Fail(
            ErrorKind::kInternal,
            "road connection gate has no unique SectionEvaluation");
      }
      context.derived.connection_gates.push_back(ConnectionGate{
          key,
          key.segment_id,
          key.node_id,
          To3(position.value),
          To3(tangent),
          To3(lateral),
          Vec3d{0.0, 0.0, 1.0},
          section->boundaries,
      });
    }
  }
  return Result<bool>::Ok(true);
}

} // namespace city::road::build
