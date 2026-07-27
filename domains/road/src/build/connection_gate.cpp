#include "stages.hpp"

#include "stage_support.hpp"

namespace city::road::build {

Result<bool> BuildConnectionGates(BuildContext& context) {
  context.derived.connection_gates.clear();
  for (const ResolvedNodeLayout& layout :
       context.derived.resolved_node_layouts) {
    for (const ResolvedApproachLayout& approach : layout.approaches) {
      const SectionEvaluation* section = FindSectionEvaluation(
          context.derived, approach.key.segment_id, approach.gate_station_m);
      if (section == nullptr) {
        return Result<bool>::Fail(
            ErrorKind::kInternal,
            "road connection gate has no unique SectionEvaluation");
      }
      ConnectionGate gate = approach.gate;
      gate.boundaries = section->boundaries;
      context.derived.connection_gates.push_back(std::move(gate));
    }
  }
  return Result<bool>::Ok(true);
}

} // namespace city::road::build
