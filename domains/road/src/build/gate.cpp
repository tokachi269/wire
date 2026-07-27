#include "pipeline.hpp"

#include "geometry.hpp"
#include "read.hpp"

namespace city::road::build {

Result<bool> make_gates(pipeline &pipe) {
  pipe.out.gates.clear();
  for (const ResolvedNodeLayout &layout : pipe.out.layouts) {
    for (const ResolvedApproachLayout &approach : layout.approaches) {
      const SectionEvaluation *section = find_section(
          pipe.out, approach.key.segment_id, approach.gate_station_m);
      if (section == nullptr) {
        return Result<bool>::Fail(
            ErrorKind::kInternal,
            "road connection gate has no unique SectionEvaluation");
      }
      ConnectionGate gate = approach.gate;
      gate.boundaries = section->boundaries;
      pipe.out.gates.push_back(std::move(gate));
    }
  }
  return Result<bool>::Ok(true);
}

} // namespace city::road::build
