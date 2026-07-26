#include "stages.hpp"

#include "stage_support.hpp"

namespace city::road::build {

Result<bool> BuildTopologyIndex(BuildContext& context) {
  context.topology.clear();
  context.topology.reserve(context.authoritative.nodes.size());
  for (const RoadNode& node : context.authoritative.nodes) {
    NodeTopology topology{};
    topology.node_id = node.id;
    for (const RoadSegment& segment : context.authoritative.segments) {
      if (segment.node_a == node.id || segment.node_b == node.id) {
        topology.endpoints.push_back(TopologyEndpoint{
            segment.id,
            segment.node_a == node.id ? EndpointRole::kStart
                                      : EndpointRole::kEnd,
        });
      }
    }
    context.topology.push_back(std::move(topology));
  }
  return Result<bool>::Ok(true);
}

} // namespace city::road::build
