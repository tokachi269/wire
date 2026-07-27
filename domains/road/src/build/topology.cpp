#include "pipeline.hpp"

#include "geometry.hpp"
#include "read.hpp"

namespace city::road::build {

Result<bool> make_topology(pipeline &pipe) {
  pipe.nodes.clear();
  pipe.nodes.reserve(pipe.source.nodes.size());
  for (const RoadNode &node : pipe.source.nodes) {
    topology item{};
    item.node_id = node.id;
    for (const RoadSegment &segment : pipe.source.segments) {
      if (segment.node_a == node.id || segment.node_b == node.id) {
        item.endpoints.push_back(endpoint{
            segment.id,
            segment.node_a == node.id ? EndpointRole::kStart
                                      : EndpointRole::kEnd,
        });
      }
    }
    pipe.nodes.push_back(std::move(item));
  }
  return Result<bool>::Ok(true);
}

} // namespace city::road::build
