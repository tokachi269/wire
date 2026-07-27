#include "pipeline.hpp"

#include "geometry.hpp"
#include "read.hpp"

namespace city::road::build {

Result<bool> make_alignments(pipeline &pipe) {
  pipe.out.alignments.clear();
  pipe.out.alignments.reserve(pipe.source.segments.size());
  for (const RoadSegment &segment : pipe.source.segments) {
    const RoadNode *node_a = find_node(pipe.source, segment.node_a);
    const RoadNode *node_b = find_node(pipe.source, segment.node_b);
    if (node_a == nullptr || node_b == nullptr) {
      return Result<bool>::Fail(ErrorKind::kValidation,
                                "road segment endpoint node is missing");
    }
    Result<Path> alignment = BuildCanonicalAlignment(
        node_a->position, node_b->position, segment.shape);
    if (!alignment.ok)
      return Result<bool>::Fail(alignment.error_kind, alignment.error);
    pipe.out.alignments.push_back(
        CanonicalAlignment{segment.id, std::move(alignment.value)});
  }
  return Result<bool>::Ok(true);
}

} // namespace city::road::build
