#include "stages.hpp"

#include "stage_support.hpp"

namespace city::road::build {

Result<bool> BuildCanonicalAlignments(BuildContext& context) {
  context.derived.canonical_alignments.clear();
  context.derived.canonical_alignments.reserve(context.authoritative.segments.size());
  for (const RoadSegment& segment : context.authoritative.segments) {
    const RoadNode* node_a = FindNode(context.authoritative, segment.node_a);
    const RoadNode* node_b = FindNode(context.authoritative, segment.node_b);
    if (node_a == nullptr || node_b == nullptr) {
      return Result<bool>::Fail(ErrorKind::kValidation, "road segment endpoint node is missing");
    }
    Result<Path> alignment = BuildCanonicalAlignment(node_a->position, node_b->position, segment.shape);
    if (!alignment.ok) return Result<bool>::Fail(alignment.error_kind, alignment.error);
    context.derived.canonical_alignments.push_back(
        CanonicalAlignment{segment.id, std::move(alignment.value)});
  }
  return Result<bool>::Ok(true);
}

} // namespace city::road::build
