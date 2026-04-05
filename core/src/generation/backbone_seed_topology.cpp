#include "backbone_seed_topology.hpp"

namespace wire::core::generation::detail {

void apply_seed_topology_projection(const EndpointContinuityDecision& source, ObjectId owner_pole_id,
                                    SupportLayoutSemanticDecision* decision) {
  if (decision == nullptr) {
    return;
  }
  decision->owner_pole_id = owner_pole_id;
  decision->relation_kind = source.relation_kind;
  decision->continuity_class = source.continuity_class;
  decision->in_through_pair = source.in_through_pair;
  decision->support_pair_peer_low = source.support_pair_peer_low;
  decision->support_pair_peer_high = source.support_pair_peer_high;
}

} // namespace wire::core::generation::detail
