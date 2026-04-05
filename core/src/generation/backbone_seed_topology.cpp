#include "backbone_seed_topology.hpp"

namespace wire::core::generation::detail {

SeedDecisionTopologyProjection build_seed_topology_projection(const EndpointContinuityDecision& decision,
                                                              ObjectId owner_pole_id) {
  SeedDecisionTopologyProjection projection{};
  projection.owner_pole_id = owner_pole_id;
  projection.relation_kind = decision.relation_kind;
  projection.continuity_class = decision.continuity_class;
  projection.in_through_pair = decision.in_through_pair;
  projection.support_pair_peer_low = decision.support_pair_peer_low;
  projection.support_pair_peer_high = decision.support_pair_peer_high;
  return projection;
}

void apply_seed_topology_projection(const SeedDecisionTopologyProjection& projection,
                                    EndpointContinuityDecision* decision) {
  if (decision == nullptr) {
    return;
  }
  decision->owner_pole_id = projection.owner_pole_id;
  decision->relation_kind = projection.relation_kind;
  decision->continuity_class = projection.continuity_class;
  decision->in_through_pair = projection.in_through_pair;
  decision->support_pair_peer_low = projection.support_pair_peer_low;
  decision->support_pair_peer_high = projection.support_pair_peer_high;
}

} // namespace wire::core::generation::detail
