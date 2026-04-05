#pragma once

#include "grouped_span_common.hpp"

namespace wire::core::generation::detail {

struct SeedDecisionTopologyProjection {
  ObjectId owner_pole_id = kInvalidObjectId;
  JunctionRelationKind relation_kind = JunctionRelationKind::kNone;
  ContinuityCategoryClass continuity_class = ContinuityCategoryClass::kPointLike;
  bool in_through_pair = false;
  ObjectId support_pair_peer_low = kInvalidObjectId;
  ObjectId support_pair_peer_high = kInvalidObjectId;
};

[[nodiscard]] SeedDecisionTopologyProjection build_seed_topology_projection(const EndpointContinuityDecision& decision,
                                                                            ObjectId owner_pole_id);

void apply_seed_topology_projection(const SeedDecisionTopologyProjection& projection,
                                    EndpointContinuityDecision* decision);

} // namespace wire::core::generation::detail
