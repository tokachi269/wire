#pragma once

#include "grouped_span_common.hpp"
#include "grouped_span_lowering.hpp"
#include "grouped_span_orientation.hpp"

#include <map>

namespace wire::core {
class CoreState;
}

namespace wire::core::generation::detail {

class GroupedSpanLanePreparer {
public:
  GroupedSpanLanePreparer(CoreState& state, const GroupedSpanSharedContext& ctx,
                          const GroupedSpanLoweringDecider& lowering,
                          GroupedSpanOrientationDecider& orientation, ObjectId bundle_id,
                          ConnectionCategory category, BundleKind bundle_template_id, PortLayer target_port_layer,
                          int lane_count, double spacing_m, bool use_lane_row_geometry,
                          OrderDecisionPolicyKind order_decision_policy, ChangeSet* change_set);

  [[nodiscard]] EditResult<std::vector<ObjectId>>
  EnsurePorts(ObjectId node_id, ObjectId peer_id, int segment_index, bool prefer_existing_neighbor_order,
              bool* out_seeded_from_previous = nullptr);

private:
  [[nodiscard]] double LayoutYawForPole(const Pole& pole) const;
  [[nodiscard]] double TemplateLayerBaseZForPole(const Pole& pole) const;
  [[nodiscard]] double LaneRowBaseZForPole(const Pole& pole) const;
  [[nodiscard]] double LaneRowTargetZForEndpoint(const Pole& pole, const SegmentRelationFeasibility& feasibility) const;
  [[nodiscard]] std::size_t PortConnectionCount(ObjectId port_id) const;

  CoreState& state_;
  const GroupedSpanSharedContext& ctx_;
  const GroupedSpanLoweringDecider& lowering_;
  GroupedSpanOrientationDecider& orientation_;
  ObjectId bundle_id_ = kInvalidObjectId;
  ConnectionCategory category_ = ConnectionCategory::kLowVoltage;
  BundleKind bundle_template_id_ = BundleKind::kLowVoltage;
  PortLayer target_port_layer_ = PortLayer::kUnknown;
  int lane_count_ = 1;
  double spacing_m_ = 0.0;
  bool use_lane_row_geometry_ = false;
  OrderDecisionPolicyKind order_decision_policy_ = OrderDecisionPolicyKind::kFixedOrder;
  ChangeSet* change_set_ = nullptr;
  std::map<std::pair<ObjectId, ObjectId>, std::vector<ObjectId>> node_lane_ports_cache_{};
};

} // namespace wire::core::generation::detail
