#pragma once

#include "grouped_span_common.hpp"
#include "grouped_span_lowering.hpp"
#include "grouped_span_orientation.hpp"

#include <map>

namespace wire::core {
class CoreState;
}

namespace wire::core::generation::detail {

struct GroupedSpanLanePlan {
  std::vector<std::vector<ObjectId>> base_ports_by_node{};
  std::vector<OrderDecisionChoiceKind> node_order_choices{};
  std::vector<double> turn_angle_by_segment{};
  bool first_seeded_from_previous = false;
};

struct GroupedSpanPreparedPortUsage {
  bool uses_branch_support = false;
  bool solver_used_same_level_constraint = false;
  bool used_special_case_ports = false;
};

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
  [[nodiscard]] EditResult<GroupedSpanLanePlan> BuildLanePlan(const BackboneLoweringPolicy& lowering_policy,
                                                              double corner_threshold_deg);
  void PopulateAssignmentOrdering(const GroupedSpanLanePlan& plan, std::size_t segment_index,
                                  SegmentLaneAssignment* assignment) const;
  [[nodiscard]] GroupedSpanPreparedPortUsage AnalyzePreparedPorts(const std::vector<ObjectId>& port_ids_a,
                                                                  const std::vector<ObjectId>& port_ids_b,
                                                                  bool segment_same_level_feasible) const;
  static void SyncAssignmentFromDecisions(SegmentLaneAssignment* assignment);

private:
  [[nodiscard]] double LayoutYawForPole(const Pole& pole) const;
  [[nodiscard]] double TemplateLayerBaseZForPole(const Pole& pole) const;
  [[nodiscard]] double LaneRowBaseZForPole(const Pole& pole) const;
  [[nodiscard]] double LaneRowTargetZForEndpoint(const Pole& pole, const SegmentRelationFeasibility& feasibility) const;
  [[nodiscard]] std::size_t PortConnectionCount(ObjectId port_id) const;
  [[nodiscard]] double ComputeTurnAngleDeg(const GroupedSpanLanePlan& plan, std::size_t segment_index) const;

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
