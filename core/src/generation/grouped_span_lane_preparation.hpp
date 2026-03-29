#pragma once

#include "grouped_span_common.hpp"
#include "grouped_span_lowering.hpp"
#include "grouped_span_orientation.hpp"
#include "wire/core/core_state_internal_types.hpp"

#include <functional>
#include <map>
#include <optional>
#include <tuple>

namespace wire::core {
class CoreState;
}

namespace wire::core::generation::detail {

struct GroupedSpanLanePortOrderKey {
  double local_y = 0.0;
  int template_layer = 0;
  int side_rank = 0;
  int height_millimeters = 0;
  ObjectId id = kInvalidObjectId;
};

struct GroupedSpanLaneTerminalPortOrderKey {
  int template_layer = 0;
  int side_rank = 0;
  double local_y = 0.0;
  ObjectId id = kInvalidObjectId;
};

struct GroupedSpanLaneSpanEndpointSample {
  ObjectId port_a_id = kInvalidObjectId;
  ObjectId port_b_id = kInvalidObjectId;
  ObjectId owner_pole_a_id = kInvalidObjectId;
  ObjectId owner_pole_b_id = kInvalidObjectId;
  PortLayer layer_a = PortLayer::kUnknown;
  PortLayer layer_b = PortLayer::kUnknown;
  Vec3d world_a{};
  Vec3d world_b{};
};

[[nodiscard]] inline bool operator<(const GroupedSpanLanePortOrderKey& a, const GroupedSpanLanePortOrderKey& b) {
  return std::tie(a.local_y, a.template_layer, a.side_rank, a.height_millimeters, a.id) <
         std::tie(b.local_y, b.template_layer, b.side_rank, b.height_millimeters, b.id);
}

[[nodiscard]] inline bool operator<(const GroupedSpanLaneTerminalPortOrderKey& a,
                                    const GroupedSpanLaneTerminalPortOrderKey& b) {
  return std::tie(a.template_layer, a.side_rank, a.local_y, a.id) <
         std::tie(b.template_layer, b.side_rank, b.local_y, b.id);
}

class GroupedSpanLaneStateAccess {
public:
  explicit GroupedSpanLaneStateAccess(CoreState& state);

  [[nodiscard]] double effective_pole_layout_yaw_deg(const Pole& pole) const;
  [[nodiscard]] double effective_port_layout_yaw_deg(const Pole& pole, ConnectionCategory category) const;
  [[nodiscard]] const PoleTypeDefinition* find_pole_type(PoleTypeId pole_type_id) const;
  [[nodiscard]] std::optional<Vec3d> port_world_position(ObjectId port_id) const;
  [[nodiscard]] std::optional<int> port_template_layer(ObjectId port_id) const;
  [[nodiscard]] std::optional<double> port_local_y(ObjectId port_id, const Pole* pole, double layout_yaw_deg) const;
  [[nodiscard]] std::optional<PortPlacementSourceKind> port_placement_source(ObjectId port_id) const;
  [[nodiscard]] std::optional<ObjectId> port_owner_pole_id_on_layer(ObjectId port_id, PortLayer layer) const;
  [[nodiscard]] bool port_matches_owner_layer(ObjectId port_id, ObjectId owner_pole_id, PortLayer layer) const;
  [[nodiscard]] bool port_matches_owner_layer_category(ObjectId port_id, ObjectId owner_pole_id, PortLayer layer,
                                                       ConnectionCategory category) const;
  [[nodiscard]] std::optional<GroupedSpanLanePortOrderKey> port_order_key(ObjectId port_id, const Pole* pole,
                                                                          double layout_yaw_deg) const;
  [[nodiscard]] std::optional<GroupedSpanLaneTerminalPortOrderKey>
  port_terminal_order_key(ObjectId port_id, const Pole* pole, double layout_yaw_deg) const;
  [[nodiscard]] std::optional<GroupedSpanLaneSpanEndpointSample> port_segment_sample(const Span& span) const;
  void for_each_span_in_bundle_template(BundleKind bundle_template_id,
                                        const std::function<void(const Span&)>& visitor) const;
  void for_each_port_owned_by_pole(ObjectId pole_id, const std::function<void(const Port&)>& visitor) const;
  void for_each_connected_span(ObjectId port_id, const std::function<void(const Span&)>& visitor) const;
  [[nodiscard]] const GeometrySettings& geometry_settings() const;
  [[nodiscard]] const LayoutSettings& layout_settings() const;
  [[nodiscard]] PortKind port_kind_for_category(ConnectionCategory category) const;
  [[nodiscard]] PortLayer port_layer_for_category(ConnectionCategory category) const;
  [[nodiscard]] int template_layer_for_category(ConnectionCategory category) const;
  [[nodiscard]] double port_category_base_z_for_pole(const Pole& pole, ConnectionCategory category) const;
  [[nodiscard]] double pole_radius_at_height_m(const Pole& pole, double local_z_m) const;
  [[nodiscard]] Vec3d apply_pole_clearance_to_local(const Pole& pole, const Vec3d& local, SlotSide side) const;
  [[nodiscard]] EditResult<ObjectId> AddPort(ObjectId owner_pole_id, const Vec3d& world_position, PortKind kind,
                                             PortLayer layer) const;
  [[nodiscard]] EditResult<ObjectId> ensure_pole_connection_port(const PortResolutionRequest& request) const;
  [[nodiscard]] bool reposition_port_world(ObjectId port_id, const Vec3d& world_position) const;
  [[nodiscard]] bool mark_aerial_branch_port(ObjectId port_id) const;
  [[nodiscard]] bool finalize_branch_support_port(ObjectId port_id, ConnectionCategory category, int template_layer,
                                                  SlotSide template_side) const;
  [[nodiscard]] bool finalize_generated_row_port(ObjectId port_id, const Vec3d* world_position,
                                                 ConnectionCategory category, int template_layer,
                                                 SlotSide template_side) const;
  [[nodiscard]] bool finalize_constrained_solver_port(ObjectId port_id, const Vec3d& world_position,
                                                      ConnectionCategory category, int template_layer,
                                                      SlotSide template_side) const;
  [[nodiscard]] bool finalize_terminal_fallback_port(ObjectId port_id, const Vec3d* world_position,
                                                     ConnectionCategory category, int template_layer,
                                                     SlotSide template_side) const;
  void add_unique_id(std::vector<ObjectId>& ids, ObjectId id) const;

private:
  CoreState* state_ = nullptr;
};

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

  GroupedSpanLaneStateAccess state_;
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
