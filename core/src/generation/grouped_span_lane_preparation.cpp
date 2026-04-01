#include "grouped_span_lane_preparation.hpp"

#include "detail_utils.hpp"
#include "support_policy.hpp"
#include "wire/core/core_view.hpp"
#include "wire/core/core_state.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <tuple>
#include <unordered_map>
#include <unordered_set>

namespace wire::core::generation::detail {

namespace {

struct BundleOrderScore {
  int segment_xy_intersections = 0;
  int cross_y = 0;
  int cross_z = 0;
  int layer_jump = 0;
  double span_z_delta = 0.0;
};

struct OrientationPlanScore {
  BundleOrderScore order{};
  int adjacent_xy_intersections = 0;
  int orientation_flips = 0;
  int acute_orientation_flips = 0;
};

constexpr double kAngleEps = 1e-6;
constexpr double kReverseStraightAngleDeg = 179.999;

[[nodiscard]] std::vector<ObjectId> order_for_choice(const std::vector<ObjectId>& base_ports,
                                                     OrderDecisionChoiceKind choice) {
  std::vector<ObjectId> ordered = base_ports;
  if (choice == OrderDecisionChoiceKind::kReversed) {
    std::reverse(ordered.begin(), ordered.end());
  }
  return ordered;
}

[[nodiscard]] OrderDecisionChoiceKind choice_for_parity(int parity) {
  return (parity != 0) ? OrderDecisionChoiceKind::kReversed : OrderDecisionChoiceKind::kNormal;
}

void add_order_score(BundleOrderScore* dst, const BundleOrderScore& src) {
  if (dst == nullptr) {
    return;
  }
  dst->segment_xy_intersections += src.segment_xy_intersections;
  dst->cross_y += src.cross_y;
  dst->cross_z += src.cross_z;
  dst->layer_jump += src.layer_jump;
  dst->span_z_delta += src.span_z_delta;
}

[[nodiscard]] bool secondary_score_less(const BundleOrderScore& a, const BundleOrderScore& b) {
  const auto key_a = std::make_tuple(a.cross_z, a.segment_xy_intersections, a.layer_jump,
                                     static_cast<long long>(std::llround(a.span_z_delta * 1000.0)));
  const auto key_b = std::make_tuple(b.cross_z, b.segment_xy_intersections, b.layer_jump,
                                     static_cast<long long>(std::llround(b.span_z_delta * 1000.0)));
  return key_a < key_b;
}

[[nodiscard]] bool orientation_plan_less(const OrientationPlanScore& a, const OrientationPlanScore& b,
                                         bool use_lane_row_geometry) {
  if (use_lane_row_geometry) {
    if (a.adjacent_xy_intersections != b.adjacent_xy_intersections) {
      return a.adjacent_xy_intersections < b.adjacent_xy_intersections;
    }
    if (a.order.segment_xy_intersections != b.order.segment_xy_intersections) {
      return a.order.segment_xy_intersections < b.order.segment_xy_intersections;
    }
    if (a.order.cross_y != b.order.cross_y) {
      return a.order.cross_y < b.order.cross_y;
    }
    if (a.acute_orientation_flips != b.acute_orientation_flips) {
      return a.acute_orientation_flips < b.acute_orientation_flips;
    }
    if (a.orientation_flips != b.orientation_flips) {
      return a.orientation_flips < b.orientation_flips;
    }
  } else if (a.order.cross_y != b.order.cross_y) {
    return a.order.cross_y < b.order.cross_y;
  }
  if (a.acute_orientation_flips != b.acute_orientation_flips) {
    return a.acute_orientation_flips < b.acute_orientation_flips;
  }
  if (a.orientation_flips != b.orientation_flips) {
    return a.orientation_flips < b.orientation_flips;
  }
  return secondary_score_less(a.order, b.order);
}

[[nodiscard]] OrderDecisionChoiceReason order_choice_reason_from_scores(const OrientationPlanScore& chosen,
                                                                        const OrientationPlanScore& alternate,
                                                                        OrderDecisionPolicyKind policy,
                                                                        bool use_lane_row_geometry) {
  if (policy != OrderDecisionPolicyKind::kPermutableHomogeneous) {
    return OrderDecisionChoiceReason::kFixedOrder;
  }
  if (use_lane_row_geometry) {
    if (chosen.adjacent_xy_intersections != alternate.adjacent_xy_intersections ||
        chosen.order.segment_xy_intersections != alternate.order.segment_xy_intersections ||
        chosen.order.cross_y != alternate.order.cross_y) {
      return OrderDecisionChoiceReason::kCrossingFewer;
    }
    if (chosen.acute_orientation_flips != alternate.acute_orientation_flips ||
        chosen.orientation_flips != alternate.orientation_flips) {
      return OrderDecisionChoiceReason::kTwistSmaller;
    }
  } else if (chosen.order.cross_y != alternate.order.cross_y) {
    return OrderDecisionChoiceReason::kCrossingFewer;
  }
  if (chosen.acute_orientation_flips != alternate.acute_orientation_flips ||
      chosen.orientation_flips != alternate.orientation_flips) {
    return OrderDecisionChoiceReason::kTwistSmaller;
  }
  if (chosen.order.cross_z != alternate.order.cross_z ||
      std::abs(chosen.order.span_z_delta - alternate.order.span_z_delta) > 1e-6) {
    return OrderDecisionChoiceReason::kSpacingBetter;
  }
  return OrderDecisionChoiceReason::kKeptDefault;
}

} // namespace

GroupedSpanLaneStateAccess::GroupedSpanLaneStateAccess(CoreState& state) : state_(&state) {}

double GroupedSpanLaneStateAccess::effective_pole_layout_yaw_deg(const Pole& pole) const {
  return state_->effective_pole_layout_yaw_deg(pole);
}

double GroupedSpanLaneStateAccess::effective_port_layout_yaw_deg(const Pole& pole, ConnectionCategory category) const {
  return state_->effective_port_layout_yaw_deg(pole, category);
}

const PoleTypeDefinition* GroupedSpanLaneStateAccess::find_pole_type(PoleTypeId pole_type_id) const {
  return state_->find_pole_type(pole_type_id);
}

namespace {

[[nodiscard]] int side_rank(SlotSide side) {
  switch (side) {
  case SlotSide::kLeft:
    return 0;
  case SlotSide::kCenter:
    return 1;
  case SlotSide::kRight:
    return 2;
  default:
    return 3;
  }
}

} // namespace

std::optional<Vec3d> GroupedSpanLaneStateAccess::port_world_position(ObjectId port_id) const {
  const Port* port = state_->edit_state_access().ports.find(port_id);
  if (port == nullptr) {
    return std::nullopt;
  }
  return port->world_position;
}

std::optional<int> GroupedSpanLaneStateAccess::port_template_layer(ObjectId port_id) const {
  const Port* port = state_->edit_state_access().ports.find(port_id);
  if (port == nullptr) {
    return std::nullopt;
  }
  return port->template_layer;
}

std::optional<double> GroupedSpanLaneStateAccess::port_local_y(ObjectId port_id, const Pole* pole,
                                                               double layout_yaw_deg) const {
  const auto world = port_world_position(port_id);
  if (!world.has_value()) {
    return std::nullopt;
  }
  if (pole == nullptr) {
    return 0.0;
  }
  return WorldPointToLocal(BuildPoleFrame(pole->world_transform, layout_yaw_deg), *world).y;
}

std::optional<PortPlacementSourceKind> GroupedSpanLaneStateAccess::port_placement_source(ObjectId port_id) const {
  const Port* port = state_->edit_state_access().ports.find(port_id);
  if (port == nullptr) {
    return std::nullopt;
  }
  return port->placement_source;
}

std::optional<ObjectId> GroupedSpanLaneStateAccess::port_owner_pole_id_on_layer(ObjectId port_id, PortLayer layer) const {
  const Port* port = state_->edit_state_access().ports.find(port_id);
  if (port == nullptr || port->layer != layer) {
    return std::nullopt;
  }
  return port->owner_pole_id;
}

bool GroupedSpanLaneStateAccess::port_matches_owner_layer(ObjectId port_id, ObjectId owner_pole_id,
                                                          PortLayer layer) const {
  const Port* port = state_->edit_state_access().ports.find(port_id);
  return port != nullptr && port->owner_pole_id == owner_pole_id && port->layer == layer;
}

bool GroupedSpanLaneStateAccess::port_matches_owner_layer_category(ObjectId port_id, ObjectId owner_pole_id,
                                                                   PortLayer layer,
                                                                   ConnectionCategory category) const {
  const Port* port = state_->edit_state_access().ports.find(port_id);
  return port != nullptr && port->owner_pole_id == owner_pole_id && port->layer == layer &&
         port->category == category;
}

std::optional<GroupedSpanLanePortOrderKey> GroupedSpanLaneStateAccess::port_order_key(ObjectId port_id,
                                                                                       const Pole* pole,
                                                                                       double layout_yaw_deg) const {
  const Port* port = state_->edit_state_access().ports.find(port_id);
  if (port == nullptr) {
    return std::nullopt;
  }
  double local_y = 0.0;
  if (pole != nullptr) {
    local_y = WorldPointToLocal(BuildPoleFrame(pole->world_transform, layout_yaw_deg), port->world_position).y;
  }
  return GroupedSpanLanePortOrderKey{
      .local_y = local_y,
      .template_layer = port->template_layer,
      .side_rank = side_rank(port->template_side),
      .height_millimeters = static_cast<int>(std::llround(HeightAlongWorldUp(port->world_position) * 1000.0)),
      .id = port->id,
  };
}

std::optional<GroupedSpanLaneTerminalPortOrderKey>
GroupedSpanLaneStateAccess::port_terminal_order_key(ObjectId port_id, const Pole* pole, double layout_yaw_deg) const {
  const Port* port = state_->edit_state_access().ports.find(port_id);
  if (port == nullptr) {
    return std::nullopt;
  }
  double local_y = 0.0;
  if (pole != nullptr) {
    local_y = WorldPointToLocal(BuildPoleFrame(pole->world_transform, layout_yaw_deg), port->world_position).y;
  }
  return GroupedSpanLaneTerminalPortOrderKey{
      .template_layer = port->template_layer,
      .side_rank = side_rank(port->template_side),
      .local_y = local_y,
      .id = port->id,
  };
}

std::optional<GroupedSpanLaneSpanEndpointSample>
GroupedSpanLaneStateAccess::port_segment_sample(const Span& span) const {
  const Port* port_a = state_->edit_state_access().ports.find(span.port_a_id);
  const Port* port_b = state_->edit_state_access().ports.find(span.port_b_id);
  if (port_a == nullptr || port_b == nullptr) {
    return std::nullopt;
  }
  return GroupedSpanLaneSpanEndpointSample{
      .port_a_id = port_a->id,
      .port_b_id = port_b->id,
      .owner_pole_a_id = port_a->owner_pole_id,
      .owner_pole_b_id = port_b->owner_pole_id,
      .layer_a = port_a->layer,
      .layer_b = port_b->layer,
      .world_a = port_a->world_position,
      .world_b = port_b->world_position,
  };
}

void GroupedSpanLaneStateAccess::for_each_span_in_bundle_template(
    BundleKind bundle_template_id, const std::function<void(const Span&)>& visitor) const {
  for (const Span& span : state_->edit_state_access().spans.items()) {
    if (span.bundle_id == kInvalidObjectId) {
      continue;
    }
    const Bundle* bundle = state_->edit_state_access().bundles.find(span.bundle_id);
    if (bundle == nullptr || bundle->bundle_template_id != bundle_template_id) {
      continue;
    }
    visitor(span);
  }
}

void GroupedSpanLaneStateAccess::for_each_port_owned_by_pole(ObjectId pole_id,
                                                             const std::function<void(const Port&)>& visitor) const {
  const auto it = state_->relation_index_access().ports_by_pole.find(pole_id);
  if (it == state_->relation_index_access().ports_by_pole.end()) {
    return;
  }
  for (ObjectId port_id : it->second) {
    const Port* port = state_->edit_state_access().ports.find(port_id);
    if (port != nullptr) {
      visitor(*port);
    }
  }
}

void GroupedSpanLaneStateAccess::for_each_connected_span(ObjectId port_id,
                                                         const std::function<void(const Span&)>& visitor) const {
  const auto it = state_->connection_index_access().spans_by_port.find(port_id);
  if (it == state_->connection_index_access().spans_by_port.end()) {
    return;
  }
  for (ObjectId span_id : it->second) {
    const Span* span = state_->edit_state_access().spans.find(span_id);
    if (span != nullptr) {
      visitor(*span);
    }
  }
}

const GeometrySettings& GroupedSpanLaneStateAccess::geometry_settings() const {
  return state_->cache_state_access().geometry_settings;
}

const LayoutSettings& GroupedSpanLaneStateAccess::layout_settings() const {
  return state_->authoritative_.layout_settings;
}

PortKind GroupedSpanLaneStateAccess::port_kind_for_category(ConnectionCategory category) const {
  return CoreState::category_to_port_kind(category);
}

PortLayer GroupedSpanLaneStateAccess::port_layer_for_category(ConnectionCategory category) const {
  return CoreState::category_to_port_layer(category);
}

int GroupedSpanLaneStateAccess::template_layer_for_category(ConnectionCategory category) const {
  return generation::detail::TemplateLayerForCategory(category);
}

double GroupedSpanLaneStateAccess::port_category_base_z_for_pole(const Pole& pole, ConnectionCategory category) const {
  return state_->view().port_category_base_z_for_pole(pole, category);
}

double GroupedSpanLaneStateAccess::pole_radius_at_height_m(const Pole& pole, double local_z_m) const {
  return state_->pole_radius_at_height_m(pole, local_z_m);
}

Vec3d GroupedSpanLaneStateAccess::apply_pole_clearance_to_local(const Pole& pole, const Vec3d& local,
                                                                SlotSide side) const {
  return state_->apply_pole_clearance_to_local(pole, local, side);
}

EditResult<ObjectId> GroupedSpanLaneStateAccess::AddPort(ObjectId owner_pole_id, const Vec3d& world_position,
                                                         PortKind kind, PortLayer layer) const {
  return state_->AddPort(owner_pole_id, world_position, kind, layer);
}

EditResult<ObjectId> GroupedSpanLaneStateAccess::ensure_pole_connection_port(
    const PortResolutionRequest& request) const {
  return state_->ensure_pole_connection_port(request);
}

bool GroupedSpanLaneStateAccess::reposition_port_world(ObjectId port_id, const Vec3d& world_position) const {
  Port* port = state_->edit_state_access().ports.find(port_id);
  if (port == nullptr) {
    return false;
  }
  port->world_position = world_position;
  return true;
}

bool GroupedSpanLaneStateAccess::mark_aerial_branch_port(ObjectId port_id) const {
  Port* port = state_->edit_state_access().ports.find(port_id);
  if (port == nullptr) {
    return false;
  }
  port->generated_by_rule = true;
  port->placement_context = ConnectionContext::kBranchAdd;
  CoreState::apply_port_position_mode(*port, PortPositionMode::kAuto, PortPlacementSourceKind::kAerialBranch);
  return true;
}

bool GroupedSpanLaneStateAccess::finalize_branch_support_port(ObjectId port_id, ConnectionCategory category,
                                                              int template_layer, SlotSide template_side) const {
  Port* port = state_->edit_state_access().ports.find(port_id);
  if (port == nullptr) {
    return false;
  }
  port->category = category;
  port->template_layer = template_layer;
  port->template_side = template_side;
  port->template_role = SlotRole::kBranchPreferred;
  port->generated_from_template = false;
  port->generated_by_rule = true;
  port->placement_context = ConnectionContext::kBranchAdd;
  CoreState::apply_port_position_mode(*port, PortPositionMode::kAuto, PortPlacementSourceKind::kBranchSupport);
  return true;
}

bool GroupedSpanLaneStateAccess::finalize_generated_row_port(ObjectId port_id, const Vec3d* world_position,
                                                             ConnectionCategory category, int template_layer,
                                                             SlotSide template_side) const {
  Port* port = state_->edit_state_access().ports.find(port_id);
  if (port == nullptr) {
    return false;
  }
  if (world_position != nullptr) {
    port->world_position = *world_position;
  }
  port->category = category;
  port->template_layer = template_layer;
  port->template_side = template_side;
  port->generated_by_rule = true;
  port->placement_context = ConnectionContext::kTrunkContinue;
  CoreState::apply_port_position_mode(*port, PortPositionMode::kAuto, PortPlacementSourceKind::kGenerated);
  return true;
}

bool GroupedSpanLaneStateAccess::finalize_constrained_solver_port(ObjectId port_id, const Vec3d& world_position,
                                                                  ConnectionCategory category, int template_layer,
                                                                  SlotSide template_side) const {
  Port* port = state_->edit_state_access().ports.find(port_id);
  if (port == nullptr) {
    return false;
  }
  port->world_position = world_position;
  port->category = category;
  port->template_layer = template_layer;
  port->template_side = template_side;
  port->generated_from_template = true;
  port->generated_by_rule = true;
  CoreState::apply_port_position_mode(*port, PortPositionMode::kAuto, PortPlacementSourceKind::kPlacementBandConstrained);
  return true;
}

bool GroupedSpanLaneStateAccess::finalize_terminal_fallback_port(ObjectId port_id, const Vec3d* world_position,
                                                                 ConnectionCategory category, int template_layer,
                                                                 SlotSide template_side) const {
  Port* port = state_->edit_state_access().ports.find(port_id);
  if (port == nullptr) {
    return false;
  }
  if (world_position != nullptr) {
    port->world_position = *world_position;
  }
  port->category = category;
  port->template_layer = template_layer;
  port->template_side = template_side;
  port->generated_by_rule = true;
  port->placement_context = ConnectionContext::kTrunkContinue;
  CoreState::apply_port_position_mode(*port, PortPositionMode::kAuto, PortPlacementSourceKind::kGenerated);
  return true;
}

void GroupedSpanLaneStateAccess::add_unique_id(std::vector<ObjectId>& ids, ObjectId id) const {
  CoreState::add_unique_id(ids, id);
}

GroupedSpanLanePreparer::GroupedSpanLanePreparer(CoreState& state, const GroupedSpanSharedContext& ctx,
                                                 const GroupedSpanLoweringDecider& lowering,
                                                 GroupedSpanOrientationDecider& orientation, ObjectId bundle_id,
                                                 ConnectionCategory category, BundleKind bundle_template_id,
                                                 PortLayer target_port_layer, int lane_count, double spacing_m,
                                                 bool use_lane_row_geometry,
                                                 OrderDecisionPolicyKind order_decision_policy,
                                                 ChangeSet* change_set)
    : state_(state), ctx_(ctx), lowering_(lowering), orientation_(orientation), bundle_id_(bundle_id),
      category_(category), bundle_template_id_(bundle_template_id), target_port_layer_(target_port_layer),
      lane_count_(lane_count), spacing_m_(spacing_m), use_lane_row_geometry_(use_lane_row_geometry),
      order_decision_policy_(order_decision_policy), change_set_(change_set) {}

double GroupedSpanLanePreparer::LayoutYawForPole(const Pole& pole) const {
  return state_.effective_port_layout_yaw_deg(pole, category_);
}

double GroupedSpanLanePreparer::TemplateLayerBaseZForPole(const Pole& pole) const {
  return state_.port_category_base_z_for_pole(pole, category_);
}

double GroupedSpanLanePreparer::LaneRowBaseZForPole(const Pole& pole) const {
  return TemplateLayerBaseZForPole(pole);
}

double GroupedSpanLanePreparer::LaneRowTargetZForEndpoint(const Pole& pole,
                                                          const SegmentRelationFeasibility& feasibility) const {
  return std::max(0.5, LaneRowBaseZForPole(pole) - lowering_.EndpointLoweringOffsetM(feasibility));
}

std::size_t GroupedSpanLanePreparer::PortConnectionCount(ObjectId port_id) const {
  std::size_t count = 0;
  state_.for_each_connected_span(port_id, [&](const Span&) { ++count; });
  return count;
}

double GroupedSpanLanePreparer::ComputeTurnAngleDeg(const GroupedSpanLanePlan& plan, std::size_t segment_index) const {
  (void)plan;
  if (segment_index == 0 || segment_index + 1 >= ctx_.node_ids.size()) {
    return 180.0;
  }
  const Vec3d prev = ctx_.support_position(ctx_.node_ids[segment_index - 1]);
  const Vec3d curr = ctx_.support_position(ctx_.node_ids[segment_index]);
  const Vec3d next = ctx_.support_position(ctx_.node_ids[segment_index + 1]);
  const Vec3d in_check = prev - curr;
  const Vec3d out_check = next - curr;
  if ((in_check.x * in_check.x + in_check.y * in_check.y + in_check.z * in_check.z) <= 1e-12 ||
      (out_check.x * out_check.x + out_check.y * out_check.y + out_check.z * out_check.z) <= 1e-12) {
    return 180.0;
  }
  Vec3d in_dir = prev - curr;
  Vec3d out_dir = next - curr;
  if (!normalize_xy(&in_dir) || !normalize_xy(&out_dir)) {
    return 180.0;
  }
  const double d = std::clamp(dot_xy(in_dir, out_dir), -1.0, 1.0);
  const double angle = std::acos(d) * (180.0 / kPi);
  return std::isfinite(angle) ? angle : 180.0;
}

void GroupedSpanLanePreparer::SyncAssignmentFromDecisions(SegmentLaneAssignment* assignment) {
  if (assignment == nullptr) {
    return;
  }
  assignment->relation_a = assignment->decision_a.relation_kind;
  assignment->relation_b = assignment->decision_b.relation_kind;
  assignment->continuity_class =
      (assignment->decision_a.continuity_class == ContinuityCategoryClass::kBundleLike ||
       assignment->decision_b.continuity_class == ContinuityCategoryClass::kBundleLike)
          ? ContinuityCategoryClass::kBundleLike
          : ContinuityCategoryClass::kPointLike;
  assignment->default_lower_required =
      assignment->decision_a.default_lower_required || assignment->decision_b.default_lower_required;
  assignment->same_level_feasible =
      assignment->decision_a.same_level_feasible && assignment->decision_b.same_level_feasible;
  assignment->same_level_reason =
      assignment->same_level_feasible ? SameLevelFeasibilityReason::kNone
                                      : assignment->decision_a.same_level_feasible
                                            ? assignment->decision_b.same_level_reason
                                            : assignment->decision_a.same_level_reason;
  assignment->projected_spacing_topview_m =
      (!assignment->decision_a.same_level_feasible && !assignment->decision_b.same_level_feasible)
          ? std::min(assignment->decision_a.projected_spacing_topview_m,
                     assignment->decision_b.projected_spacing_topview_m)
          : (!assignment->decision_a.same_level_feasible ? assignment->decision_a.projected_spacing_topview_m
                                                         : assignment->decision_b.projected_spacing_topview_m);
  assignment->required_clearance_m =
      std::max(assignment->decision_a.required_clearance_m, assignment->decision_b.required_clearance_m);
  assignment->lowering_blocked_by_policy =
      assignment->decision_a.lowering_blocked_by_policy || assignment->decision_b.lowering_blocked_by_policy;
  assignment->unresolved_same_level_conflict =
      assignment->decision_a.unresolved_same_level_conflict || assignment->decision_b.unresolved_same_level_conflict;
}

GroupedSpanPreparedPortUsage
GroupedSpanLanePreparer::AnalyzePreparedPorts(const std::vector<ObjectId>& port_ids_a,
                                              const std::vector<ObjectId>& port_ids_b,
                                              bool segment_same_level_feasible) const {
  auto ports_use_branch_support = [&](const std::vector<ObjectId>& port_ids) {
    return std::any_of(port_ids.begin(), port_ids.end(), [&](ObjectId port_id) {
      const auto source = state_.port_placement_source(port_id);
      return source && *source == PortPlacementSourceKind::kBranchSupport;
    });
  };
  auto ports_use_constrained_solver = [&](const std::vector<ObjectId>& port_ids) {
    return std::any_of(port_ids.begin(), port_ids.end(), [&](ObjectId port_id) {
      const auto source = state_.port_placement_source(port_id);
      return source && *source == PortPlacementSourceKind::kPlacementBandConstrained;
    });
  };
  auto ports_use_special_case_source = [&](const std::vector<ObjectId>& port_ids) {
    return std::any_of(port_ids.begin(), port_ids.end(), [&](ObjectId port_id) {
      const auto source = state_.port_placement_source(port_id);
      return source &&
             (*source == PortPlacementSourceKind::kGenerated ||
              *source == PortPlacementSourceKind::kBranchSupport ||
              *source == PortPlacementSourceKind::kAerialBranch);
    });
  };

  GroupedSpanPreparedPortUsage usage{};
  usage.uses_branch_support = ports_use_branch_support(port_ids_a) || ports_use_branch_support(port_ids_b);
  usage.solver_used_same_level_constraint =
      ports_use_constrained_solver(port_ids_a) || ports_use_constrained_solver(port_ids_b);
  usage.used_special_case_ports =
      !usage.solver_used_same_level_constraint && !segment_same_level_feasible &&
      (ports_use_special_case_source(port_ids_a) || ports_use_special_case_source(port_ids_b));
  return usage;
}

EditResult<GroupedSpanLanePlan>
GroupedSpanLanePreparer::BuildLanePlan(const BackboneLoweringPolicy& lowering_policy, double corner_threshold_deg) {
  (void)lowering_policy;

  EditResult<GroupedSpanLanePlan> result;
  const std::size_t node_count = ctx_.node_ids.size();
  const std::size_t segment_count = (node_count > 0) ? (node_count - 1) : 0;
  if (segment_count == 0) {
    result.error = "at least one grouped segment is required";
    return result;
  }

  GroupedSpanLanePlan plan{};
  plan.base_ports_by_node.resize(node_count);
  EditResult<std::vector<ObjectId>> first_ports =
      EnsurePorts(ctx_.node_ids.front(), ctx_.node_ids[1], 0, true, &plan.first_seeded_from_previous);
  if (!first_ports.ok) {
    result.error = first_ports.error;
    return result;
  }
  if (static_cast<int>(first_ports.value.size()) != lane_count_) {
    result.error = "failed to seed first segment lanes";
    return result;
  }
  plan.base_ports_by_node.front() = first_ports.value;

  for (std::size_t seg = 0; seg < segment_count; ++seg) {
    EditResult<std::vector<ObjectId>> right_ports =
        EnsurePorts(ctx_.node_ids[seg + 1], ctx_.node_ids[seg], static_cast<int>(seg), false, nullptr);
    if (!right_ports.ok) {
      result.error = right_ports.error;
      return result;
    }
    if (static_cast<int>(right_ports.value.size()) != lane_count_) {
      result.error = "failed to prepare right-side lane candidates";
      return result;
    }
    plan.base_ports_by_node[seg + 1] = std::move(right_ports.value);
  }

  auto evaluate_increment = [&](ObjectId node_a, ObjectId node_b, const std::vector<ObjectId>& lanes_a,
                                const std::vector<ObjectId>& lanes_b) -> BundleOrderScore {
    BundleOrderScore score{};
    const Pole* pole_a = ctx_.support_pole(node_a);
    const Pole* pole_b = ctx_.support_pole(node_b);
    const Vec3d pos_a = ctx_.support_position(node_a);
    const Vec3d pos_b = ctx_.support_position(node_b);

    Vec3d segment_dir{1.0, 0.0, 0.0};
    if (((pos_b.x - pos_a.x) * (pos_b.x - pos_a.x) + (pos_b.y - pos_a.y) * (pos_b.y - pos_a.y) +
         (pos_b.z - pos_a.z) * (pos_b.z - pos_a.z)) > 1e-12) {
      segment_dir = pos_b - pos_a;
      if (!normalize_xy(&segment_dir) || !std::isfinite(segment_dir.x) || !std::isfinite(segment_dir.y)) {
        segment_dir = {1.0, 0.0, 0.0};
      }
    }
    const Vec3d lateral_axis{-segment_dir.y, segment_dir.x, 0.0};
    auto axis_for_node = [&](ObjectId node_id, ObjectId peer_id) -> Vec3d {
      const Vec3d axis = orientation_.CanonicalSideAxisForOrder(node_id, peer_id);
      if (axis.x != 0.0 || axis.y != 0.0) {
        return axis;
      }
      return lateral_axis;
    };
    const Vec3d axis_a = axis_for_node(node_a, node_b);
    const Vec3d axis_b = axis_for_node(node_b, node_a);
    const double y_sign_b = (dot_xy(axis_a, axis_b) < 0.0) ? -1.0 : 1.0;

    std::vector<double> y_a(static_cast<std::size_t>(lane_count_), 0.0);
    std::vector<double> y_b(static_cast<std::size_t>(lane_count_), 0.0);
    std::vector<double> z_a(static_cast<std::size_t>(lane_count_), 0.0);
    std::vector<double> z_b(static_cast<std::size_t>(lane_count_), 0.0);
    for (int lane = 0; lane < lane_count_; ++lane) {
      const std::size_t idx = static_cast<std::size_t>(lane);
      const auto world_a = state_.port_world_position(lanes_a[idx]);
      const auto world_b = state_.port_world_position(lanes_b[idx]);
      const auto template_layer_a = state_.port_template_layer(lanes_a[idx]);
      const auto template_layer_b = state_.port_template_layer(lanes_b[idx]);
      if (!world_a || !world_b || !template_layer_a || !template_layer_b) {
        score.layer_jump += 4;
        score.span_z_delta += 5.0;
        continue;
      }
      if (use_lane_row_geometry_) {
        const Vec3d local_a =
            (pole_a == nullptr) ? *world_a
                                : WorldPointToLocal(BuildPoleFrame(pole_a->world_transform, LayoutYawForPole(*pole_a)),
                                                    *world_a);
        const Vec3d local_b =
            (pole_b == nullptr) ? *world_b
                                : WorldPointToLocal(BuildPoleFrame(pole_b->world_transform, LayoutYawForPole(*pole_b)),
                                                    *world_b);
        y_a[idx] = local_a.y;
        y_b[idx] = local_b.y * y_sign_b;
      } else {
        y_a[idx] = dot_xy(*world_a - pos_a, axis_a);
        y_b[idx] = dot_xy(*world_b - pos_b, axis_b) * y_sign_b;
      }
      z_a[idx] = world_a->z;
      z_b[idx] = world_b->z;
      score.layer_jump += std::abs(*template_layer_a - *template_layer_b);
      score.span_z_delta += std::abs(world_a->z - world_b->z);
    }

    for (int i = 0; i < lane_count_; ++i) {
      for (int j = i + 1; j < lane_count_; ++j) {
        const std::size_t ii = static_cast<std::size_t>(i);
        const std::size_t jj = static_cast<std::size_t>(j);
        const auto world_ai = state_.port_world_position(lanes_a[ii]);
        const auto world_bi = state_.port_world_position(lanes_b[ii]);
        const auto world_aj = state_.port_world_position(lanes_a[jj]);
        const auto world_bj = state_.port_world_position(lanes_b[jj]);
        if (world_ai && world_bi && world_aj && world_bj &&
            segments_intersect_xy_strict_local(*world_ai, *world_bi, *world_aj, *world_bj)) {
          ++score.segment_xy_intersections;
        }
        constexpr double kOrderEps = 1e-4;
        const double dy_a = y_a[ii] - y_a[jj];
        const double dy_b = y_b[ii] - y_b[jj];
        if ((dy_a > kOrderEps && dy_b < -kOrderEps) || (dy_a < -kOrderEps && dy_b > kOrderEps)) {
          ++score.cross_y;
        }
        const double dz_a = z_a[ii] - z_a[jj];
        const double dz_b = z_b[ii] - z_b[jj];
        if ((dz_a > kOrderEps && dz_b < -kOrderEps) || (dz_a < -kOrderEps && dz_b > kOrderEps)) {
          ++score.cross_z;
        }
      }
    }
    return score;
  };

  auto count_adjacent_segment_xy_intersections = [&](const std::vector<ObjectId>& prev_a,
                                                     const std::vector<ObjectId>& prev_b,
                                                     const std::vector<ObjectId>& curr_a,
                                                     const std::vector<ObjectId>& curr_b) {
    int intersections = 0;
    for (int i = 0; i < lane_count_; ++i) {
      const std::size_t ii = static_cast<std::size_t>(i);
      const auto curr_a_world = state_.port_world_position(curr_a[ii]);
      const auto curr_b_world = state_.port_world_position(curr_b[ii]);
      if (!curr_a_world || !curr_b_world) {
        continue;
      }
      for (int j = 0; j < lane_count_; ++j) {
        if (i == j) {
          continue;
        }
        const std::size_t jj = static_cast<std::size_t>(j);
        const auto prev_a_world = state_.port_world_position(prev_a[jj]);
        const auto prev_b_world = state_.port_world_position(prev_b[jj]);
        if (!prev_a_world || !prev_b_world) {
          continue;
        }
        if (segments_intersect_xy_strict_local(*curr_a_world, *curr_b_world, *prev_a_world, *prev_b_world)) {
          ++intersections;
        }
      }
    }
    return intersections;
  };

  const bool allow_order_decision_reverse =
      order_decision_policy_ == OrderDecisionPolicyKind::kPermutableHomogeneous && lane_count_ > 1;
  plan.turn_angle_by_segment.assign(segment_count, 180.0);
  for (std::size_t seg = 0; seg < segment_count; ++seg) {
    plan.turn_angle_by_segment[seg] = ComputeTurnAngleDeg(plan, seg);
  }

  std::vector<int> node_parity(node_count, 0);
  const std::vector<int> first_candidates =
      (plan.first_seeded_from_previous || !allow_order_decision_reverse) ? std::vector<int>{0}
                                                                         : std::vector<int>{0, 1};
  if (segment_count == 1) {
    OrientationPlanScore best_score{};
    bool has_best = false;
    for (int parity_a : first_candidates) {
      const std::vector<ObjectId> ports_a = order_for_choice(plan.base_ports_by_node[0], choice_for_parity(parity_a));
      for (int parity_b : {0, 1}) {
        if (!allow_order_decision_reverse && parity_b != 0) {
          continue;
        }
        const std::vector<ObjectId> ports_b =
            order_for_choice(plan.base_ports_by_node[1], choice_for_parity(parity_b));
        OrientationPlanScore candidate{};
        candidate.order = evaluate_increment(ctx_.node_ids[0], ctx_.node_ids[1], ports_a, ports_b);
        if (!has_best || orientation_plan_less(candidate, best_score, use_lane_row_geometry_)) {
          has_best = true;
          best_score = candidate;
          node_parity[0] = parity_a;
          node_parity[1] = parity_b;
        }
      }
    }
  } else {
    struct DpCell {
      bool reachable = false;
      OrientationPlanScore score{};
      int prev_prev_parity = -1;
    };
    std::vector<std::array<std::array<DpCell, 2>, 2>> dp(segment_count + 1);
    for (int parity_0 : first_candidates) {
      const std::vector<ObjectId> ports_0 = order_for_choice(plan.base_ports_by_node[0], choice_for_parity(parity_0));
      for (int parity_1 : {0, 1}) {
        if (!allow_order_decision_reverse && parity_1 != 0) {
          continue;
        }
        const std::vector<ObjectId> ports_1 =
            order_for_choice(plan.base_ports_by_node[1], choice_for_parity(parity_1));
        DpCell& cell = dp[1][parity_0][parity_1];
        cell.reachable = true;
        cell.prev_prev_parity = -1;
        cell.score.order = evaluate_increment(ctx_.node_ids[0], ctx_.node_ids[1], ports_0, ports_1);
      }
    }

    for (std::size_t step = 1; step < segment_count; ++step) {
      for (int parity_prev_prev : {0, 1}) {
        for (int parity_prev : {0, 1}) {
          const DpCell& cell = dp[step][parity_prev_prev][parity_prev];
          if (!cell.reachable) {
            continue;
          }
          const int prev_orientation = parity_prev_prev ^ parity_prev;
          for (int parity_curr : {0, 1}) {
            if (!allow_order_decision_reverse && parity_curr != 0) {
              continue;
            }
            const std::vector<ObjectId> ports_prev =
                order_for_choice(plan.base_ports_by_node[step], choice_for_parity(parity_prev));
            const std::vector<ObjectId> ports_curr =
                order_for_choice(plan.base_ports_by_node[step + 1], choice_for_parity(parity_curr));
            OrientationPlanScore candidate = cell.score;
            add_order_score(&candidate.order,
                            evaluate_increment(ctx_.node_ids[step], ctx_.node_ids[step + 1], ports_prev, ports_curr));
            if (use_lane_row_geometry_) {
              const std::vector<ObjectId> ports_prev_prev =
                  order_for_choice(plan.base_ports_by_node[step - 1], choice_for_parity(parity_prev_prev));
              candidate.adjacent_xy_intersections +=
                  count_adjacent_segment_xy_intersections(ports_prev_prev, ports_prev, ports_prev, ports_curr);
            }
            const int curr_orientation = parity_prev ^ parity_curr;
            if (curr_orientation != prev_orientation &&
                (plan.turn_angle_by_segment[step] + kAngleEps < kReverseStraightAngleDeg)) {
              ++candidate.orientation_flips;
              if (plan.turn_angle_by_segment[step] + kAngleEps < corner_threshold_deg) {
                ++candidate.acute_orientation_flips;
              }
            }

            DpCell& next = dp[step + 1][parity_prev][parity_curr];
            if (!next.reachable || orientation_plan_less(candidate, next.score, use_lane_row_geometry_)) {
              next.reachable = true;
              next.score = candidate;
              next.prev_prev_parity = parity_prev_prev;
            }
          }
        }
      }
    }

    bool has_best = false;
    OrientationPlanScore best_score{};
    int best_prev = 0;
    int best_curr = 0;
    for (int parity_prev : {0, 1}) {
      for (int parity_curr : {0, 1}) {
        const DpCell& cell = dp[segment_count][parity_prev][parity_curr];
        if (!cell.reachable) {
          continue;
        }
        if (!has_best || orientation_plan_less(cell.score, best_score, use_lane_row_geometry_)) {
          has_best = true;
          best_score = cell.score;
          best_prev = parity_prev;
          best_curr = parity_curr;
        }
      }
    }
    if (!has_best) {
      result.error = "failed to resolve deterministic node orientation plan";
      return result;
    }
    node_parity[node_count - 2] = best_prev;
    node_parity[node_count - 1] = best_curr;
    for (std::size_t step = segment_count; step > 1; --step) {
      const DpCell& cell = dp[step][node_parity[step - 1]][node_parity[step]];
      node_parity[step - 2] = cell.prev_prev_parity;
    }
  }

  if (use_lane_row_geometry_ && allow_order_decision_reverse && node_count >= 3) {
    auto score_segments_for_parity = [&](const std::vector<int>& parities, std::size_t first_segment,
                                         std::size_t one_past_last_segment) {
      OrientationPlanScore score{};
      if (first_segment >= one_past_last_segment || first_segment >= segment_count) {
        return score;
      }
      const std::size_t last_segment = std::min(one_past_last_segment, segment_count);
      for (std::size_t seg = first_segment; seg < last_segment; ++seg) {
        const std::vector<ObjectId> ports_a =
            order_for_choice(plan.base_ports_by_node[seg], choice_for_parity(parities[seg]));
        const std::vector<ObjectId> ports_b =
            order_for_choice(plan.base_ports_by_node[seg + 1], choice_for_parity(parities[seg + 1]));
        add_order_score(&score.order, evaluate_increment(ctx_.node_ids[seg], ctx_.node_ids[seg + 1], ports_a, ports_b));
        if (seg > 0) {
          const std::vector<ObjectId> prev_ports_a =
              order_for_choice(plan.base_ports_by_node[seg - 1], choice_for_parity(parities[seg - 1]));
          const std::vector<ObjectId> prev_ports_b =
              order_for_choice(plan.base_ports_by_node[seg], choice_for_parity(parities[seg]));
          score.adjacent_xy_intersections +=
              count_adjacent_segment_xy_intersections(prev_ports_a, prev_ports_b, prev_ports_b, ports_b);
          const int prev_orientation = parities[seg - 1] ^ parities[seg];
          const int curr_orientation = parities[seg] ^ parities[seg + 1];
          if (curr_orientation != prev_orientation &&
              (plan.turn_angle_by_segment[seg] + kAngleEps < kReverseStraightAngleDeg)) {
            ++score.orientation_flips;
            if (plan.turn_angle_by_segment[seg] + kAngleEps < corner_threshold_deg) {
              ++score.acute_orientation_flips;
            }
          }
        }
      }
      return score;
    };

    bool improved = true;
    while (improved) {
      improved = false;
      for (std::size_t node_index = 1; node_index + 1 < node_count; ++node_index) {
        const std::size_t first_segment = (node_index == 0) ? 0 : (node_index - 1);
        const std::size_t one_past_last_segment = std::min(segment_count, node_index + 1);
        const OrientationPlanScore current_score =
            score_segments_for_parity(node_parity, first_segment, one_past_last_segment);
        std::vector<int> flipped = node_parity;
        flipped[node_index] ^= 1;
        const OrientationPlanScore flipped_score =
            score_segments_for_parity(flipped, first_segment, one_past_last_segment);
        if (orientation_plan_less(flipped_score, current_score, use_lane_row_geometry_)) {
          node_parity.swap(flipped);
          improved = true;
        }
      }
    }
  }

  plan.node_order_choices.assign(node_count, OrderDecisionChoiceKind::kNormal);
  for (std::size_t i = 0; i < node_count; ++i) {
    plan.node_order_choices[i] = choice_for_parity(node_parity[i]);
  }

  result.value = std::move(plan);
  result.ok = true;
  return result;
}

void GroupedSpanLanePreparer::PopulateAssignmentOrdering(const GroupedSpanLanePlan& plan, std::size_t segment_index,
                                                         SegmentLaneAssignment* assignment) const {
  if (assignment == nullptr || segment_index + 1 >= plan.base_ports_by_node.size() ||
      segment_index + 1 >= plan.node_order_choices.size()) {
    return;
  }

  assignment->order_decision_policy = order_decision_policy_;
  assignment->order_decision_choice_a = plan.node_order_choices[segment_index];
  assignment->order_decision_choice_b = plan.node_order_choices[segment_index + 1];
  assignment->port_ids_a =
      order_for_choice(plan.base_ports_by_node[segment_index], assignment->order_decision_choice_a);
  assignment->port_ids_b =
      order_for_choice(plan.base_ports_by_node[segment_index + 1], assignment->order_decision_choice_b);

  const bool chosen_orientation_flip = (assignment->order_decision_choice_a != assignment->order_decision_choice_b);
  assignment->turn_angle_deg =
      (segment_index < plan.turn_angle_by_segment.size()) ? plan.turn_angle_by_segment[segment_index] : 180.0;
  assignment->flipped_from_previous = false;
  assignment->flip_reason = LaneFlipReason::kNone;
  const bool previous_orientation_flip =
      (segment_index > 0) ? (plan.node_order_choices[segment_index - 1] != plan.node_order_choices[segment_index])
                          : false;
  if (segment_index > 0 && chosen_orientation_flip != previous_orientation_flip &&
      (assignment->turn_angle_deg + kAngleEps < kReverseStraightAngleDeg)) {
    assignment->flipped_from_previous = true;
    if (assignment->turn_angle_deg + kAngleEps < state_.layout_settings().corner_threshold_deg) {
      assignment->flip_reason = LaneFlipReason::kAcuteTurn;
    }
  }

  if (order_decision_policy_ != OrderDecisionPolicyKind::kPermutableHomogeneous) {
    assignment->order_decision_choice_reason_a = OrderDecisionChoiceReason::kFixedOrder;
    assignment->order_decision_choice_reason_b = OrderDecisionChoiceReason::kFixedOrder;
    return;
  }

  auto evaluate_increment = [&](const std::vector<ObjectId>& lanes_a,
                                const std::vector<ObjectId>& lanes_b) -> BundleOrderScore {
    const ObjectId node_a = ctx_.node_ids[segment_index];
    const ObjectId node_b = ctx_.node_ids[segment_index + 1];
    const Pole* pole_a = ctx_.support_pole(node_a);
    const Pole* pole_b = ctx_.support_pole(node_b);
    const Vec3d pos_a = ctx_.support_position(node_a);
    const Vec3d pos_b = ctx_.support_position(node_b);
    BundleOrderScore score{};

    Vec3d segment_dir{1.0, 0.0, 0.0};
    if (((pos_b.x - pos_a.x) * (pos_b.x - pos_a.x) + (pos_b.y - pos_a.y) * (pos_b.y - pos_a.y) +
         (pos_b.z - pos_a.z) * (pos_b.z - pos_a.z)) > 1e-12) {
      segment_dir = pos_b - pos_a;
      if (!normalize_xy(&segment_dir) || !std::isfinite(segment_dir.x) || !std::isfinite(segment_dir.y)) {
        segment_dir = {1.0, 0.0, 0.0};
      }
    }
    const Vec3d lateral_axis{-segment_dir.y, segment_dir.x, 0.0};
    auto axis_for_node = [&](ObjectId node_id, ObjectId peer_id) -> Vec3d {
      const Vec3d axis = orientation_.CanonicalSideAxisForOrder(node_id, peer_id);
      return (axis.x != 0.0 || axis.y != 0.0) ? axis : lateral_axis;
    };
    const Vec3d axis_a = axis_for_node(node_a, node_b);
    const Vec3d axis_b = axis_for_node(node_b, node_a);
    const double y_sign_b = (dot_xy(axis_a, axis_b) < 0.0) ? -1.0 : 1.0;

    std::vector<double> y_a(static_cast<std::size_t>(lane_count_), 0.0);
    std::vector<double> y_b(static_cast<std::size_t>(lane_count_), 0.0);
    std::vector<double> z_a(static_cast<std::size_t>(lane_count_), 0.0);
    std::vector<double> z_b(static_cast<std::size_t>(lane_count_), 0.0);
    for (int lane = 0; lane < lane_count_; ++lane) {
      const std::size_t idx = static_cast<std::size_t>(lane);
      const auto world_a = state_.port_world_position(lanes_a[idx]);
      const auto world_b = state_.port_world_position(lanes_b[idx]);
      const auto template_layer_a = state_.port_template_layer(lanes_a[idx]);
      const auto template_layer_b = state_.port_template_layer(lanes_b[idx]);
      if (!world_a || !world_b || !template_layer_a || !template_layer_b) {
        score.layer_jump += 4;
        score.span_z_delta += 5.0;
        continue;
      }
      if (use_lane_row_geometry_) {
        const Vec3d local_a =
            (pole_a == nullptr) ? *world_a
                                : WorldPointToLocal(BuildPoleFrame(pole_a->world_transform, LayoutYawForPole(*pole_a)),
                                                    *world_a);
        const Vec3d local_b =
            (pole_b == nullptr) ? *world_b
                                : WorldPointToLocal(BuildPoleFrame(pole_b->world_transform, LayoutYawForPole(*pole_b)),
                                                    *world_b);
        y_a[idx] = local_a.y;
        y_b[idx] = local_b.y * y_sign_b;
      } else {
        y_a[idx] = dot_xy(*world_a - pos_a, axis_a);
        y_b[idx] = dot_xy(*world_b - pos_b, axis_b) * y_sign_b;
      }
      z_a[idx] = world_a->z;
      z_b[idx] = world_b->z;
      score.layer_jump += std::abs(*template_layer_a - *template_layer_b);
      score.span_z_delta += std::abs(world_a->z - world_b->z);
    }
    for (int i = 0; i < lane_count_; ++i) {
      for (int j = i + 1; j < lane_count_; ++j) {
        const std::size_t ii = static_cast<std::size_t>(i);
        const std::size_t jj = static_cast<std::size_t>(j);
        const auto world_ai = state_.port_world_position(lanes_a[ii]);
        const auto world_bi = state_.port_world_position(lanes_b[ii]);
        const auto world_aj = state_.port_world_position(lanes_a[jj]);
        const auto world_bj = state_.port_world_position(lanes_b[jj]);
        if (world_ai && world_bi && world_aj && world_bj &&
            segments_intersect_xy_strict_local(*world_ai, *world_bi, *world_aj, *world_bj)) {
          ++score.segment_xy_intersections;
        }
        constexpr double kOrderEps = 1e-4;
        const double dy_a = y_a[ii] - y_a[jj];
        const double dy_b = y_b[ii] - y_b[jj];
        if ((dy_a > kOrderEps && dy_b < -kOrderEps) || (dy_a < -kOrderEps && dy_b > kOrderEps)) {
          ++score.cross_y;
        }
        const double dz_a = z_a[ii] - z_a[jj];
        const double dz_b = z_b[ii] - z_b[jj];
        if ((dz_a > kOrderEps && dz_b < -kOrderEps) || (dz_a < -kOrderEps && dz_b > kOrderEps)) {
          ++score.cross_z;
        }
      }
    }
    return score;
  };

  auto build_local_order_score = [&](int parity_a, int parity_b, int prev_parity, bool has_prev_segment) {
    OrientationPlanScore score{};
    const std::vector<ObjectId> ports_a =
        order_for_choice(plan.base_ports_by_node[segment_index], choice_for_parity(parity_a));
    const std::vector<ObjectId> ports_b =
        order_for_choice(plan.base_ports_by_node[segment_index + 1], choice_for_parity(parity_b));
    score.order = evaluate_increment(ports_a, ports_b);
    if (use_lane_row_geometry_ && has_prev_segment) {
      const std::vector<ObjectId> prev_ports =
          order_for_choice(plan.base_ports_by_node[segment_index - 1], choice_for_parity(prev_parity));
      for (int i = 0; i < lane_count_; ++i) {
        const std::size_t ii = static_cast<std::size_t>(i);
        const auto curr_a_world = state_.port_world_position(ports_a[ii]);
        const auto curr_b_world = state_.port_world_position(ports_b[ii]);
        if (!curr_a_world || !curr_b_world) {
          continue;
        }
        for (int j = 0; j < lane_count_; ++j) {
          if (i == j) {
            continue;
          }
          const std::size_t jj = static_cast<std::size_t>(j);
          const auto prev_a_world = state_.port_world_position(prev_ports[jj]);
          const auto prev_b_world = state_.port_world_position(ports_a[jj]);
          if (prev_a_world && prev_b_world &&
              segments_intersect_xy_strict_local(*prev_a_world, *prev_b_world, *curr_a_world, *curr_b_world)) {
            ++score.adjacent_xy_intersections;
          }
        }
      }
    }
    if (has_prev_segment) {
      const int prev_orientation = prev_parity ^ parity_a;
      const int curr_orientation = parity_a ^ parity_b;
      if (curr_orientation != prev_orientation &&
          (plan.turn_angle_by_segment[segment_index] + kAngleEps < kReverseStraightAngleDeg)) {
        ++score.orientation_flips;
        if (plan.turn_angle_by_segment[segment_index] + kAngleEps <
            state_.layout_settings().corner_threshold_deg) {
          ++score.acute_orientation_flips;
        }
      }
    }
    return score;
  };

  const int parity_a = (assignment->order_decision_choice_a == OrderDecisionChoiceKind::kReversed) ? 1 : 0;
  const int parity_b = (assignment->order_decision_choice_b == OrderDecisionChoiceKind::kReversed) ? 1 : 0;
  const int prev_parity =
      (segment_index > 0 && plan.node_order_choices[segment_index - 1] == OrderDecisionChoiceKind::kReversed) ? 1 : 0;
  const OrientationPlanScore chosen_score =
      build_local_order_score(parity_a, parity_b, prev_parity, segment_index > 0);
  const OrientationPlanScore alternate_a =
      build_local_order_score(parity_a ^ 1, parity_b, prev_parity, segment_index > 0);
  const OrientationPlanScore alternate_b =
      build_local_order_score(parity_a, parity_b ^ 1, prev_parity, segment_index > 0);
  assignment->order_decision_choice_reason_a =
      order_choice_reason_from_scores(chosen_score, alternate_a, order_decision_policy_, use_lane_row_geometry_);
  assignment->order_decision_choice_reason_b =
      order_choice_reason_from_scores(chosen_score, alternate_b, order_decision_policy_, use_lane_row_geometry_);
}

EditResult<std::vector<ObjectId>>
GroupedSpanLanePreparer::EnsurePorts(ObjectId node_id, ObjectId peer_id, int segment_index,
                                     bool prefer_existing_neighbor_order, bool* out_seeded_from_previous) {
  EditResult<std::vector<ObjectId>> ports_result;
  const std::pair<ObjectId, ObjectId> cache_key{node_id, peer_id};
  if (out_seeded_from_previous != nullptr) {
    *out_seeded_from_previous = false;
  }
  if (const auto it_cached = node_lane_ports_cache_.find(cache_key); it_cached != node_lane_ports_cache_.end()) {
    if (static_cast<int>(it_cached->second.size()) == lane_count_) {
      ports_result.value = it_cached->second;
      ports_result.ok = true;
      return ports_result;
    }
  }

  auto append_state_change_set = [&](const ChangeSet& src) {
    if (change_set_ != nullptr) {
      append_change_set(*change_set_, src);
    }
  };
  auto mark_updated = [&](ObjectId id) {
    if (change_set_ != nullptr) {
      state_.add_unique_id(change_set_->updated_ids, id);
    }
  };

  const SupportKind kind = ctx_.support_kind(node_id);
  const Pole* pole = ctx_.support_pole(node_id);
  if (kind != SupportKind::kPole || pole == nullptr) {
    const Vec3d base_position = ctx_.support_position(node_id);
    const Vec3d side_axis = orientation_.GroupedLineAxisForEndpoint(node_id, peer_id);
    auto sort_nonpole_ports = [&](std::vector<ObjectId>* port_ids) {
      if (port_ids == nullptr) {
        return;
      }
      std::sort(port_ids->begin(), port_ids->end(), [&](ObjectId a, ObjectId b) {
        const auto pa = state_.port_world_position(a);
        const auto pb = state_.port_world_position(b);
        const auto key_a =
            pa ? std::tuple<double, double, ObjectId>(dot_xy(*pa - base_position, side_axis), pa->z, a)
               : std::tuple<double, double, ObjectId>(0.0, 0.0, kInvalidObjectId);
        const auto key_b =
            pb ? std::tuple<double, double, ObjectId>(dot_xy(*pb - base_position, side_axis), pb->z, b)
               : std::tuple<double, double, ObjectId>(0.0, 0.0, kInvalidObjectId);
        return key_a < key_b;
      });
    };

    std::unordered_set<ObjectId> unique_nonpole_ports{};
    state_.for_each_span_in_bundle_template(bundle_template_id_, [&](const Span& span) {
      const auto sample = state_.port_segment_sample(span);
      if (!sample) {
        return;
      }
      if (ctx_.resolve_span_endpoint_node(span, sample->owner_pole_a_id, true) == node_id &&
          sample->layer_a == target_port_layer_ && unique_nonpole_ports.insert(sample->port_a_id).second) {
        ports_result.value.push_back(sample->port_a_id);
      }
      if (ctx_.resolve_span_endpoint_node(span, sample->owner_pole_b_id, false) == node_id &&
          sample->layer_b == target_port_layer_ && unique_nonpole_ports.insert(sample->port_b_id).second) {
        ports_result.value.push_back(sample->port_b_id);
      }
    });
    sort_nonpole_ports(&ports_result.value);
    if (static_cast<int>(ports_result.value.size()) > lane_count_) {
      ports_result.value.resize(static_cast<std::size_t>(lane_count_));
    }

    const auto it_support = ctx_.support_node_by_id.find(node_id);
    if (static_cast<int>(ports_result.value.size()) < lane_count_ && it_support != ctx_.support_node_by_id.end() &&
        it_support->second.has_source_edge) {
      struct SourcePortSample {
        Vec3d world{};
        double side = 0.0;
      };
      std::vector<SourcePortSample> samples{};
      const SupportNode& support = it_support->second;
      state_.for_each_span_in_bundle_template(bundle_template_id_, [&](const Span& span) {
        const auto sample = state_.port_segment_sample(span);
        if (!sample || sample->layer_a != target_port_layer_ || sample->layer_b != target_port_layer_) {
          return;
        }
        const ObjectId endpoint_a = ctx_.resolve_span_endpoint_node(span, sample->owner_pole_a_id, true);
        const ObjectId endpoint_b = ctx_.resolve_span_endpoint_node(span, sample->owner_pole_b_id, false);
        double t = support.source_edge_t;
        if (endpoint_a == support.source_edge_node_a_id && endpoint_b == support.source_edge_node_b_id) {
        } else if (endpoint_a == support.source_edge_node_b_id && endpoint_b == support.source_edge_node_a_id) {
          t = 1.0 - t;
        } else {
          return;
        }
        const Vec3d world{
            sample->world_a.x + (sample->world_b.x - sample->world_a.x) * t,
            sample->world_a.y + (sample->world_b.y - sample->world_a.y) * t,
            sample->world_a.z + (sample->world_b.z - sample->world_a.z) * t,
        };
        samples.push_back({world, dot_xy(world - base_position, side_axis)});
      });
      std::sort(samples.begin(), samples.end(), [](const SourcePortSample& a, const SourcePortSample& b) {
        if (std::abs(a.side - b.side) > 1e-9) {
          return a.side < b.side;
        }
        if (std::abs(a.world.z - b.world.z) > 1e-9) {
          return a.world.z < b.world.z;
        }
        if (std::abs(a.world.y - b.world.y) > 1e-9) {
          return a.world.y < b.world.y;
        }
        return a.world.x < b.world.x;
      });
      for (const SourcePortSample& sample : samples) {
        if (static_cast<int>(ports_result.value.size()) >= lane_count_) {
          break;
        }
      EditResult<ObjectId> add_port =
          state_.AddPort(kInvalidObjectId, sample.world, state_.port_kind_for_category(category_),
                         target_port_layer_);
        if (!add_port.ok) {
          ports_result.error = add_port.error;
          return ports_result;
        }
        append_state_change_set(add_port.change_set);
        if (state_.mark_aerial_branch_port(add_port.value)) {
          mark_updated(add_port.value);
        }
        ports_result.value.push_back(add_port.value);
      }
    }

    ports_result.value.reserve(static_cast<std::size_t>(lane_count_));
    while (static_cast<int>(ports_result.value.size()) < lane_count_) {
      EditResult<ObjectId> add_port =
          state_.AddPort(kInvalidObjectId, base_position, state_.port_kind_for_category(category_),
                         target_port_layer_);
      if (!add_port.ok) {
        ports_result.error = add_port.error;
        return ports_result;
      }
      append_state_change_set(add_port.change_set);
      if (state_.mark_aerial_branch_port(add_port.value)) {
        mark_updated(add_port.value);
      }
      ports_result.value.push_back(add_port.value);
    }
    sort_nonpole_ports(&ports_result.value);
    node_lane_ports_cache_[cache_key] = ports_result.value;
    ports_result.ok = true;
    return ports_result;
  }

  const bool use_outboard_lowered_ports_here = lowering_.EndpointRequiresOutboardLoweredPorts(node_id, peer_id);
  auto desired_scaffold_world_for_target = [&](double target_y,
                                               const SegmentRelationFeasibility& feasibility) -> Vec3d {
    const double base_z_m = LaneRowTargetZForEndpoint(*pole, feasibility);
    const double layout_yaw = LayoutYawForPole(*pole);
    const Vec3d local{0.0, target_y, base_z_m};
    if (!use_lane_row_geometry_) {
      return local_to_world_on_pole_local(pole->world_transform, layout_yaw, local);
    }

    const auto it_index = std::find(ctx_.node_ids.begin(), ctx_.node_ids.end(), node_id);
    const std::size_t node_index = (it_index == ctx_.node_ids.end())
                                       ? ctx_.node_ids.size()
                                       : static_cast<std::size_t>(it_index - ctx_.node_ids.begin());
    const bool is_terminal_node = (node_index == 0 || node_index + 1 >= ctx_.node_ids.size());
    if (is_terminal_node || node_index == ctx_.node_ids.size()) {
      return local_to_world_on_pole_local(pole->world_transform, layout_yaw, local);
    }
    const EndpointSideDecision scaffold_side_decision =
        orientation_.PreferredSideAxisForEndpoint(node_id, peer_id, feasibility);
    const Vec3d stable_side_axis =
        scaffold_side_decision.has_side_axis ? scaffold_side_decision.side_axis
                                             : orientation_.GroupedLineAxisForEndpoint(node_id, peer_id);
    const Vec3d base = pole->world_transform.position;
    const Vec3d prev = ctx_.support_position(ctx_.node_ids[node_index - 1]);
    const Vec3d next = ctx_.support_position(ctx_.node_ids[node_index + 1]);
    if ((prev - base).x * (prev - base).x + (prev - base).y * (prev - base).y <= 1e-12 ||
        (next - base).x * (next - base).x + (next - base).y * (next - base).y <= 1e-12) {
      return local_to_world_on_pole_local(pole->world_transform, layout_yaw, local);
    }

    Vec3d dir_in = base - prev;
    Vec3d dir_out = next - base;
    if (!normalize_xy(&dir_in) || !normalize_xy(&dir_out)) {
      return local_to_world_on_pole_local(pole->world_transform, layout_yaw, local);
    }
    Vec3d normal_in = ComputeLateralAxis(dir_in);
    Vec3d normal_out = ComputeLateralAxis(dir_out);
    if (!normalize_xy(&normal_in)) {
      normal_in = stable_side_axis;
    }
    if (!normalize_xy(&normal_out)) {
      normal_out = stable_side_axis;
    }
    if (dot_xy(normal_in, stable_side_axis) < 0.0) {
      normal_in.x = -normal_in.x;
      normal_in.y = -normal_in.y;
    }
    if (dot_xy(normal_out, stable_side_axis) < 0.0) {
      normal_out.x = -normal_out.x;
      normal_out.y = -normal_out.y;
    }

    Vec3d joined_xy{};
    const Vec3d offset_in{base.x + normal_in.x * target_y, base.y + normal_in.y * target_y, HeightAlongWorldUp(base)};
    const Vec3d offset_out{base.x + normal_out.x * target_y, base.y + normal_out.y * target_y,
                           HeightAlongWorldUp(base)};
    if (line_intersection_xy_local(offset_in, dir_in, offset_out, dir_out, &joined_xy)) {
      SetHeightAlongWorldUp(&joined_xy, HeightAlongWorldUp(base) + base_z_m);
      const double dx = joined_xy.x - base.x;
      const double dy = joined_xy.y - base.y;
      const double dist = std::sqrt(dx * dx + dy * dy);
      const double spacing = lowering_.LaneSpacingForEndpoint(feasibility);
      const double limit = std::max(spacing * 8.0, std::abs(target_y) * 8.0 + 0.2);
      if (dist <= limit || dist <= 1e-9) {
        return joined_xy;
      }
      const double scale = limit / dist;
      return Vec3d{base.x + dx * scale, base.y + dy * scale, HeightAlongWorldUp(base) + base_z_m};
    }

    return local_to_world_on_pole_local(pole->world_transform, layout_yaw, local);
  };

  auto try_solver_ports_for_pole = [&](ConnectionContext solver_context, SlotRole preferred_role,
                                       const SegmentRelationFeasibility& feasibility,
                                       std::vector<ObjectId>* out_ports) -> bool {
    if (pole == nullptr || out_ports == nullptr || feasibility.same_level_feasible) {
      return false;
    }
    const Pole* peer_pole = ctx_.support_pole(peer_id);
    const double spacing = lowering_.LaneSpacingForEndpoint(feasibility);
    const double center = (static_cast<double>(lane_count_) - 1.0) * 0.5;
    std::unordered_set<ObjectId> used_ports{};
    std::vector<ObjectId> solved_ports{};
    solved_ports.reserve(static_cast<std::size_t>(lane_count_));
    for (int lane = 0; lane < lane_count_; ++lane) {
      const double target_y = (static_cast<double>(lane) - center) * spacing;
      PortResolutionRequest request{};
      request.pole_id = node_id;
      request.peer_pole_id = (peer_pole == nullptr) ? kInvalidObjectId : peer_pole->id;
      request.category = category_;
      request.connection_context = solver_context;
      request.pole_context = pole->context.kind;
      request.corner_angle_deg = pole->context.corner_angle_deg;
      request.corner_turn_sign = pole->context.corner_turn_sign;
      request.allow_generate_port = true;
      request.prefer_template_match = true;
      request.preferred_template_layer = state_.template_layer_for_category(category_);
      request.preferred_template_side =
          (target_y < -1e-9) ? SlotSide::kLeft : ((target_y > 1e-9) ? SlotSide::kRight : SlotSide::kCenter);
      request.preferred_template_role = preferred_role;
      request.branch_index = static_cast<std::uint32_t>(lane);
      request.hints.relation_kind = feasibility.kind;
      request.hints.continuity_class = feasibility.continuity_class;
      request.hints.default_lower_required = feasibility.default_lower_required;
      request.hints.same_level_feasible = feasibility.same_level_feasible;
      request.hints.same_level_reason = feasibility.reason;
      request.hints.projected_spacing_topview_m = feasibility.projected_spacing_topview_m;
      request.hints.required_clearance_m = feasibility.required_clearance_m;
      request.excluded_port_ids.assign(solved_ports.begin(), solved_ports.end());
      EditResult<ObjectId> port_result = state_.ensure_pole_connection_port(request);
      if (!port_result.ok) {
        return false;
      }
      const auto source = state_.port_placement_source(port_result.value);
      if (!state_.port_matches_owner_layer(port_result.value, node_id, target_port_layer_) || !source ||
          (*source != PortPlacementSourceKind::kPlacementBand &&
           *source != PortPlacementSourceKind::kPlacementBandConstrained) ||
          !used_ports.insert(port_result.value).second) {
        return false;
      }
      solved_ports.push_back(port_result.value);
    }
    const double layout_yaw = LayoutYawForPole(*pole);
    std::sort(solved_ports.begin(), solved_ports.end(), [&](ObjectId a, ObjectId b) {
      const double ya = state_.port_local_y(a, pole, layout_yaw).value_or(0.0);
      const double yb = state_.port_local_y(b, pole, layout_yaw).value_or(0.0);
      if (std::abs(ya - yb) > 1e-9) {
        return ya < yb;
      }
      return a < b;
    });
    if (static_cast<int>(solved_ports.size()) == lane_count_ && !solved_ports.empty()) {
      const double normalize_center = (static_cast<double>(lane_count_) - 1.0) * 0.5;
      for (int lane = 0; lane < lane_count_; ++lane) {
        const ObjectId port_id = solved_ports[static_cast<std::size_t>(lane)];
        const double target_y = (static_cast<double>(lane) - normalize_center) * spacing;
        const SlotSide template_side =
            (target_y < -1e-9) ? SlotSide::kLeft : ((target_y > 1e-9) ? SlotSide::kRight : SlotSide::kCenter);
        const Vec3d desired_world = desired_scaffold_world_for_target(target_y, feasibility);
        if (state_.finalize_constrained_solver_port(
                port_id, desired_world, category_, state_.template_layer_for_category(category_), template_side)) {
          mark_updated(port_id);
        }
      }
    }
    *out_ports = std::move(solved_ports);
    return static_cast<int>(out_ports->size()) == lane_count_;
  };

  if (use_outboard_lowered_ports_here) {
    const SegmentRelationFeasibility branch_feasibility = lowering_.SegmentRelationFeasibilityFor(node_id, peer_id);
    if (branch_feasibility.kind == JunctionRelationKind::kCrossUnderpass &&
        try_solver_ports_for_pole(ConnectionContext::kBranchAdd, SlotRole::kBranchPreferred, branch_feasibility,
                                  &ports_result.value)) {
      node_lane_ports_cache_[cache_key] = ports_result.value;
      ports_result.ok = true;
      return ports_result;
    }
    const Vec3d peer_delta = ctx_.support_position(peer_id) - pole->world_transform.position;
    Vec3d branch_forward_axis = peer_delta;
    if (!normalize_xy(&branch_forward_axis)) {
      branch_forward_axis = {1.0, 0.0, 0.0};
    }
    const Vec3d branch_row_axis = ComputeLateralAxis(branch_forward_axis);
    const EndpointSideDecision preferred_side_decision =
        orientation_.PreferredSideAxisForEndpoint(node_id, peer_id, branch_feasibility);
    const Vec3d side_axis = preferred_side_decision.has_side_axis ? preferred_side_decision.side_axis : branch_row_axis;
    Vec3d peer_dir = peer_delta;
    peer_dir.z = 0.0;
    double side_sign = preferred_side_decision.chosen_side_sign;
    if (std::abs(side_sign) <= 1e-9 &&
        preferred_side_decision.side_assignment_rule == SideAssignmentRuleKind::kThroughPairNormal &&
        normalize_xy(&peer_dir)) {
      const double along = dot_xy(peer_dir, side_axis);
      side_sign = (std::abs(along) <= 1e-9) ? 1.0 : ((along >= 0.0) ? 1.0 : -1.0);
    } else if (std::abs(side_sign) <= 1e-9) {
      side_sign = (dot_xy(side_axis, branch_row_axis) >= 0.0) ? 1.0 : -1.0;
      if (std::abs(dot_xy(side_axis, branch_row_axis)) <= 1e-9) {
        side_sign = 1.0;
      }
    }
    const SlotSide branch_side = (side_sign >= 0.0) ? SlotSide::kRight : SlotSide::kLeft;
    const double branch_support_yaw_deg = std::atan2(branch_forward_axis.y, branch_forward_axis.x) * (180.0 / kPi);
    const double branch_base_z_m = LaneRowTargetZForEndpoint(*pole, branch_feasibility);
    const double lane_spacing = lowering_.LaneSpacingForEndpoint(branch_feasibility);
    const double center = (static_cast<double>(lane_count_) - 1.0) * 0.5;
    const double half_span = center * lane_spacing;
    const double min_outboard = state_.pole_radius_at_height_m(*pole, branch_base_z_m) +
                                state_.geometry_settings().pole_clearance_m + half_span + 0.25;
    const double branch_support_offset_m = std::max(0.65, min_outboard);
    ports_result.value.reserve(static_cast<std::size_t>(lane_count_));
    for (int lane = 0; lane < lane_count_; ++lane) {
      const double lane_offset = (static_cast<double>(lane) - center) * lane_spacing;
      const double lane_z_m = branch_base_z_m;
      Vec3d local{0.0, side_sign * branch_support_offset_m + lane_offset, lane_z_m};
      local = state_.apply_pole_clearance_to_local(*pole, local, branch_side);
      const Vec3d world = local_to_world_on_pole_local(pole->world_transform, branch_support_yaw_deg, local);
      EditResult<ObjectId> add_port =
          state_.AddPort(node_id, world, state_.port_kind_for_category(category_),
                         state_.port_layer_for_category(category_));
      if (!add_port.ok) {
        ports_result.error = add_port.error;
        return ports_result;
      }
      append_state_change_set(add_port.change_set);
      if (state_.finalize_branch_support_port(add_port.value, category_,
                                              state_.template_layer_for_category(category_), branch_side)) {
        mark_updated(add_port.value);
      }
      ports_result.value.push_back(add_port.value);
    }
    std::sort(ports_result.value.begin(), ports_result.value.end(), [&](ObjectId a, ObjectId b) {
      const double ya = state_.port_local_y(a, pole, branch_support_yaw_deg).value_or(0.0);
      const double yb = state_.port_local_y(b, pole, branch_support_yaw_deg).value_or(0.0);
      if (std::abs(ya - yb) > 1e-9) {
        return ya < yb;
      }
      return a < b;
    });
    node_lane_ports_cache_[cache_key] = ports_result.value;
    ports_result.ok = true;
    return ports_result;
  }

  if (use_lane_row_geometry_ && pole != nullptr) {
    const SegmentRelationFeasibility relation_feasibility = lowering_.SegmentRelationFeasibilityFor(node_id, peer_id);
    if (!relation_feasibility.same_level_feasible) {
      const ConnectionContext solver_context =
          (relation_feasibility.kind == JunctionRelationKind::kCornerContinuation)
              ? ConnectionContext::kCornerPass
              : ((relation_feasibility.kind == JunctionRelationKind::kSideBranch ||
                  relation_feasibility.kind == JunctionRelationKind::kCrossUnderpass)
                     ? ConnectionContext::kBranchAdd
                     : ConnectionContext::kTrunkContinue);
      const SlotRole preferred_role =
          (relation_feasibility.kind == JunctionRelationKind::kSideBranch ||
           relation_feasibility.kind == JunctionRelationKind::kCrossUnderpass)
              ? SlotRole::kBranchPreferred
              : SlotRole::kNeutral;
      if (try_solver_ports_for_pole(solver_context, preferred_role, relation_feasibility, &ports_result.value)) {
        node_lane_ports_cache_[cache_key] = ports_result.value;
        ports_result.ok = true;
        return ports_result;
      }
    }
    std::unordered_set<ObjectId> unique{};
    std::vector<ObjectId> reusable_generated_ports{};

    state_.for_each_port_owned_by_pole(node_id, [&](const Port& port) {
      if (port.layer != target_port_layer_ || port.category != category_) {
        return;
      }
      if (port.generated_by_rule && PortConnectionCount(port.id) == 0) {
        reusable_generated_ports.push_back(port.id);
      }
    });

    const double target_z_m = LaneRowTargetZForEndpoint(*pole, relation_feasibility);
    const double spacing = lowering_.LaneSpacingForEndpoint(relation_feasibility);
    const double center = (static_cast<double>(lane_count_) - 1.0) * 0.5;
    for (int lane = 0; lane < lane_count_; ++lane) {
      const double target_y = (static_cast<double>(lane) - center) * spacing;
      const Vec3d local{0.0, target_y, target_z_m};
      const Vec3d world = local_to_world_on_pole_local(pole->world_transform, LayoutYawForPole(*pole), local);

      ObjectId port_id = kInvalidObjectId;
      while (!reusable_generated_ports.empty() && port_id == kInvalidObjectId) {
        const ObjectId candidate_id = reusable_generated_ports.back();
        reusable_generated_ports.pop_back();
        if (unique.find(candidate_id) == unique.end()) {
          port_id = candidate_id;
        }
      }

      if (port_id != kInvalidObjectId) {
        const SlotSide template_side =
            (target_y < -1e-9) ? SlotSide::kLeft : ((target_y > 1e-9) ? SlotSide::kRight : SlotSide::kCenter);
        if (!state_.finalize_generated_row_port(port_id, &world, category_,
                                                state_.template_layer_for_category(category_), template_side)) {
          continue;
        }
        mark_updated(port_id);
        unique.insert(port_id);
        ports_result.value.push_back(port_id);
        continue;
      }

        EditResult<ObjectId> add_port =
          state_.AddPort(node_id, world, state_.port_kind_for_category(category_),
                         state_.port_layer_for_category(category_));
      if (!add_port.ok) {
        ports_result.error = add_port.error;
        return ports_result;
      }
      append_state_change_set(add_port.change_set);
      const SlotSide template_side =
          (target_y < -1e-9) ? SlotSide::kLeft : ((target_y > 1e-9) ? SlotSide::kRight : SlotSide::kCenter);
      if (state_.finalize_generated_row_port(add_port.value, nullptr, category_,
                                             state_.template_layer_for_category(category_), template_side)) {
        mark_updated(add_port.value);
      }
      unique.insert(add_port.value);
      ports_result.value.push_back(add_port.value);
    }
  } else {
    state_.for_each_port_owned_by_pole(node_id, [&](const Port& port) {
      if (port.layer == target_port_layer_) {
        ports_result.value.push_back(port.id);
      }
    });
  }

  auto port_links_to_neighbor = [&](ObjectId port_id, ObjectId neighbor_node_id) -> int {
    int count = 0;
    state_.for_each_connected_span(port_id, [&](const Span& span) {
      const ObjectId other_port_id = (span.port_a_id == port_id) ? span.port_b_id : span.port_a_id;
      if (state_.port_matches_owner_layer(other_port_id, neighbor_node_id, target_port_layer_)) {
        ++count;
      }
    });
    return count;
  };

  ObjectId continuity_neighbor_id = kInvalidObjectId;
  if (prefer_existing_neighbor_order) {
    std::unordered_map<ObjectId, int> neighbor_counts{};
    for (ObjectId port_id : ports_result.value) {
      state_.for_each_connected_span(port_id, [&](const Span& span) {
        const ObjectId other_port_id = (span.port_a_id == port_id) ? span.port_b_id : span.port_a_id;
        const auto other_node_id = state_.port_owner_pole_id_on_layer(other_port_id, target_port_layer_);
        if (!other_node_id) {
          return;
        }
        if (*other_node_id == kInvalidObjectId || *other_node_id == node_id || *other_node_id == peer_id) {
          return;
        }
        neighbor_counts[*other_node_id] += 1;
      });
    }
    int best_count = 0;
    for (const auto& [neighbor_id, count] : neighbor_counts) {
      if (count > best_count || (count == best_count && count > 0 && neighbor_id < continuity_neighbor_id)) {
        best_count = count;
        continuity_neighbor_id = neighbor_id;
      }
    }
  }
  if (static_cast<int>(ports_result.value.size()) < lane_count_) {
    ports_result.error = "insufficient ports for grouped generation";
    return ports_result;
  }
  if (prefer_existing_neighbor_order && continuity_neighbor_id != kInvalidObjectId) {
    std::vector<ObjectId> seeded_order{};
    std::unordered_set<ObjectId> unique_seeded_ports{};
    seeded_order.reserve(static_cast<std::size_t>(lane_count_));
    for (ObjectId port_id : ports_result.value) {
      if (!state_.port_matches_owner_layer(port_id, node_id, target_port_layer_) ||
          port_links_to_neighbor(port_id, continuity_neighbor_id) <= 0) {
        continue;
      }
      if (!unique_seeded_ports.insert(port_id).second) {
        continue;
      }
      seeded_order.push_back(port_id);
    }
    if (static_cast<int>(seeded_order.size()) >= lane_count_) {
      const double layout_yaw = LayoutYawForPole(*pole);
      std::sort(seeded_order.begin(), seeded_order.end(), [&](ObjectId a, ObjectId b) {
        return state_.port_order_key(a, pole, layout_yaw) < state_.port_order_key(b, pole, layout_yaw);
      });
      seeded_order.resize(static_cast<std::size_t>(lane_count_));
      ports_result.value = std::move(seeded_order);
      node_lane_ports_cache_[cache_key] = ports_result.value;
      if (out_seeded_from_previous != nullptr) {
        *out_seeded_from_previous = true;
      }
      ports_result.ok = true;
      return ports_result;
    }
  }

  const bool use_scaffold_layout = use_lane_row_geometry_;
  if (use_scaffold_layout && pole != nullptr) {
    const SegmentRelationFeasibility scaffold_feasibility = lowering_.SegmentRelationFeasibilityFor(node_id, peer_id);
    const double spacing = lowering_.LaneSpacingForEndpoint(scaffold_feasibility);
    const double center = (static_cast<double>(lane_count_) - 1.0) * 0.5;
    std::vector<double> target_local_y(static_cast<std::size_t>(lane_count_), 0.0);
    for (int lane = 0; lane < lane_count_; ++lane) {
      target_local_y[static_cast<std::size_t>(lane)] = (static_cast<double>(lane) - center) * spacing;
    }
    const EndpointSideDecision scaffold_side_decision =
        orientation_.PreferredSideAxisForEndpoint(node_id, peer_id, scaffold_feasibility);
    const Vec3d stable_side_axis =
        scaffold_side_decision.has_side_axis ? scaffold_side_decision.side_axis
                                             : orientation_.GroupedLineAxisForEndpoint(node_id, peer_id);
    const auto it_index = std::find(ctx_.node_ids.begin(), ctx_.node_ids.end(), node_id);
    const std::size_t node_index = (it_index == ctx_.node_ids.end())
                                       ? ctx_.node_ids.size()
                                       : static_cast<std::size_t>(it_index - ctx_.node_ids.begin());
    const bool is_terminal_node = (node_index == 0 || node_index + 1 >= ctx_.node_ids.size());
    if (is_terminal_node) {
      const double layout_yaw = LayoutYawForPole(*pole);
      std::sort(ports_result.value.begin(), ports_result.value.end(), [&](ObjectId a, ObjectId b) {
        return state_.port_terminal_order_key(a, pole, layout_yaw) <
               state_.port_terminal_order_key(b, pole, layout_yaw);
      });

      std::vector<ObjectId> normalized_ports{};
      normalized_ports.reserve(static_cast<std::size_t>(lane_count_));
      std::vector<ObjectId> reusable_fallback_ports{};
      reusable_fallback_ports.reserve(ports_result.value.size());
      for (ObjectId port_id : ports_result.value) {
        if (PortConnectionCount(port_id) == 0) {
          reusable_fallback_ports.push_back(port_id);
        }
      }

      const double target_z_m = LaneRowTargetZForEndpoint(*pole, scaffold_feasibility);
      auto realize_terminal_world = [&](double target_y) {
        const Vec3d local{0.0, target_y, target_z_m};
        return local_to_world_on_pole_local(pole->world_transform, LayoutYawForPole(*pole), local);
      };

      for (int lane = static_cast<int>(normalized_ports.size()); lane < lane_count_; ++lane) {
        const double target_y = target_local_y[static_cast<std::size_t>(lane)];
        const Vec3d world = realize_terminal_world(target_y);
        if (!reusable_fallback_ports.empty()) {
          const ObjectId port_id = reusable_fallback_ports.front();
          reusable_fallback_ports.erase(reusable_fallback_ports.begin());
          const SlotSide template_side =
              (target_y < -1e-9) ? SlotSide::kLeft : ((target_y > 1e-9) ? SlotSide::kRight : SlotSide::kCenter);
          if (state_.finalize_terminal_fallback_port(port_id, &world, category_,
                                                     state_.template_layer_for_category(category_), template_side)) {
            mark_updated(port_id);
            normalized_ports.push_back(port_id);
            continue;
          }
        }

        EditResult<ObjectId> add_port =
            state_.AddPort(node_id, world, state_.port_kind_for_category(category_),
                           state_.port_layer_for_category(category_));
        if (!add_port.ok) {
          ports_result.error = add_port.error;
          return ports_result;
        }
        append_state_change_set(add_port.change_set);
        const SlotSide template_side =
            (target_y < -1e-9) ? SlotSide::kLeft : ((target_y > 1e-9) ? SlotSide::kRight : SlotSide::kCenter);
        if (state_.finalize_terminal_fallback_port(add_port.value, nullptr, category_,
                                                   state_.template_layer_for_category(category_), template_side)) {
          mark_updated(add_port.value);
        }
        normalized_ports.push_back(add_port.value);
      }

      std::sort(normalized_ports.begin(), normalized_ports.end(), [&](ObjectId a, ObjectId b) {
        return state_.port_order_key(a, pole, layout_yaw) < state_.port_order_key(b, pole, layout_yaw);
      });
      if (static_cast<int>(normalized_ports.size()) > lane_count_) {
        normalized_ports.resize(static_cast<std::size_t>(lane_count_));
      }

      ports_result.value = std::move(normalized_ports);
      node_lane_ports_cache_[cache_key] = ports_result.value;
      ports_result.ok = true;
      return ports_result;
    }

    const bool use_scaffold_geometry = use_lane_row_geometry_;
    auto desired_world_for_target = [&](double target_y) {
      return desired_scaffold_world_for_target(target_y, scaffold_feasibility);
    };

    std::vector<ObjectId> ordered_ports(static_cast<std::size_t>(lane_count_), kInvalidObjectId);
    std::unordered_set<ObjectId> used{};
    for (int lane = 0; lane < lane_count_; ++lane) {
      const double target_y = target_local_y[static_cast<std::size_t>(lane)];
      const Vec3d desired_world = desired_world_for_target(target_y);
      ObjectId best_id = kInvalidObjectId;
      double best_dist = std::numeric_limits<double>::max();
      for (ObjectId candidate_id : ports_result.value) {
        if (used.find(candidate_id) != used.end()) {
          continue;
        }
        const auto world = state_.port_world_position(candidate_id);
        if (!world || !state_.port_matches_owner_layer(candidate_id, node_id, target_port_layer_)) {
          continue;
        }
        const double dist =
            use_scaffold_geometry
                ? std::sqrt(std::pow(world->x - desired_world.x, 2.0) + std::pow(world->y - desired_world.y, 2.0))
                : std::abs(dot_xy(*world - pole->world_transform.position, stable_side_axis) - target_y);
        if (dist < best_dist) {
          best_dist = dist;
          best_id = candidate_id;
        }
      }

      const double kTargetMatchTolerance =
          use_scaffold_geometry ? std::max(0.05, spacing * 0.35) : std::max(0.02, spacing * 0.1);
      if (best_id == kInvalidObjectId || best_dist > kTargetMatchTolerance) {
        EditResult<ObjectId> extra =
            state_.AddPort(node_id, desired_world, state_.port_kind_for_category(category_),
                           state_.port_layer_for_category(category_));
        if (!extra.ok) {
          ports_result.error = extra.error;
          return ports_result;
        }
        append_state_change_set(extra.change_set);
        best_id = extra.value;
      }
      if (use_scaffold_geometry) {
        if (state_.port_matches_owner_layer(best_id, node_id, target_port_layer_)) {
          if (state_.reposition_port_world(best_id, desired_world)) {
            mark_updated(best_id);
          }
        }
      }

      used.insert(best_id);
      ordered_ports[static_cast<std::size_t>(lane)] = best_id;
    }

    ports_result.value = std::move(ordered_ports);
    node_lane_ports_cache_[cache_key] = ports_result.value;
    ports_result.ok = true;
    return ports_result;
  }

  std::sort(ports_result.value.begin(), ports_result.value.end(), [&](ObjectId a, ObjectId b) {
    const double layout_yaw = (pole != nullptr) ? LayoutYawForPole(*pole) : 0.0;
    return state_.port_order_key(a, pole, layout_yaw) < state_.port_order_key(b, pole, layout_yaw);
  });

  if (static_cast<int>(ports_result.value.size()) > lane_count_) {
    if (continuity_neighbor_id != kInvalidObjectId) {
      std::stable_sort(ports_result.value.begin(), ports_result.value.end(), [&](ObjectId a, ObjectId b) {
        const int score_a = port_links_to_neighbor(a, continuity_neighbor_id);
        const int score_b = port_links_to_neighbor(b, continuity_neighbor_id);
        if (score_a != score_b) {
          return score_a > score_b;
        }
        const double layout_yaw = (pole != nullptr) ? LayoutYawForPole(*pole) : 0.0;
        return state_.port_order_key(a, pole, layout_yaw) < state_.port_order_key(b, pole, layout_yaw);
      });
      ports_result.value.resize(static_cast<std::size_t>(lane_count_));
      std::sort(ports_result.value.begin(), ports_result.value.end(), [&](ObjectId a, ObjectId b) {
        const double layout_yaw = (pole != nullptr) ? LayoutYawForPole(*pole) : 0.0;
        return state_.port_order_key(a, pole, layout_yaw) < state_.port_order_key(b, pole, layout_yaw);
      });
    } else {
      const std::size_t target = static_cast<std::size_t>(lane_count_);
      const std::size_t total = ports_result.value.size();
      std::size_t best_start = 0;
      double best_abs_mean = std::numeric_limits<double>::max();
      double best_span = std::numeric_limits<double>::max();
      for (std::size_t start = 0; start + target <= total; ++start) {
        double sum = 0.0;
        double y_min = std::numeric_limits<double>::max();
        double y_max = -std::numeric_limits<double>::max();
        for (std::size_t i = start; i < start + target; ++i) {
          const auto key = state_.port_order_key(ports_result.value[i], pole, LayoutYawForPole(*pole));
          const double y = key ? key->local_y : 0.0;
          sum += y;
          y_min = std::min(y_min, y);
          y_max = std::max(y_max, y);
        }
        const double abs_mean = std::abs(sum / static_cast<double>(target));
        const double span = y_max - y_min;
        if (abs_mean + 1e-9 < best_abs_mean || (std::abs(abs_mean - best_abs_mean) <= 1e-9 && span < best_span)) {
          best_abs_mean = abs_mean;
          best_span = span;
          best_start = start;
        }
      }
      std::vector<ObjectId> centered{};
      centered.reserve(target);
      for (std::size_t i = best_start; i < best_start + target; ++i) {
        centered.push_back(ports_result.value[i]);
      }
      ports_result.value.swap(centered);
    }
  }
  if (static_cast<int>(ports_result.value.size()) > lane_count_) {
    ports_result.value.resize(static_cast<std::size_t>(lane_count_));
  }
  node_lane_ports_cache_[cache_key] = ports_result.value;
  ports_result.ok = true;
  return ports_result;
}

} // namespace wire::core::generation::detail
