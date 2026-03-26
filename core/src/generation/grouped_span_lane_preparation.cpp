#include "grouped_span_lane_preparation.hpp"

#include "detail_utils.hpp"
#include "support_policy.hpp"
#include "wire/core/core_state.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <tuple>
#include <unordered_map>
#include <unordered_set>

namespace wire::core::generation::detail {

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
  return state_.effective_pole_layout_yaw_deg(pole);
}

double GroupedSpanLanePreparer::TemplateLayerBaseZForPole(const Pole& pole) const {
  double best_z = -std::numeric_limits<double>::infinity();
  const int target_layer = generation::detail::TemplateLayerForCategory(category_);
  if (const PoleTypeDefinition* pole_type = state_.find_pole_type(pole.pole_type_id); pole_type != nullptr) {
    for (const PortPlacementBand& band : pole_type->port_bands) {
      if (!band.enabled) {
        continue;
      }
      if (band.layer == target_layer) {
        best_z = std::max(best_z, band.height_max_m);
      }
    }
    if (!std::isfinite(best_z)) {
      for (const PortPlacementBand& band : pole_type->port_bands) {
        if (band.enabled && band.category == category_) {
          best_z = std::max(best_z, band.height_max_m);
        }
      }
    }
  }
  if (std::isfinite(best_z)) {
    return best_z;
  }
  return std::max(0.5, pole.height_m * 0.8);
}

double GroupedSpanLanePreparer::LaneRowBaseZForPole(const Pole& pole) const {
  return TemplateLayerBaseZForPole(pole);
}

double GroupedSpanLanePreparer::LaneRowTargetZForEndpoint(const Pole& pole,
                                                          const SegmentRelationFeasibility& feasibility) const {
  return std::max(0.5, LaneRowBaseZForPole(pole) - lowering_.EndpointLoweringOffsetM(feasibility));
}

std::size_t GroupedSpanLanePreparer::PortConnectionCount(ObjectId port_id) const {
  const auto it = state_.connection_index_access().spans_by_port.find(port_id);
  return (it == state_.connection_index_access().spans_by_port.end()) ? 0 : it->second.size();
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
      CoreState::add_unique_id(change_set_->updated_ids, id);
    }
  };

  const SupportKind kind = ctx_.support_kind(node_id);
  const Pole* pole = ctx_.support_pole(node_id);
  if (kind != SupportKind::kPole || pole == nullptr) {
    const Vec3d base_position = ctx_.support_position(node_id);
    const Vec3d side_axis = orientation_.GroupedLineAxisForEndpoint(node_id, peer_id);
    auto nonpole_order_key = [&](const Port* port) {
      if (port == nullptr) {
        return std::tuple<double, double, ObjectId>(0.0, 0.0, kInvalidObjectId);
      }
      return std::tuple<double, double, ObjectId>(dot_xy(port->world_position - base_position, side_axis),
                                                  port->world_position.z, port->id);
    };
    auto sort_nonpole_ports = [&](std::vector<ObjectId>* port_ids) {
      if (port_ids == nullptr) {
        return;
      }
      std::sort(port_ids->begin(), port_ids->end(), [&](ObjectId a, ObjectId b) {
        const Port* pa = state_.edit_state_access().ports.find(a);
        const Port* pb = state_.edit_state_access().ports.find(b);
        return nonpole_order_key(pa) < nonpole_order_key(pb);
      });
    };

    std::unordered_set<ObjectId> unique_nonpole_ports{};
    for (const Span& span : state_.edit_state_access().spans.items()) {
      if (span.bundle_id == kInvalidObjectId) {
        continue;
      }
      const Bundle* bundle = state_.edit_state_access().bundles.find(span.bundle_id);
      if (bundle == nullptr || bundle->bundle_template_id != bundle_template_id_) {
        continue;
      }
      const Port* pa = state_.edit_state_access().ports.find(span.port_a_id);
      const Port* pb = state_.edit_state_access().ports.find(span.port_b_id);
      if (pa == nullptr || pb == nullptr) {
        continue;
      }
      if (ctx_.resolve_span_endpoint_node(span, pa, true) == node_id && pa->layer == target_port_layer_ &&
          unique_nonpole_ports.insert(pa->id).second) {
        ports_result.value.push_back(pa->id);
      }
      if (ctx_.resolve_span_endpoint_node(span, pb, false) == node_id && pb->layer == target_port_layer_ &&
          unique_nonpole_ports.insert(pb->id).second) {
        ports_result.value.push_back(pb->id);
      }
    }
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
      for (const Span& span : state_.edit_state_access().spans.items()) {
        if (span.bundle_id == kInvalidObjectId) {
          continue;
        }
        const Bundle* bundle = state_.edit_state_access().bundles.find(span.bundle_id);
        if (bundle == nullptr || bundle->bundle_template_id != bundle_template_id_) {
          continue;
        }
        const Port* pa = state_.edit_state_access().ports.find(span.port_a_id);
        const Port* pb = state_.edit_state_access().ports.find(span.port_b_id);
        if (pa == nullptr || pb == nullptr || pa->layer != target_port_layer_ || pb->layer != target_port_layer_) {
          continue;
        }
        const ObjectId endpoint_a = ctx_.resolve_span_endpoint_node(span, pa, true);
        const ObjectId endpoint_b = ctx_.resolve_span_endpoint_node(span, pb, false);
        double t = support.source_edge_t;
        if (endpoint_a == support.source_edge_node_a_id && endpoint_b == support.source_edge_node_b_id) {
        } else if (endpoint_a == support.source_edge_node_b_id && endpoint_b == support.source_edge_node_a_id) {
          t = 1.0 - t;
        } else {
          continue;
        }
        const Vec3d world{
            pa->world_position.x + (pb->world_position.x - pa->world_position.x) * t,
            pa->world_position.y + (pb->world_position.y - pa->world_position.y) * t,
            pa->world_position.z + (pb->world_position.z - pa->world_position.z) * t,
        };
        samples.push_back({world, dot_xy(world - base_position, side_axis)});
      }
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
            state_.AddPort(kInvalidObjectId, sample.world, CoreState::category_to_port_kind(category_),
                           target_port_layer_);
        if (!add_port.ok) {
          ports_result.error = add_port.error;
          return ports_result;
        }
        append_state_change_set(add_port.change_set);
        Port* created_port = state_.edit_state_access().ports.find(add_port.value);
        if (created_port != nullptr) {
          created_port->generated_by_rule = true;
          created_port->placement_context = ConnectionContext::kBranchAdd;
          CoreState::apply_port_position_mode(*created_port, PortPositionMode::kAuto,
                                              PortPlacementSourceKind::kAerialBranch);
          mark_updated(created_port->id);
        }
        ports_result.value.push_back(add_port.value);
      }
    }

    ports_result.value.reserve(static_cast<std::size_t>(lane_count_));
    while (static_cast<int>(ports_result.value.size()) < lane_count_) {
      EditResult<ObjectId> add_port =
          state_.AddPort(kInvalidObjectId, base_position, CoreState::category_to_port_kind(category_),
                         target_port_layer_);
      if (!add_port.ok) {
        ports_result.error = add_port.error;
        return ports_result;
      }
      append_state_change_set(add_port.change_set);
      Port* created_port = state_.edit_state_access().ports.find(add_port.value);
      if (created_port != nullptr) {
        created_port->generated_by_rule = true;
        created_port->placement_context = ConnectionContext::kBranchAdd;
        CoreState::apply_port_position_mode(*created_port, PortPositionMode::kAuto,
                                            PortPlacementSourceKind::kAerialBranch);
        mark_updated(created_port->id);
      }
      ports_result.value.push_back(add_port.value);
    }
    sort_nonpole_ports(&ports_result.value);
    node_lane_ports_cache_[cache_key] = ports_result.value;
    ports_result.ok = true;
    return ports_result;
  }

  const bool use_outboard_lowered_ports_here = lowering_.EndpointRequiresOutboardLoweredPorts(node_id, peer_id);
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
      request.preferred_template_layer = generation::detail::TemplateLayerForCategory(category_);
      request.preferred_template_side =
          (target_y < -1e-9) ? SlotSide::kLeft : ((target_y > 1e-9) ? SlotSide::kRight : SlotSide::kCenter);
      request.preferred_template_role = preferred_role;
      request.branch_index = static_cast<std::uint32_t>(lane);
      request.endpoint_decision.relation_kind = feasibility.kind;
      request.endpoint_decision.continuity_class = feasibility.continuity_class;
      request.endpoint_decision.in_through_pair = feasibility.in_through_pair;
      request.endpoint_decision.lower_required =
          feasibility.default_lower_required || !feasibility.same_level_feasible;
      request.endpoint_decision.default_lower_required = feasibility.default_lower_required;
      request.endpoint_decision.same_level_feasible = false;
      request.endpoint_decision.same_level_reason = feasibility.reason;
      request.endpoint_decision.projected_spacing_topview_m = feasibility.projected_spacing_topview_m;
      request.endpoint_decision.required_clearance_m = feasibility.required_clearance_m;
      request.endpoint_decision.order_decision_policy = order_decision_policy_;
      request.excluded_port_ids.assign(solved_ports.begin(), solved_ports.end());
      EditResult<ObjectId> port_result = state_.ensure_pole_connection_port(request);
      if (!port_result.ok) {
        return false;
      }
      Port* selected_port = state_.edit_state_access().ports.find(port_result.value);
      if (selected_port == nullptr || selected_port->owner_pole_id != node_id ||
          (selected_port->placement_source != PortPlacementSourceKind::kPlacementBand &&
           selected_port->placement_source != PortPlacementSourceKind::kPlacementBandConstrained) ||
          !used_ports.insert(selected_port->id).second) {
        return false;
      }
      solved_ports.push_back(selected_port->id);
    }
    std::sort(solved_ports.begin(), solved_ports.end(), [&](ObjectId a, ObjectId b) {
      const Port* pa = state_.edit_state_access().ports.find(a);
      const Port* pb = state_.edit_state_access().ports.find(b);
      const double ya =
          (pa == nullptr) ? 0.0
                          : WorldPointToLocal(BuildPoleFrame(pole->world_transform, LayoutYawForPole(*pole)),
                                              pa->world_position)
                                .y;
      const double yb =
          (pb == nullptr) ? 0.0
                          : WorldPointToLocal(BuildPoleFrame(pole->world_transform, LayoutYawForPole(*pole)),
                                              pb->world_position)
                                .y;
      if (std::abs(ya - yb) > 1e-9) {
        return ya < yb;
      }
      return a < b;
    });
    if (static_cast<int>(solved_ports.size()) == lane_count_ && !solved_ports.empty()) {
      const double layout_yaw = LayoutYawForPole(*pole);
      const PoleFrame frame = BuildPoleFrame(pole->world_transform, layout_yaw);
      const double target_uniform_z = LaneRowTargetZForEndpoint(*pole, feasibility);
      for (ObjectId port_id : solved_ports) {
        Port* selected_port = state_.edit_state_access().ports.find(port_id);
        if (selected_port == nullptr) {
          continue;
        }
        Vec3d local = WorldPointToLocal(frame, selected_port->world_position);
        if (std::abs(local.z - target_uniform_z) <= 1e-9) {
          continue;
        }
        local.z = target_uniform_z;
        selected_port->world_position = local_to_world_on_pole_local(pole->world_transform, layout_yaw, local);
        mark_updated(selected_port->id);
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
                                state_.cache_state_access().geometry_settings.pole_clearance_m + half_span + 0.25;
    const double branch_support_offset_m = std::max(0.65, min_outboard);
    ports_result.value.reserve(static_cast<std::size_t>(lane_count_));
    for (int lane = 0; lane < lane_count_; ++lane) {
      const double lane_offset = (static_cast<double>(lane) - center) * lane_spacing;
      const double lane_z_m = branch_base_z_m;
      Vec3d local{0.0, side_sign * branch_support_offset_m + lane_offset, lane_z_m};
      local = state_.apply_pole_clearance_to_local(*pole, local, branch_side);
      const Vec3d world = local_to_world_on_pole_local(pole->world_transform, branch_support_yaw_deg, local);
      EditResult<ObjectId> add_port =
          state_.AddPort(node_id, world, CoreState::category_to_port_kind(category_),
                         CoreState::category_to_port_layer(category_));
      if (!add_port.ok) {
        ports_result.error = add_port.error;
        return ports_result;
      }
      append_state_change_set(add_port.change_set);
      Port* created_port = state_.edit_state_access().ports.find(add_port.value);
      if (created_port != nullptr) {
        created_port->category = category_;
        created_port->template_layer = generation::detail::TemplateLayerForCategory(category_);
        created_port->template_side = branch_side;
        created_port->template_role = SlotRole::kBranchPreferred;
        created_port->generated_from_template = false;
        created_port->generated_by_rule = true;
        created_port->placement_context = ConnectionContext::kBranchAdd;
        CoreState::apply_port_position_mode(*created_port, PortPositionMode::kAuto,
                                            PortPlacementSourceKind::kBranchSupport);
        mark_updated(created_port->id);
      }
      ports_result.value.push_back(add_port.value);
    }
    std::sort(ports_result.value.begin(), ports_result.value.end(), [&](ObjectId a, ObjectId b) {
      const Port* pa = state_.edit_state_access().ports.find(a);
      const Port* pb = state_.edit_state_access().ports.find(b);
      const double ya =
          (pa == nullptr) ? 0.0
                          : WorldPointToLocal(BuildPoleFrame(pole->world_transform, branch_support_yaw_deg),
                                              pa->world_position)
                                .y;
      const double yb =
          (pb == nullptr) ? 0.0
                          : WorldPointToLocal(BuildPoleFrame(pole->world_transform, branch_support_yaw_deg),
                                              pb->world_position)
                                .y;
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

    if (const auto it_ports = state_.relation_index_access().ports_by_pole.find(node_id);
        it_ports != state_.relation_index_access().ports_by_pole.end()) {
      for (ObjectId port_id : it_ports->second) {
        const Port* port = state_.edit_state_access().ports.find(port_id);
        if (port == nullptr || port->layer != target_port_layer_ || port->category != category_) {
          continue;
        }
        if (port->generated_by_rule && PortConnectionCount(port_id) == 0) {
          reusable_generated_ports.push_back(port_id);
        }
      }
    }

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
        Port* reused_port = state_.edit_state_access().ports.find(port_id);
        if (reused_port == nullptr) {
          continue;
        }
        reused_port->world_position = world;
        reused_port->category = category_;
        reused_port->generated_by_rule = true;
        reused_port->placement_context = ConnectionContext::kTrunkContinue;
        reused_port->template_layer = generation::detail::TemplateLayerForCategory(category_);
        reused_port->template_side =
            (target_y < -1e-9) ? SlotSide::kLeft : ((target_y > 1e-9) ? SlotSide::kRight : SlotSide::kCenter);
        CoreState::apply_port_position_mode(*reused_port, PortPositionMode::kAuto, PortPlacementSourceKind::kGenerated);
        mark_updated(reused_port->id);
        unique.insert(port_id);
        ports_result.value.push_back(port_id);
        continue;
      }

      EditResult<ObjectId> add_port =
          state_.AddPort(node_id, world, CoreState::category_to_port_kind(category_),
                         CoreState::category_to_port_layer(category_));
      if (!add_port.ok) {
        ports_result.error = add_port.error;
        return ports_result;
      }
      append_state_change_set(add_port.change_set);
      Port* created_port = state_.edit_state_access().ports.find(add_port.value);
      if (created_port != nullptr) {
        created_port->category = category_;
        created_port->generated_by_rule = true;
        created_port->placement_context = ConnectionContext::kTrunkContinue;
        created_port->template_layer = generation::detail::TemplateLayerForCategory(category_);
        created_port->template_side =
            (target_y < -1e-9) ? SlotSide::kLeft : ((target_y > 1e-9) ? SlotSide::kRight : SlotSide::kCenter);
        CoreState::apply_port_position_mode(*created_port, PortPositionMode::kAuto, PortPlacementSourceKind::kGenerated);
        mark_updated(created_port->id);
      }
      unique.insert(add_port.value);
      ports_result.value.push_back(add_port.value);
    }
  } else {
    for (const Port& port : state_.edit_state_access().ports.items()) {
      if (port.owner_pole_id == node_id && port.layer == target_port_layer_) {
        ports_result.value.push_back(port.id);
      }
    }
  }

  auto port_links_to_neighbor = [&](ObjectId port_id, ObjectId neighbor_node_id) -> int {
    int count = 0;
    const auto it = state_.connection_index_access().spans_by_port.find(port_id);
    if (it == state_.connection_index_access().spans_by_port.end()) {
      return 0;
    }
    for (ObjectId span_id : it->second) {
      const Span* span = state_.edit_state_access().spans.find(span_id);
      if (span == nullptr) {
        continue;
      }
      const ObjectId other_port_id = (span->port_a_id == port_id) ? span->port_b_id : span->port_a_id;
      const Port* other_port = state_.edit_state_access().ports.find(other_port_id);
      if (other_port == nullptr || other_port->layer != target_port_layer_) {
        continue;
      }
      if (other_port->owner_pole_id == neighbor_node_id) {
        ++count;
      }
    }
    return count;
  };

  auto side_rank = [](SlotSide side) -> int {
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
  };
  auto local_y_of = [&](const Port* p) -> double {
    if (p == nullptr || pole == nullptr) {
      return 0.0;
    }
    return WorldPointToLocal(BuildPoleFrame(pole->world_transform, LayoutYawForPole(*pole)), p->world_position).y;
  };
  auto order_key = [&](const Port* p) -> std::tuple<double, int, int, int, int, ObjectId> {
    if (p == nullptr) {
      return {0.0, 1, 999, 999999, 999999, kInvalidObjectId};
    }
    return {local_y_of(p), 0, p->template_layer, side_rank(p->template_side),
            static_cast<int>(std::llround(HeightAlongWorldUp(p->world_position) * 1000.0)), p->id};
  };

  ObjectId continuity_neighbor_id = kInvalidObjectId;
  if (prefer_existing_neighbor_order) {
    std::unordered_map<ObjectId, int> neighbor_counts{};
    for (ObjectId port_id : ports_result.value) {
      const auto it = state_.connection_index_access().spans_by_port.find(port_id);
      if (it == state_.connection_index_access().spans_by_port.end()) {
        continue;
      }
      for (ObjectId span_id : it->second) {
        const Span* span = state_.edit_state_access().spans.find(span_id);
        if (span == nullptr) {
          continue;
        }
        const ObjectId other_port_id = (span->port_a_id == port_id) ? span->port_b_id : span->port_a_id;
        const Port* other_port = state_.edit_state_access().ports.find(other_port_id);
        if (other_port == nullptr || other_port->layer != target_port_layer_) {
          continue;
        }
        const ObjectId other_node_id = other_port->owner_pole_id;
        if (other_node_id == kInvalidObjectId || other_node_id == node_id || other_node_id == peer_id) {
          continue;
        }
        neighbor_counts[other_node_id] += 1;
      }
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
      const Port* port = state_.edit_state_access().ports.find(port_id);
      if (port == nullptr || port->owner_pole_id != node_id || port->layer != target_port_layer_ ||
          port_links_to_neighbor(port_id, continuity_neighbor_id) <= 0) {
        continue;
      }
      if (!unique_seeded_ports.insert(port_id).second) {
        continue;
      }
      seeded_order.push_back(port_id);
    }
    if (static_cast<int>(seeded_order.size()) >= lane_count_) {
      std::sort(seeded_order.begin(), seeded_order.end(), [&](ObjectId a, ObjectId b) {
        const Port* pa = state_.edit_state_access().ports.find(a);
        const Port* pb = state_.edit_state_access().ports.find(b);
        return order_key(pa) < order_key(pb);
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
      auto terminal_order_key = [&](const Port* p) -> std::tuple<int, int, double, ObjectId> {
        if (p == nullptr) {
          return {999, 999, 0.0, kInvalidObjectId};
        }
        return {p->template_layer, side_rank(p->template_side), local_y_of(p), p->id};
      };
      std::sort(ports_result.value.begin(), ports_result.value.end(), [&](ObjectId a, ObjectId b) {
        const Port* pa = state_.edit_state_access().ports.find(a);
        const Port* pb = state_.edit_state_access().ports.find(b);
        return terminal_order_key(pa) < terminal_order_key(pb);
      });

      std::vector<ObjectId> normalized_ports{};
      normalized_ports.reserve(static_cast<std::size_t>(lane_count_));
      std::vector<ObjectId> reusable_fallback_ports{};
      reusable_fallback_ports.reserve(ports_result.value.size());
      for (ObjectId port_id : ports_result.value) {
        const Port* port = state_.edit_state_access().ports.find(port_id);
        if (port == nullptr) {
          continue;
        }
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
          Port* port = state_.edit_state_access().ports.find(port_id);
          if (port != nullptr) {
            port->world_position = world;
            port->category = category_;
            port->generated_by_rule = true;
            port->template_layer = generation::detail::TemplateLayerForCategory(category_);
            port->template_side =
                (target_y < -1e-9) ? SlotSide::kLeft : ((target_y > 1e-9) ? SlotSide::kRight : SlotSide::kCenter);
            CoreState::apply_port_position_mode(*port, PortPositionMode::kAuto, PortPlacementSourceKind::kGenerated);
            mark_updated(port->id);
            normalized_ports.push_back(port_id);
            continue;
          }
        }

        EditResult<ObjectId> add_port =
            state_.AddPort(node_id, world, CoreState::category_to_port_kind(category_),
                           CoreState::category_to_port_layer(category_));
        if (!add_port.ok) {
          ports_result.error = add_port.error;
          return ports_result;
        }
        append_state_change_set(add_port.change_set);
        Port* created_port = state_.edit_state_access().ports.find(add_port.value);
        if (created_port != nullptr) {
          created_port->category = category_;
          created_port->generated_by_rule = true;
          created_port->placement_context = ConnectionContext::kTrunkContinue;
          created_port->template_layer = generation::detail::TemplateLayerForCategory(category_);
          created_port->template_side =
              (target_y < -1e-9) ? SlotSide::kLeft : ((target_y > 1e-9) ? SlotSide::kRight : SlotSide::kCenter);
          CoreState::apply_port_position_mode(*created_port, PortPositionMode::kAuto, PortPlacementSourceKind::kGenerated);
          mark_updated(created_port->id);
        }
        normalized_ports.push_back(add_port.value);
      }

      std::sort(normalized_ports.begin(), normalized_ports.end(), [&](ObjectId a, ObjectId b) {
        const Port* pa = state_.edit_state_access().ports.find(a);
        const Port* pb = state_.edit_state_access().ports.find(b);
        return local_y_of(pa) < local_y_of(pb);
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
      const Vec3d base = pole->world_transform.position;
      const SegmentRelationFeasibility relation_feasibility = lowering_.SegmentRelationFeasibilityFor(node_id, peer_id);
      const double base_z_m = LaneRowTargetZForEndpoint(*pole, relation_feasibility);
      if (!use_scaffold_geometry || node_index == ctx_.node_ids.size()) {
        const Vec3d local{0.0, target_y, base_z_m};
        return local_to_world_on_pole_local(pole->world_transform, LayoutYawForPole(*pole), local);
      }

      const Vec3d prev = ctx_.support_position(ctx_.node_ids[node_index - 1]);
      const Vec3d next = ctx_.support_position(ctx_.node_ids[node_index + 1]);
      if ((prev - base).x * (prev - base).x + (prev - base).y * (prev - base).y <= 1e-12 ||
          (next - base).x * (next - base).x + (next - base).y * (next - base).y <= 1e-12) {
        const Vec3d local{0.0, target_y, base_z_m};
        return local_to_world_on_pole_local(pole->world_transform, LayoutYawForPole(*pole), local);
      }

      Vec3d dir_in = base - prev;
      Vec3d dir_out = next - base;
      if (!normalize_xy(&dir_in) || !normalize_xy(&dir_out)) {
        const Vec3d local{0.0, target_y, base_z_m};
        return local_to_world_on_pole_local(pole->world_transform, LayoutYawForPole(*pole), local);
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
        const double limit = std::max(spacing * 8.0, std::abs(target_y) * 8.0 + 0.2);
        if (dist <= limit || dist <= 1e-9) {
          return joined_xy;
        }
        const double scale = limit / dist;
        return Vec3d{base.x + dx * scale, base.y + dy * scale, HeightAlongWorldUp(base) + base_z_m};
      }

      const Vec3d local{0.0, target_y, base_z_m};
      return local_to_world_on_pole_local(pole->world_transform, LayoutYawForPole(*pole), local);
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
        const Port* candidate = state_.edit_state_access().ports.find(candidate_id);
        if (candidate == nullptr || candidate->owner_pole_id != node_id || candidate->layer != target_port_layer_) {
          continue;
        }
        const double dist =
            use_scaffold_geometry
                ? std::sqrt(std::pow(candidate->world_position.x - desired_world.x, 2.0) +
                            std::pow(candidate->world_position.y - desired_world.y, 2.0))
                : std::abs(dot_xy(candidate->world_position - pole->world_transform.position, stable_side_axis) - target_y);
        if (dist < best_dist) {
          best_dist = dist;
          best_id = candidate_id;
        }
      }

      const double kTargetMatchTolerance =
          use_scaffold_geometry ? std::max(0.05, spacing * 0.35) : std::max(0.02, spacing * 0.1);
      if (best_id == kInvalidObjectId || best_dist > kTargetMatchTolerance) {
        EditResult<ObjectId> extra =
            state_.AddPort(node_id, desired_world, CoreState::category_to_port_kind(category_),
                           CoreState::category_to_port_layer(category_));
        if (!extra.ok) {
          ports_result.error = extra.error;
          return ports_result;
        }
        append_state_change_set(extra.change_set);
        best_id = extra.value;
      }
      if (use_scaffold_geometry) {
        Port* selected = state_.edit_state_access().ports.find(best_id);
        if (selected != nullptr && selected->owner_pole_id == node_id) {
          selected->world_position = desired_world;
          mark_updated(selected->id);
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
    const Port* pa = state_.edit_state_access().ports.find(a);
    const Port* pb = state_.edit_state_access().ports.find(b);
    return order_key(pa) < order_key(pb);
  });

  if (static_cast<int>(ports_result.value.size()) > lane_count_) {
    if (continuity_neighbor_id != kInvalidObjectId) {
      std::stable_sort(ports_result.value.begin(), ports_result.value.end(), [&](ObjectId a, ObjectId b) {
        const int score_a = port_links_to_neighbor(a, continuity_neighbor_id);
        const int score_b = port_links_to_neighbor(b, continuity_neighbor_id);
        if (score_a != score_b) {
          return score_a > score_b;
        }
        const Port* pa = state_.edit_state_access().ports.find(a);
        const Port* pb = state_.edit_state_access().ports.find(b);
        return order_key(pa) < order_key(pb);
      });
      ports_result.value.resize(static_cast<std::size_t>(lane_count_));
      std::sort(ports_result.value.begin(), ports_result.value.end(), [&](ObjectId a, ObjectId b) {
        const Port* pa = state_.edit_state_access().ports.find(a);
        const Port* pb = state_.edit_state_access().ports.find(b);
        return order_key(pa) < order_key(pb);
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
          const Port* p = state_.edit_state_access().ports.find(ports_result.value[i]);
          const double y = local_y_of(p);
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
