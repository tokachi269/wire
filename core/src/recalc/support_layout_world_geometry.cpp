#include "support_layout_world_geometry_internal.hpp"

#include "wire/core/core_state.hpp"
#include "wire/core/core_view.hpp"
#include "wire/core/coord_utils.hpp"
#include "../pole_orientation_utils.hpp"
#include "../support_orientation_utils.hpp"
#include "detail_curve_input_resolution.hpp"

namespace wire::core {

namespace {

Vec3d grouped_lowered_route_local_departure_dir(const SupportLayoutEndpoint& endpoint,
                                                const SupportLayoutEndpoint& other_endpoint) {
  Vec3d fallback = other_endpoint.endpoint_world - endpoint.endpoint_world;
  if (!Normalize(&fallback)) {
    fallback = WorldForward();
  }
  if (!endpoint.has_side_axis) {
    return fallback;
  }
  Vec3d side_axis = endpoint.side_axis;
  side_axis.z = 0.0;
  if (!Normalize(&side_axis)) {
    return fallback;
  }
  Vec3d tangent = ComputeLateralAxis(side_axis);
  if (!Normalize(&tangent)) {
    return fallback;
  }
  Vec3d peer_dir = other_endpoint.endpoint_world - endpoint.endpoint_world;
  peer_dir.z = 0.0;
  if (Normalize(&peer_dir) && Dot(tangent, peer_dir) < 0.0) {
    tangent = ScaleVec(tangent, -1.0);
  }
  return tangent;
}

std::pair<Vec3d, Vec3d> shared_support_anchor_points(const CoreState& state, const Pole& pole, const Vec3d& support_axis,
                                                     ConnectionCategory category, double z_m,
                                                     const CacheState& cache_state) {
  Vec3d axis = SafeHorizontalNormalized(support_axis);
  const double mount_radius =
      cache_state.visual_settings.support_center_threshold_m + cache_state.geometry_settings.pole_clearance_m;
  const double tip_radius = mount_radius + cache_state.visual_settings.support_arm_extra_m;
  Vec3d center_world = pole.world_transform.position;
  const double layout_yaw_deg = state.effective_port_layout_yaw_deg(pole, category);
  const PoleFrame frame = BuildPoleFrame(pole.world_transform, layout_yaw_deg);
  if (std::abs(frame.up.z) > 1e-9) {
    const double local_z = (z_m - frame.origin.z) / frame.up.z;
    center_world = LocalPointToWorld(frame, {0.0, 0.0, local_z});
  } else {
    center_world.z = z_m;
  }
  const Vec3d mount{center_world.x + axis.x * mount_radius, center_world.y + axis.y * mount_radius, z_m};
  const Vec3d tip{center_world.x + axis.x * tip_radius, center_world.y + axis.y * tip_radius, z_m};
  return {mount, tip};
}

} // namespace

LoweredSupportGroupPlacement build_grouped_support_placement_from_decision(const CoreState& state,
                                                                           const SupportGroupDecision& group_decision,
                                                                           const EditState& edit_state,
                                                                           const CacheState& cache_state,
                                                                           const ConnectionCategory* observed_category,
                                                                           const std::vector<Vec3d>* observed_attachment_worlds) {
  LoweredSupportGroupPlacement group{};
  group.grouping_rule = SupportGroupingRuleKind::kDecisionGroup;
  group.grouped_port_count = (observed_attachment_worlds == nullptr) ? 0 : static_cast<int>(observed_attachment_worlds->size());
  group.down_offset_m = 0.0;
  group.attachment_worlds = (observed_attachment_worlds == nullptr) ? std::vector<Vec3d>{} : *observed_attachment_worlds;
  group.down_offset_variation = {};

  const Pole* pole = edit_state.poles.find(group_decision.owner_pole_id);
  if (pole == nullptr) {
    return group;
  }
  Vec3d support_axis = AuthoritativeSupportAxisForGroup(group_decision);
  if (!Normalize(&support_axis) || !IsFiniteXY(support_axis)) {
    return group;
  }
  const ConnectionCategory category =
      (observed_category == nullptr) ? ConnectionCategory::kLowVoltage : *observed_category;
  const double support_z =
      state.view().port_category_base_z_for_pole(*pole, category) - group.down_offset_m;
  const auto [mount_world, tip_world] =
      shared_support_anchor_points(state, *pole, support_axis, category, support_z, cache_state);
  group.mount_world = mount_world;
  group.tip_world = tip_world;
  return group;
}

void apply_materialized_visual_arm_geometry(const CoreState& state, const Port& port, const Pole* pole,
                                            SupportLayoutEndpoint* endpoint) {
  if (endpoint == nullptr || pole == nullptr) {
    return;
  }
  if (!endpoint->support_authority.has_signed_support_axis) {
    endpoint->has_visual_arm_geometry = false;
    endpoint->visual_arm_mount_world = {};
    endpoint->visual_arm_tip_world = {};
    endpoint->visual_insulator_base_world = {};
    return;
  }

  Vec3d support_axis = endpoint->support_authority.signed_support_axis;
  if (!Normalize(&support_axis) || !IsFiniteXY(support_axis)) {
    endpoint->has_visual_arm_geometry = false;
    endpoint->visual_arm_mount_world = {};
    endpoint->visual_arm_tip_world = {};
    endpoint->visual_insulator_base_world = {};
    return;
  }

  const auto [mount_world, tip_world] =
      shared_support_anchor_points(state, *pole, support_axis, port.category, endpoint->support_world.z,
                                   state.view().cache_state());
  endpoint->has_visual_arm_geometry = true;
  endpoint->visual_arm_mount_world = {pole->world_transform.position.x, pole->world_transform.position.y, mount_world.z};
  endpoint->visual_arm_tip_world = tip_world;
  endpoint->visual_insulator_base_world = tip_world;
}

void finalize_support_layout_materialization(const Vec3d& fallback_chord_dir, SpanLayoutEntry* layout) {
  if (layout == nullptr) {
    return;
  }
  Vec3d resolved_chord_dir = layout->end.endpoint_world - layout->start.endpoint_world;
  if (!Normalize(&resolved_chord_dir)) {
    resolved_chord_dir = fallback_chord_dir;
  }
  const bool grouped_lowered_span =
      (UsesAuthoritativeGroupedLoweredSupport(layout->start) ||
       UsesAuthoritativeGroupedLoweredSupport(layout->end)) &&
      (!layout->start.same_level_feasible || !layout->end.same_level_feasible);
  if (grouped_lowered_span) {
    layout->start.departure_dir = grouped_lowered_route_local_departure_dir(layout->start, layout->end);
    layout->end.departure_dir = grouped_lowered_route_local_departure_dir(layout->end, layout->start);
  }
  layout->detail_curve_profile_hint = detail_curve_profile_hint_from_support_layout(*layout);
}

} // namespace wire::core
