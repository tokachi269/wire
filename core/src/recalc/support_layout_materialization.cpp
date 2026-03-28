#include "support_layout_materialization.hpp"

#include "wire/core/core_state.hpp"
#include "wire/core/core_view.hpp"
#include "wire/core/coord_utils.hpp"
#include "detail_curve_input_resolution.hpp"
#include "../generation/support_policy.hpp"
#include "../support_orientation_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

namespace wire::core {

namespace {

const AttachmentSocketTemplate* find_attachment_socket(const AttachmentTemplate& attachment_template, int socket_id) {
  for (const AttachmentSocketTemplate& socket : attachment_template.sockets) {
    if (socket.id == socket_id) {
      return &socket;
    }
  }
  return nullptr;
}

Vec3d attachment_direction_to_world(const Vec3d& forward, const Vec3d& lateral, const Vec3d& up, const Vec3d& local_dir,
                                    const Vec3d& fallback) {
  Vec3d world =
      ScaleVec(forward, local_dir.x) + ScaleVec(lateral, local_dir.y) + ScaleVec(up, local_dir.z);
  if (!Normalize(&world)) {
    return fallback;
  }
  return world;
}

bool apply_endpoint_attachment_socket(const CoreState& state, ObjectId attachment_id, int socket_id,
                                      CurveConstraint* constraint) {
  if (constraint == nullptr) {
    return false;
  }
  if (attachment_id == kInvalidObjectId || socket_id < 0) {
    return false;
  }
  const Attachment* attachment = state.view().attachments().find(attachment_id);
  if (attachment == nullptr) {
    return false;
  }
  const AttachmentTemplate* attachment_template = state.find_attachment_template(attachment->template_id);
  if (attachment_template == nullptr) {
    return false;
  }
  const AttachmentSocketTemplate* socket = find_attachment_socket(*attachment_template, socket_id);
  if (socket == nullptr) {
    return false;
  }

  Vec3d forward{};
  Vec3d lateral{};
  Vec3d up{};
  if (!build_attachment_frame(constraint->tangent_dir, &forward, &lateral, &up)) {
    return false;
  }
  Vec3d origin = constraint->point;
  OffsetAlongWorldUp(&origin, attachment->display_offset_m);
  constraint->point = attachment_local_to_world(origin, forward, lateral, up, socket->local_position);
  constraint->tangent_dir =
      attachment_direction_to_world(forward, lateral, up, socket->tangent_dir, constraint->tangent_dir);
  constraint->endpoint_mode = CurveEndpointMode::kDirectThrough;
  constraint->endpoint_offset = {};
  return true;
}

EndpointAttachmentRequest make_endpoint_attachment_request(ObjectId attachment_id, int resolved_socket_id,
                                                           bool socket_override_active) {
  EndpointAttachmentRequest request{};
  if (attachment_id != kInvalidObjectId) {
    request.attachment_id = attachment_id;
    if (resolved_socket_id >= 0) {
      request.kind = EndpointAttachmentRequestKind::kAttachmentSocket;
      request.requested_socket_id = resolved_socket_id;
    } else {
      request.kind = EndpointAttachmentRequestKind::kAttachmentAuto;
    }
    return request;
  }

  if (resolved_socket_id >= 0 || socket_override_active) {
    request.kind = EndpointAttachmentRequestKind::kDanglingSocket;
    if (resolved_socket_id >= 0) {
      request.requested_socket_id = resolved_socket_id;
    }
  }
  return request;
}

bool bundle_prefers_offset_endpoint_mode(const Bundle* bundle, const BundleTemplate* bundle_template) {
  if (bundle_template == nullptr || !bundle_template->preserve_conductor_identity) {
    return false;
  }
  int conductor_count = 1;
  if (bundle != nullptr && bundle->conductor_count > 0) {
    conductor_count = bundle->conductor_count;
  } else if (bundle_template->count_rule == BundleCountRuleKind::kFixed) {
    conductor_count = bundle_template->fixed_count;
  } else {
    conductor_count = bundle_template->default_count;
  }
  return conductor_count > 1;
}

Vec3d span_tangent_from_port(const Port& port, const Vec3d& chord_dir) {
  Vec3d tangent = port.direction.forward;
  if (!Normalize(&tangent)) {
    return chord_dir;
  }
  if (Dot(tangent, chord_dir) < 0.0) {
    tangent = ScaleVec(tangent, -1.0);
  }
  return tangent;
}

CurveConstraint make_curve_constraint_from_port(const Port& port, const Pole* owner_pole, const Vec3d& chord_dir,
                                                double basis_length, double endpoint_offset_m,
                                                double effective_sag_ratio, double bend_stiffness_hint,
                                                double min_bend_radius_hint_m,
                                                CableContinuityPolicyHint continuity_preference,
                                                CurvePassMode pass_mode, ConnectionContext connection_context,
                                                CurveEndpointMode endpoint_mode, bool reverse_endpoint_offset) {
  CurveConstraint constraint{};
  constraint.point = port.world_position;
  constraint.tangent_dir = span_tangent_from_port(port, chord_dir);
  constraint.tangent_strength = 1.0;
  constraint.tangent_length_hint_m = basis_length * 0.30;
  if (pass_mode == CurvePassMode::kBranch) {
    constraint.support_departure_length_m =
        std::clamp(std::max(endpoint_offset_m * 2.5, basis_length * 0.10), 0.18, std::max(0.18, basis_length * 0.22));
  } else if (pass_mode == CurvePassMode::kTerminate) {
    constraint.support_departure_length_m =
        std::clamp(std::max(endpoint_offset_m * 2.2, basis_length * 0.09), 0.16, std::max(0.16, basis_length * 0.18));
  } else {
    constraint.support_departure_length_m =
        std::clamp(std::max(endpoint_offset_m * 2.0, basis_length * 0.16), 0.22, std::max(0.22, basis_length * 0.30));
  }
  constraint.bend_stiffness_hint = bend_stiffness_hint;
  constraint.min_bend_radius_hint_m = min_bend_radius_hint_m;
  const double signed_offset_m = reverse_endpoint_offset ? -endpoint_offset_m : endpoint_offset_m;
  constraint.endpoint_offset = ScaleVec(constraint.tangent_dir, signed_offset_m);
  constraint.sag_hint = effective_sag_ratio * 0.5;
  constraint.slack_hint = 0.0;
  constraint.continuity_preference = continuity_preference;
  constraint.pass_mode = pass_mode;
  constraint.endpoint_mode = endpoint_mode;
  constraint.corner_pass =
      (connection_context == ConnectionContext::kCornerPass) && owner_pole != nullptr &&
      owner_pole->context.kind == PoleContextKind::kCorner && owner_pole->context.corner_angle_deg > 1e-6;
  constraint.corner_angle_deg = (owner_pole == nullptr) ? 0.0 : owner_pole->context.corner_angle_deg;
  return constraint;
}

} // namespace

bool build_attachment_frame(const Vec3d& tangent, Vec3d* forward, Vec3d* lateral, Vec3d* up) {
  if (forward == nullptr || lateral == nullptr || up == nullptr) {
    return false;
  }
  *forward = tangent;
  if (!Normalize(forward)) {
    *forward = WorldForward();
  }
  *lateral = ComputeLateralAxis(*forward);
  if (!Normalize(lateral)) {
    *lateral = {1.0, 0.0, 0.0};
  }
  *up = Cross(*forward, *lateral);
  if (!Normalize(up)) {
    *up = WorldUp();
  }
  return true;
}

Vec3d attachment_local_to_world(const Vec3d& origin, const Vec3d& forward, const Vec3d& lateral, const Vec3d& up,
                                const Vec3d& local) {
  return origin + ScaleVec(forward, local.x) + ScaleVec(lateral, local.y) + ScaleVec(up, local.z);
}

bool resolve_attachment_socket_pair(const AttachmentTemplate& attachment_template,
                                    const AttachmentSocketTemplate** out_a,
                                    const AttachmentSocketTemplate** out_b,
                                    const AttachmentInternalPathTemplate** out_internal_path) {
  if (out_a == nullptr || out_b == nullptr || out_internal_path == nullptr) {
    return false;
  }
  *out_a = nullptr;
  *out_b = nullptr;
  *out_internal_path = nullptr;
  if (!attachment_template.internal_paths.empty()) {
    const AttachmentInternalPathTemplate& path = attachment_template.internal_paths.front();
    const AttachmentSocketTemplate* start = find_attachment_socket(attachment_template, path.start_socket_id);
    const AttachmentSocketTemplate* end = find_attachment_socket(attachment_template, path.end_socket_id);
    if (start != nullptr && end != nullptr) {
      *out_a = start;
      *out_b = end;
      *out_internal_path = &path;
      return true;
    }
  }
  if (attachment_template.sockets.size() >= 2) {
    *out_a = &attachment_template.sockets[0];
    *out_b = &attachment_template.sockets[1];
    return true;
  }
  return false;
}

bool endpoint_uses_grouped_lowered_support(const SupportLayoutEndpoint* endpoint) {
  return endpoint != nullptr && UsesAuthoritativeGroupedLoweredSupport(endpoint->decision);
}

double template_layer_base_z_for_port_category(const CoreState& state, const Pole& pole, ConnectionCategory category) {
  double best_z = -std::numeric_limits<double>::infinity();
  const auto pole_type_it = state.view().pole_types().find(pole.pole_type_id);
  const int target_layer = generation::detail::TemplateLayerForCategory(category);
  if (pole_type_it != state.view().pole_types().end()) {
    for (const PortPlacementBand& band : pole_type_it->second.port_bands) {
      if (band.enabled && band.layer == target_layer) {
        best_z = std::max(best_z, band.height_max_m);
      }
    }
    if (!std::isfinite(best_z)) {
      for (const PortPlacementBand& band : pole_type_it->second.port_bands) {
        if (band.enabled && band.category == category) {
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

SupportLayoutOriginKind support_layout_origin_from_port(const Port& port) {
  switch (port.placement_source) {
  case PortPlacementSourceKind::kBranchSupport:
    return SupportLayoutOriginKind::kBranchSupport;
  case PortPlacementSourceKind::kAerialBranch:
    return SupportLayoutOriginKind::kAerialBranch;
  case PortPlacementSourceKind::kPlacementBandConstrained:
    return SupportLayoutOriginKind::kPlacementConstraint;
  case PortPlacementSourceKind::kPlacementBand:
  case PortPlacementSourceKind::kGenerated:
  case PortPlacementSourceKind::kManualEdit:
    return SupportLayoutOriginKind::kMainSupport;
  case PortPlacementSourceKind::kUnknown:
  default:
    return SupportLayoutOriginKind::kFallback;
  }
}

double fallback_branch_down_offset_for_support_port(const CoreState& state, const Port& port) {
  const Pole* pole = state.view().poles().find(port.owner_pole_id);
  if (pole == nullptr) {
    return 0.0;
  }
  const double template_z_m = template_layer_base_z_for_port_category(state, *pole, port.category);
  const double local_z_m = HeightAlongWorldUp(port.world_position) - HeightAlongWorldUp(pole->world_transform.position);
  return std::max(0.0, template_z_m - local_z_m);
}

BackboneFlowKind support_layout_flow_kind_for_span(const Span& span, const Port& port_a, const Port& port_b) {
  if (span.placement_context == ConnectionContext::kBranchAdd ||
      port_a.placement_source == PortPlacementSourceKind::kBranchSupport ||
      port_b.placement_source == PortPlacementSourceKind::kBranchSupport ||
      port_a.placement_source == PortPlacementSourceKind::kAerialBranch ||
      port_b.placement_source == PortPlacementSourceKind::kAerialBranch) {
    return BackboneFlowKind::kBranch;
  }
  return BackboneFlowKind::kMain;
}

CurveEndpointMode curve_endpoint_mode_for_template(const CableTemplate* cable_template, const Bundle* bundle,
                                                   const BundleTemplate* bundle_template) {
  if (bundle_prefers_offset_endpoint_mode(bundle, bundle_template)) {
    return CurveEndpointMode::kOffsetEndpoint;
  }
  if (cable_template == nullptr) {
    return CurveEndpointMode::kDirectThrough;
  }
  switch (cable_template->attachment_style) {
  case CableAttachmentStyleHint::kDirectThrough:
    return CurveEndpointMode::kDirectThrough;
  case CableAttachmentStyleHint::kViaAttachment:
    return CurveEndpointMode::kOffsetEndpoint;
  case CableAttachmentStyleHint::kAuto:
  default:
    break;
  }
  return CurveEndpointMode::kDirectThrough;
}

SupportLayoutEndpoint build_support_layout_endpoint(
    const CoreState& state, const Span& span, const Port& port, const Pole* owner_pole, const Vec3d& chord_dir,
    double basis_length, double endpoint_offset_m, double effective_sag_ratio, double bend_stiffness_hint,
    double min_bend_radius_hint_m, CableContinuityPolicyHint continuity_preference, CurvePassMode pass_mode,
    CurveEndpointMode endpoint_mode, BackboneFlowKind flow_kind, int resolved_socket_id, bool socket_override_active,
    double automatic_branch_down_offset_m, const HierarchicalVariationSample& down_offset_variation,
    double resolved_branch_down_offset_m, bool is_start_endpoint) {
  SupportLayoutEndpoint endpoint{};
  endpoint.endpoint_node_id = is_start_endpoint ? span.endpoint_node_a_id : span.endpoint_node_b_id;
  endpoint.owner_pole_id = port.owner_pole_id;
  endpoint.port_id = port.id;
  const ObjectId attachment_id = is_start_endpoint ? span.endpoint_attachment_a_id : span.endpoint_attachment_b_id;
  endpoint.flow_kind = flow_kind;
  endpoint.origin = support_layout_origin_from_port(port);
  endpoint.attachment_request = make_endpoint_attachment_request(attachment_id, resolved_socket_id, socket_override_active);
  if (resolved_socket_id >= 0) {
    endpoint.resolved_socket_id = resolved_socket_id;
  }
  endpoint.port_source = port.placement_source;
  endpoint.side = port.template_side;
  endpoint.support_world = port.world_position;
  endpoint.decision.owner_pole_id = endpoint.owner_pole_id;
  endpoint.automatic_branch_down_offset_m = automatic_branch_down_offset_m;
  endpoint.down_offset_variation = down_offset_variation;
  endpoint.branch_down_offset_m = resolved_branch_down_offset_m;

  CurveConstraint constraint = make_curve_constraint_from_port(
      port, owner_pole, chord_dir, basis_length, endpoint_offset_m, effective_sag_ratio, bend_stiffness_hint,
      min_bend_radius_hint_m, continuity_preference, pass_mode, span.placement_context, endpoint_mode,
      !is_start_endpoint);
  const bool applied_attachment_socket =
      endpoint.attachment_request.attachment_id.has_value() && endpoint.resolved_socket_id.has_value() &&
      apply_endpoint_attachment_socket(state, *endpoint.attachment_request.attachment_id, *endpoint.resolved_socket_id,
                                       &constraint);
  if (applied_attachment_socket) {
    endpoint.endpoint_source = socket_override_active
                                   ? SupportLayoutEndpointSourceKind::kAttachmentSocketOverride
                                   : SupportLayoutEndpointSourceKind::kAttachmentSocket;
  } else if (endpoint.attachment_request.kind != EndpointAttachmentRequestKind::kNone) {
    endpoint.endpoint_source = SupportLayoutEndpointSourceKind::kFallback;
  } else {
    endpoint.endpoint_source = SupportLayoutEndpointSourceKind::kPlainSupport;
  }

  endpoint.endpoint_mode = constraint.endpoint_mode;
  endpoint.endpoint_world = constraint.point;
  endpoint.departure_dir = constraint.tangent_dir;
  endpoint.endpoint_offset = constraint.endpoint_offset;
  endpoint.local_departure_length_m = constraint.support_departure_length_m;
  return endpoint;
}

void apply_endpoint_decision_to_layout_endpoint(const EndpointContinuityDecision& decision,
                                                SupportLayoutEndpoint* endpoint) {
  if (endpoint == nullptr) {
    return;
  }
  endpoint->decision = decision;
  endpoint->decision.owner_pole_id = endpoint->owner_pole_id;
}

void apply_support_layout_decision_seed_endpoint(const SupportLayoutDecisionSeedEndpoint& seed,
                                                 SupportLayoutEndpoint* endpoint) {
  if (endpoint == nullptr) {
    return;
  }
  endpoint->endpoint_node_id = seed.endpoint_node_id;
  endpoint->owner_pole_id = seed.owner_pole_id;
  endpoint->port_id = seed.port_id;
  apply_endpoint_decision_to_layout_endpoint(seed.decision, endpoint);
  endpoint->flow_kind = seed.flow_kind;
  endpoint->origin = seed.origin;
  endpoint->port_source = seed.port_source;
  endpoint->side = seed.side;
  endpoint->automatic_branch_down_offset_m = seed.automatic_branch_down_offset_m;
  endpoint->branch_down_offset_m = seed.branch_down_offset_m;
  endpoint->down_offset_variation = seed.down_offset_variation;
}

void apply_support_layout_decision_seed(const SpanSupportLayoutDecisionSeed& seed, SpanSupportLayoutEntry* layout) {
  if (layout == nullptr) {
    return;
  }
  layout->span_id = seed.span_id;
  layout->flow_kind = seed.flow_kind;
  layout->pass_mode = seed.pass_mode;
  layout->variation_flow_key = seed.variation_flow_key;
  layout->lowering_kind = seed.lowering_kind;
  apply_support_layout_decision_seed_endpoint(seed.start, &layout->start);
  apply_support_layout_decision_seed_endpoint(seed.end, &layout->end);
  layout->support_group_decisions = seed.support_group_decisions;
}

void apply_authoritative_support_layout_decisions(const SpanSupportLayoutEntry& authoritative_layout,
                                                  SpanSupportLayoutEntry* layout) {
  if (layout == nullptr) {
    return;
  }
  layout->flow_kind = authoritative_layout.flow_kind;
  layout->lowering_kind = authoritative_layout.lowering_kind;
  apply_endpoint_decision_to_layout_endpoint(authoritative_layout.start.decision, &layout->start);
  apply_endpoint_decision_to_layout_endpoint(authoritative_layout.end.decision, &layout->end);
  layout->start.flow_kind = authoritative_layout.flow_kind;
  layout->end.flow_kind = authoritative_layout.flow_kind;
  layout->start.origin = authoritative_layout.start.origin;
  layout->end.origin = authoritative_layout.end.origin;
  layout->support_group_decisions = authoritative_layout.support_group_decisions;
}

namespace {

std::pair<Vec3d, Vec3d> shared_support_anchor_points(const CoreState& state, const Pole& pole, const Vec3d& support_axis,
                                                     double z_m, const CacheState& cache_state) {
  Vec3d axis = SafeHorizontalNormalized(support_axis);
  const double mount_radius =
      cache_state.visual_settings.support_center_threshold_m + cache_state.geometry_settings.pole_clearance_m;
  const double tip_radius = mount_radius + cache_state.visual_settings.support_arm_extra_m;
  Vec3d center_world = pole.world_transform.position;
  double layout_yaw_deg = pole.world_transform.rotation_euler_deg.z;
  if (const auto pole_view = state.view().inspect_pole(pole.id); pole_view.has_value() && pole_view->has_layout_yaw) {
    layout_yaw_deg = pole_view->layout_yaw_deg;
  }
  const PoleFrame frame = BuildPoleFrame(pole.world_transform, layout_yaw_deg);
  if (std::abs(frame.up.z) > 1e-9) {
    const double local_z = (z_m - frame.origin.z) / frame.up.z;
    center_world = LocalPointToWorld(frame, {0.0, 0.0, local_z});
  } else {
    center_world.z = z_m;
  }
  const Vec3d mount{
      center_world.x + axis.x * mount_radius,
      center_world.y + axis.y * mount_radius,
      z_m,
  };
  const Vec3d tip{
      center_world.x + axis.x * tip_radius,
      center_world.y + axis.y * tip_radius,
      z_m,
  };
  return {mount, tip};
}

bool merge_layout_support_group_decision(
    std::unordered_map<LoweredSupportGroupKey, SupportGroupDecision, LoweredSupportGroupKeyHash>* groups,
    const LoweredSupportGroupKey& key, const SupportGroupDecision& group_copy) {
  if (groups == nullptr || key.owner_pole_id == kInvalidObjectId || key.support_group_id < 0) {
    return false;
  }
  auto [it, inserted] = groups->try_emplace(key, group_copy);
  if (!inserted) {
    it->second.grouped_port_count += group_copy.grouped_port_count;
    it->second.attachment_worlds.insert(it->second.attachment_worlds.end(), group_copy.attachment_worlds.begin(),
                                        group_copy.attachment_worlds.end());
    return true;
  }
  return true;
}

LoweredSupportGroupPlacement build_grouped_support_placement_from_decision(const CoreState& state,
                                                                           const SupportGroupDecision& group_decision,
                                                                           const EditState& edit_state,
                                                                           const CacheState& cache_state) {
  LoweredSupportGroupPlacement group{};
  group.grouping_rule = SupportGroupingRuleKind::kDecisionGroup;
  group.grouped_port_count = group_decision.grouped_port_count;
  group.down_offset_m = group_decision.down_offset_m;
  group.attachment_worlds = group_decision.attachment_worlds;
  group.down_offset_variation = group_decision.down_offset_variation;

  const Pole* pole = edit_state.poles.find(group_decision.decision.owner_pole_id);
  if (pole == nullptr) {
    return group;
  }
  Vec3d support_axis = group_decision.decision.side_axis;
  support_axis.z = 0.0;
  if (!Normalize(&support_axis) || !IsFiniteXY(support_axis)) {
    return group;
  }
  if (std::abs(group_decision.decision.chosen_side_sign) > 1e-9) {
    support_axis = ScaleVec(support_axis, (group_decision.decision.chosen_side_sign >= 0.0) ? 1.0 : -1.0);
  }
  const auto [mount_world, tip_world] =
      shared_support_anchor_points(state, *pole, support_axis, group_decision.support_world.z, cache_state);
  group.mount_world = mount_world;
  group.tip_world = tip_world;
  return group;
}

void apply_grouped_support_placement_to_layout_endpoint(const SupportGroupDecision& group_decision,
                                                        const LoweredSupportGroupPlacement& group,
                                                        SupportLayoutEndpoint* endpoint) {
  if (endpoint == nullptr) {
    return;
  }
  apply_endpoint_decision_to_layout_endpoint(group_decision.decision, endpoint);
  endpoint->automatic_branch_down_offset_m = group_decision.down_offset_m;
  endpoint->branch_down_offset_m = group_decision.down_offset_m;
  endpoint->down_offset_variation = group_decision.down_offset_variation;
  endpoint->support_world = group.tip_world;
}

} // namespace

void rebuild_all_lowered_support_groups(const CoreState& state, const EditState& edit_state, CacheState* cache_state) {
  if (cache_state == nullptr) {
    return;
  }
  (void)state;
  cache_state->support_layout_cache.support_group_decisions.clear();
  cache_state->support_layout_cache.lowered_support_groups.clear();
  std::vector<ObjectId> ordered_span_ids{};
  ordered_span_ids.reserve(cache_state->support_layout_cache.by_span.size());
  for (const auto& [span_id, _] : cache_state->support_layout_cache.by_span) {
    ordered_span_ids.push_back(span_id);
  }
  std::sort(ordered_span_ids.begin(), ordered_span_ids.end());
  for (ObjectId span_id : ordered_span_ids) {
    auto layout_it = cache_state->support_layout_cache.by_span.find(span_id);
    if (layout_it == cache_state->support_layout_cache.by_span.end()) {
      continue;
    }
    auto& layout = layout_it->second;
    layout.lowered_support_group_keys.clear();
    for (const auto& [key, group_copy] : layout.support_group_decisions) {
      const bool accepted =
          merge_layout_support_group_decision(&cache_state->support_layout_cache.support_group_decisions, key, group_copy);
      if (accepted &&
          std::find(layout.lowered_support_group_keys.begin(), layout.lowered_support_group_keys.end(), key) ==
              layout.lowered_support_group_keys.end()) {
        layout.lowered_support_group_keys.push_back(key);
      }
    }
  }
  for (const auto& [key, group_decision] : cache_state->support_layout_cache.support_group_decisions) {
    cache_state->support_layout_cache.lowered_support_groups[key] =
        build_grouped_support_placement_from_decision(state, group_decision, edit_state, *cache_state);
  }
  for (auto& [span_id, layout] : cache_state->support_layout_cache.by_span) {
    (void)span_id;
    if (endpoint_uses_grouped_lowered_support(&layout.start)) {
      const LoweredSupportGroupKey key = LoweredSupportGroupKeyFromDecision(layout.start.decision);
      const auto group_decision_it = cache_state->support_layout_cache.support_group_decisions.find(key);
      const auto it = cache_state->support_layout_cache.lowered_support_groups.find(key);
      if (group_decision_it != cache_state->support_layout_cache.support_group_decisions.end() &&
          it != cache_state->support_layout_cache.lowered_support_groups.end()) {
        apply_grouped_support_placement_to_layout_endpoint(group_decision_it->second, it->second, &layout.start);
      }
    }
    if (endpoint_uses_grouped_lowered_support(&layout.end)) {
      const LoweredSupportGroupKey key = LoweredSupportGroupKeyFromDecision(layout.end.decision);
      const auto group_decision_it = cache_state->support_layout_cache.support_group_decisions.find(key);
      const auto it = cache_state->support_layout_cache.lowered_support_groups.find(key);
      if (group_decision_it != cache_state->support_layout_cache.support_group_decisions.end() &&
          it != cache_state->support_layout_cache.lowered_support_groups.end()) {
        apply_grouped_support_placement_to_layout_endpoint(group_decision_it->second, it->second, &layout.end);
      }
    }
  }
}

SpanSupportLayoutEntry CoreState::generate_span_support_layout(const Span& span, std::string* error_message) const {
  const Port* port_a = authoritative_.edit_state.ports.find(span.port_a_id);
  const Port* port_b = authoritative_.edit_state.ports.find(span.port_b_id);
  if (port_a == nullptr || port_b == nullptr) {
    if (error_message != nullptr) {
      *error_message = "span endpoint port is missing";
    }
    return {};
  }

  const Vec3d a = port_a->world_position;
  const Vec3d b = port_b->world_position;
  const Pole* pole_a = authoritative_.edit_state.poles.find(port_a->owner_pole_id);
  const Pole* pole_b = authoritative_.edit_state.poles.find(port_b->owner_pole_id);
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  const double dz = b.z - a.z;
  const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
  const ResolvedSpanCurveInputs inputs =
      resolve_span_curve_inputs(*this, span, *port_a, *port_b, pole_a, pole_b, a, b, distance);

  Vec3d chord_dir = b - a;
  if (!Normalize(&chord_dir)) {
    chord_dir = WorldForward();
  }
  const double endpoint_offset_m = std::min(std::max(0.02, inputs.basis_length * 0.03), 0.35);

  SpanSupportLayoutEntry layout{};
  layout.span_id = span.id;
  layout.flow_kind = inputs.flow_kind;
  layout.pass_mode = inputs.pass_mode;
  layout.basis_length_m = inputs.basis_length;
  layout.effective_sag_ratio = inputs.effective_sag_ratio;
  layout.continuity_preference = inputs.continuity_preference;
  layout.bend_stiffness_hint = inputs.bend_stiffness_hint;
  layout.min_bend_radius_hint_m = inputs.min_bend_radius_hint_m;
  layout.variation_flow_key = inputs.variation_flow_key;
  layout.sag_variation = inputs.sag_variation;
  const SpanSupportLayoutDecisionSeed* decision_seed = find_span_support_layout_seed(span.id);
  const SpanSupportLayoutEntry* existing_layout = find_span_support_layout(span.id);
  const SpanSupportLayoutEntry* authoritative_layout = (decision_seed == nullptr) ? existing_layout : nullptr;
  const int resolved_socket_a = resolve_span_endpoint_socket_id(span, true);
  const int resolved_socket_b = resolve_span_endpoint_socket_id(span, false);
  const bool socket_override_a = has_span_endpoint_socket_override(span.id, true);
  const bool socket_override_b = has_span_endpoint_socket_override(span.id, false);
  HierarchicalVariationSample down_offset_variation_a{};
  HierarchicalVariationSample down_offset_variation_b{};
  const auto automatic_branch_down_offset_for_endpoint =
      [&](const Port& port, bool is_start_endpoint, HierarchicalVariationSample* variation) {
        if (decision_seed != nullptr) {
          const SupportLayoutDecisionSeedEndpoint& endpoint = is_start_endpoint ? decision_seed->start : decision_seed->end;
          if (variation != nullptr) {
            *variation = endpoint.down_offset_variation;
          }
          return endpoint.automatic_branch_down_offset_m;
        }
        if (authoritative_layout != nullptr) {
          const SupportLayoutEndpoint& endpoint = is_start_endpoint ? authoritative_layout->start : authoritative_layout->end;
          if (variation != nullptr) {
            *variation = endpoint.down_offset_variation;
          }
          return endpoint.automatic_branch_down_offset_m;
        }
        if (variation != nullptr) {
          *variation = {};
        }
        return fallback_branch_down_offset_for_support_port(*this, port);
      };
  const double automatic_branch_down_offset_a =
      automatic_branch_down_offset_for_endpoint(*port_a, true, &down_offset_variation_a);
  const double automatic_branch_down_offset_b =
      automatic_branch_down_offset_for_endpoint(*port_b, false, &down_offset_variation_b);
  const double resolved_branch_down_offset_a =
      resolve_span_branch_down_offset_m(span, automatic_branch_down_offset_a);
  const double resolved_branch_down_offset_b =
      resolve_span_branch_down_offset_m(span, automatic_branch_down_offset_b);
  layout.start = build_support_layout_endpoint(
      *this, span, *port_a, pole_a, chord_dir, inputs.basis_length, endpoint_offset_m, inputs.effective_sag_ratio,
      inputs.bend_stiffness_hint, inputs.min_bend_radius_hint_m, inputs.continuity_preference, inputs.pass_mode,
      inputs.endpoint_mode, inputs.flow_kind, resolved_socket_a, socket_override_a, automatic_branch_down_offset_a,
      down_offset_variation_a, resolved_branch_down_offset_a, true);
  layout.end = build_support_layout_endpoint(
      *this, span, *port_b, pole_b, chord_dir, inputs.basis_length, endpoint_offset_m, inputs.effective_sag_ratio,
      inputs.bend_stiffness_hint, inputs.min_bend_radius_hint_m, inputs.continuity_preference, inputs.pass_mode,
      inputs.endpoint_mode, inputs.flow_kind, resolved_socket_b, socket_override_b, automatic_branch_down_offset_b,
      down_offset_variation_b, resolved_branch_down_offset_b, false);
  if (decision_seed != nullptr) {
    apply_support_layout_decision_seed(*decision_seed, &layout);
  } else if (authoritative_layout != nullptr) {
    apply_authoritative_support_layout_decisions(*authoritative_layout, &layout);
  }
  layout.detail_curve_profile_hint = detail_curve_profile_hint_from_support_layout(layout);
  return layout;
}

} // namespace wire::core

