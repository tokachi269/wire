#include "wire/core/core_state.hpp"
#include "wire/core/coord_utils.hpp"
#include "../generation/support_policy.hpp"
#include "../support_orientation_utils.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <unordered_set>
#include <vector>

namespace wire::core {

namespace {
constexpr double kZeroLengthEps = 1e-9;

struct AttachmentSocketWorld {
  int socket_id = -1;
  Vec3d world_position{};
  Vec3d world_tangent{};
  double local_forward_x = 0.0;
};

struct ResolvedSpanCurveInputs {
  double basis_length = 0.0;
  BackboneFlowKind flow_kind = BackboneFlowKind::kMain;
  std::uint64_t variation_flow_key = 0;
  HierarchicalVariationSample sag_variation{};
  double effective_sag_ratio = 0.0;
  CurvePassMode pass_mode = CurvePassMode::kPassThrough;
  CurveEndpointMode endpoint_mode = CurveEndpointMode::kDirectThrough;
  CableContinuityPolicyHint continuity_preference = CableContinuityPolicyHint::kAuto;
  double bend_stiffness_hint = 1.0;
  double min_bend_radius_hint_m = 0.0;
};

bool endpoint_uses_grouped_lowered_support(const SupportLayoutEndpoint* endpoint, BackboneLoweringKind lowering_kind) {
  return endpoint != nullptr && UsesGroupedLoweredSupport(*endpoint, lowering_kind);
}

std::pair<Vec3d, Vec3d> shared_support_anchor_points(const Pole& pole, const Vec3d& support_axis, double z_m,
                                                     const CacheState& cache_state) {
  Vec3d axis = SafeHorizontalNormalized(support_axis, {1.0, 0.0, 0.0});
  const double mount_radius =
      cache_state.visual_settings.support_center_threshold_m + cache_state.geometry_settings.pole_clearance_m;
  const double tip_radius = mount_radius + cache_state.visual_settings.support_arm_extra_m;
  const Vec3d mount{
      pole.world_transform.position.x + axis.x * mount_radius,
      pole.world_transform.position.y + axis.y * mount_radius,
      z_m,
  };
  const Vec3d tip{
      pole.world_transform.position.x + axis.x * tip_radius,
      pole.world_transform.position.y + axis.y * tip_radius,
      z_m,
  };
  return {mount, tip};
}

void append_lowered_support_group_placement(std::vector<LoweredSupportGroupPlacement>* groups, const EditState& edit_state,
                                            const CacheState& cache_state, const SpanSupportLayoutEntry& layout,
                                            const SupportLayoutEndpoint& endpoint) {
  if (groups == nullptr || !endpoint_uses_grouped_lowered_support(&endpoint, layout.lowering_kind)) {
    return;
  }
  const Pole* pole = edit_state.poles.find(endpoint.owner_pole_id);
  if (pole == nullptr) {
    return;
  }
  const Port* port = edit_state.ports.find(endpoint.port_id);
  const Vec3d attachment_world = (port == nullptr) ? endpoint.endpoint_world : port->world_position;
  const int support_group_id = SupportGroupIdForEndpoint(endpoint.owner_pole_id, endpoint.decision);
  auto existing = std::find_if(groups->begin(), groups->end(), [&](const LoweredSupportGroupPlacement& group) {
    return group.owner_pole_id == endpoint.owner_pole_id && group.support_group_id == support_group_id;
  });
  Vec3d pole_to_tip = endpoint.support_world - pole->world_transform.position;
  const Vec3d support_axis =
      CanonicalSharedSupportAxis(endpoint.has_side_axis ? endpoint.side_axis : pole_to_tip, pole_to_tip);
  const auto [mount_world, tip_world] =
      shared_support_anchor_points(*pole, support_axis, endpoint.support_world.z, cache_state);
  if (existing == groups->end()) {
    LoweredSupportGroupPlacement group{};
    group.owner_pole_id = endpoint.owner_pole_id;
    group.decision = endpoint.decision;
    group.side = endpoint.side;
    group.origin = endpoint.origin;
    group.grouping_rule = SupportGroupingRuleKind::kDecisionGroup;
    group.support_group_id = support_group_id;
    group.grouped_port_count = 1;
    group.bundle_order_policy = endpoint.bundle_order_policy;
    group.bundle_order_choice = endpoint.bundle_order_choice;
    group.bundle_order_choice_reason = endpoint.bundle_order_choice_reason;
    group.side_assignment_rule = endpoint.side_assignment_rule;
    group.support_orientation_rule = endpoint.support_orientation_rule;
    group.used_junction_pair_side_assignment = endpoint.used_junction_pair_side_assignment;
    group.has_side_axis = endpoint.has_side_axis;
    group.side_axis = endpoint.side_axis;
    group.chosen_side_sign = endpoint.chosen_side_sign;
    group.down_offset_m = endpoint.branch_down_offset_m;
    group.mount_world = mount_world;
    group.tip_world = tip_world;
    group.attachment_worlds.push_back(attachment_world);
    group.down_offset_variation = endpoint.down_offset_variation;
    groups->push_back(std::move(group));
    return;
  }
  existing->grouped_port_count += 1;
  existing->attachment_worlds.push_back(attachment_world);
}

std::vector<LoweredSupportGroupPlacement>
build_lowered_support_group_placements(const EditState& edit_state, const CacheState& cache_state,
                                       const std::vector<const SpanSupportLayoutEntry*>& layouts) {
  std::vector<LoweredSupportGroupPlacement> groups{};
  for (const SpanSupportLayoutEntry* layout : layouts) {
    if (layout == nullptr) {
      continue;
    }
    append_lowered_support_group_placement(&groups, edit_state, cache_state, *layout, layout->start);
    append_lowered_support_group_placement(&groups, edit_state, cache_state, *layout, layout->end);
  }
  return groups;
}

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

Vec3d attachment_direction_to_world(const Vec3d& forward, const Vec3d& lateral, const Vec3d& up, const Vec3d& local_dir,
                                    const Vec3d& fallback) {
  Vec3d world =
      ScaleVec(forward, local_dir.x) + ScaleVec(lateral, local_dir.y) + ScaleVec(up, local_dir.z);
  if (!Normalize(&world)) {
    return fallback;
  }
  return world;
}

const AttachmentSocketTemplate* find_attachment_socket(const AttachmentTemplate& attachment_template, int socket_id) {
  for (const AttachmentSocketTemplate& socket : attachment_template.sockets) {
    if (socket.id == socket_id) {
      return &socket;
    }
  }
  return nullptr;
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

std::vector<CurveLengthInterval> merged_intervals(std::vector<CurveLengthInterval> intervals, double total_length_m) {
  std::vector<CurveLengthInterval> merged{};
  if (intervals.empty() || total_length_m <= kZeroLengthEps) {
    return merged;
  }
  for (CurveLengthInterval& interval : intervals) {
    interval.start_m = std::clamp(interval.start_m, 0.0, total_length_m);
    interval.end_m = std::clamp(interval.end_m, 0.0, total_length_m);
    if (interval.end_m < interval.start_m) {
      std::swap(interval.start_m, interval.end_m);
    }
  }
  std::sort(intervals.begin(), intervals.end(),
            [](const CurveLengthInterval& a, const CurveLengthInterval& b) { return a.start_m < b.start_m; });
  for (const CurveLengthInterval& interval : intervals) {
    if (interval.end_m - interval.start_m <= kZeroLengthEps) {
      continue;
    }
    if (merged.empty() || interval.start_m > merged.back().end_m + 1e-6) {
      merged.push_back(interval);
    } else {
      merged.back().end_m = std::max(merged.back().end_m, interval.end_m);
    }
  }
  return merged;
}

std::vector<CurveLengthInterval> visible_intervals_from_hidden(const std::vector<CurveLengthInterval>& hidden_intervals,
                                                               double total_length_m) {
  std::vector<CurveLengthInterval> visible{};
  double cursor = 0.0;
  for (const CurveLengthInterval& interval : hidden_intervals) {
    if (interval.start_m > cursor + kZeroLengthEps) {
      visible.push_back({cursor, interval.start_m});
    }
    cursor = std::max(cursor, interval.end_m);
  }
  if (cursor < total_length_m - kZeroLengthEps) {
    visible.push_back({cursor, total_length_m});
  }
  if (visible.empty() && total_length_m > kZeroLengthEps && hidden_intervals.empty()) {
    visible.push_back({0.0, total_length_m});
  }
  return visible;
}

std::vector<Vec3d> sample_curve_interval_points(const DetailCurve& curve, double start_s, double end_s, int samples) {
  std::vector<Vec3d> points{};
  if (curve.Length() <= kZeroLengthEps || samples < 2) {
    return points;
  }
  const double clamped_start = std::clamp(start_s, 0.0, curve.Length());
  const double clamped_end = std::clamp(end_s, 0.0, curve.Length());
  if (clamped_end - clamped_start <= kZeroLengthEps) {
    return points;
  }
  points.reserve(static_cast<std::size_t>(samples));
  for (int i = 0; i < samples; ++i) {
    const double t = (samples == 1) ? 0.0 : static_cast<double>(i) / static_cast<double>(samples - 1);
    points.push_back(curve.PositionAtLength(clamped_start + (clamped_end - clamped_start) * t));
  }
  return points;
}

std::uint64_t splitmix64(std::uint64_t x) {
  x += 0x9E3779B97F4A7C15ull;
  x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ull;
  x = (x ^ (x >> 27)) * 0x94D049BB133111EBull;
  return x ^ (x >> 31);
}

std::uint64_t hash_combine(std::uint64_t seed, std::uint64_t value) {
  return splitmix64(seed ^ (value + 0x9E3779B97F4A7C15ull + (seed << 6) + (seed >> 2)));
}

CurvePassMode curve_pass_mode_from_context(ConnectionContext context) {
  switch (context) {
  case ConnectionContext::kBranchAdd:
    return CurvePassMode::kBranch;
  case ConnectionContext::kDropAdd:
    return CurvePassMode::kTerminate;
  default:
    return CurvePassMode::kPassThrough;
  }
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

const SupportLayoutEndpoint* support_layout_endpoint_for_owner(const SpanSupportLayoutEntry* layout, ObjectId owner_pole_id) {
  if (layout == nullptr || owner_pole_id == kInvalidObjectId) {
    return nullptr;
  }
  if (layout->start.owner_pole_id == owner_pole_id) {
    return &layout->start;
  }
  if (layout->end.owner_pole_id == owner_pole_id) {
    return &layout->end;
  }
  return nullptr;
}

void apply_endpoint_decision_to_layout_endpoint(const EndpointContinuityDecision& decision,
                                                SupportLayoutEndpoint* endpoint) {
  if (endpoint == nullptr) {
    return;
  }
  endpoint->decision = decision;
  endpoint->relation_kind = decision.relation_kind;
  endpoint->continuity_class = decision.continuity_class;
  endpoint->default_lower_required = decision.default_lower_required;
  endpoint->bundle_order_policy = decision.bundle_order_policy;
  endpoint->bundle_order_choice = decision.bundle_order_choice;
  endpoint->bundle_order_choice_reason = decision.bundle_order_choice_reason;
  endpoint->side_assignment_rule = decision.side_assignment_rule;
  endpoint->support_orientation_rule = decision.support_orientation_rule;
  endpoint->used_junction_pair_side_assignment = decision.used_junction_pair_side_assignment;
  endpoint->has_side_axis = decision.has_side_axis;
  endpoint->side_axis = decision.side_axis;
  endpoint->chosen_side_sign = decision.chosen_side_sign;
  endpoint->same_level_feasible = decision.same_level_feasible;
  endpoint->same_level_reason = decision.same_level_reason;
  endpoint->projected_spacing_topview_m = decision.projected_spacing_topview_m;
  endpoint->required_clearance_m = decision.required_clearance_m;
  endpoint->lowering_blocked_by_policy = decision.lowering_blocked_by_policy;
  endpoint->unresolved_same_level_conflict = decision.unresolved_same_level_conflict;
  endpoint->solver_used_same_level_constraint = decision.solver_used_same_level_constraint;
  endpoint->used_special_case_ports = decision.used_special_case_ports;
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

double branch_down_offset_for_port(const CoreState& state, const Port& port, std::uint64_t flow_key,
                                   HierarchicalVariationSample* out_variation) {
  if (out_variation != nullptr) {
    *out_variation = {};
  }
  if (port.placement_source != PortPlacementSourceKind::kBranchSupport) {
    return 0.0;
  }
  const Pole* pole = state.view().poles().find(port.owner_pole_id);
  if (pole == nullptr) {
    return 0.0;
  }
  double main_support_base_z_m = std::max(0.5, pole->height_m * 0.8);
  const auto pole_type_it = state.view().pole_types().find(pole->pole_type_id);
  if (pole_type_it != state.view().pole_types().end()) {
    const PoleTypeDefinition& pole_type = pole_type_it->second;
    double best_band_z = -std::numeric_limits<double>::infinity();
    const int target_layer = generation::detail::TemplateLayerForCategory(port.category);
    for (const PortPlacementBand& band : pole_type.port_bands) {
      if (band.enabled && band.layer == target_layer) {
        best_band_z = std::max(best_band_z, band.height_max_m);
      }
    }
    if (!std::isfinite(best_band_z)) {
      for (const PortPlacementBand& band : pole_type.port_bands) {
        if (band.enabled && band.category == port.category) {
          best_band_z = std::max(best_band_z, band.height_max_m);
        }
      }
    }
    if (std::isfinite(best_band_z)) {
      main_support_base_z_m = best_band_z;
    }
  }

  VariationContext down_offset_context{};
  down_offset_context.world_position = port.world_position;
  down_offset_context.flow_key = flow_key;
  down_offset_context.pole_id = pole->id;
  down_offset_context.secondary_pole_id = kInvalidObjectId;
  down_offset_context.local_key = static_cast<std::uint64_t>(port.id);
  const HierarchicalVariationSample variation =
      EvaluateHierarchicalVariation(state.view().variation_settings(), down_offset_context);
  if (out_variation != nullptr) {
    *out_variation = variation;
  }

  const double local_z_m = HeightAlongWorldUp(port.world_position) - HeightAlongWorldUp(pole->world_transform.position);
  const double varied_delta =
      variation.final_value * state.view().variation_settings().branch_down_offset_variation_scale;
  return std::max(0.0, main_support_base_z_m - local_z_m + varied_delta);
}

double branch_down_offset_for_assignment(const CoreState& state, const Port& port, std::uint64_t flow_key,
                                         double base_offset_m, HierarchicalVariationSample* out_variation) {
  if (out_variation != nullptr) {
    *out_variation = {};
  }
  if (base_offset_m <= 0.0) {
    return 0.0;
  }
  const Pole* pole = state.view().poles().find(port.owner_pole_id);
  if (pole == nullptr) {
    return std::max(0.0, base_offset_m);
  }

  VariationContext down_offset_context{};
  down_offset_context.world_position = port.world_position;
  down_offset_context.flow_key = flow_key;
  down_offset_context.pole_id = pole->id;
  down_offset_context.secondary_pole_id = kInvalidObjectId;
  down_offset_context.local_key = static_cast<std::uint64_t>(port.id);
  const HierarchicalVariationSample variation =
      EvaluateHierarchicalVariation(state.view().variation_settings(), down_offset_context);
  if (out_variation != nullptr) {
    *out_variation = variation;
  }

  const double varied_delta =
      variation.final_value * state.view().variation_settings().branch_down_offset_variation_scale;
  return std::max(0.0, base_offset_m + varied_delta);
}

SupportLayoutEndpoint build_support_layout_endpoint(const CoreState& state, const Span& span, const Port& port,
                                                    const Pole* owner_pole, const Vec3d& chord_dir,
                                                    double basis_length, double endpoint_offset_m,
                                                    double effective_sag_ratio, double bend_stiffness_hint,
                                                    double min_bend_radius_hint_m,
                                                    CableContinuityPolicyHint continuity_preference,
                                                    CurvePassMode pass_mode, CurveEndpointMode endpoint_mode,
                                                    std::uint64_t flow_key, BackboneFlowKind flow_kind,
                                                    int resolved_socket_id, bool socket_override_active,
                                                    double automatic_branch_down_offset_m,
                                                    const HierarchicalVariationSample& down_offset_variation,
                                                    double resolved_branch_down_offset_m,
                                                    bool is_start_endpoint) {
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
    const BackboneLoweringKind endpoint_lowering_kind =
        (endpoint.decision.lower_required && !endpoint.decision.lowering_blocked_by_policy)
            ? ((endpoint.decision.relation_kind == JunctionRelationKind::kCrossUnderpass)
                   ? BackboneLoweringKind::kCrossUnderpass
                   : BackboneLoweringKind::kBranchSupport)
            : BackboneLoweringKind::kNone;
    if (owner_pole != nullptr && endpoint_uses_grouped_lowered_support(&endpoint, endpoint_lowering_kind)) {
      const Port* other_port =
          (port.id == span.port_a_id) ? state.view().ports().find(span.port_b_id) : state.view().ports().find(span.port_a_id);
      if (other_port != nullptr) {
        Vec3d support_axis =
          ResolveSupportAxisForEndpoint(port, span, *other_port, *owner_pole, &endpoint, state.view().edit_state());
        support_axis = CanonicalSharedSupportAxis(support_axis, endpoint.has_side_axis ? endpoint.side_axis : support_axis);
        const auto [mount_world, tip_world] = shared_support_anchor_points(
            *owner_pole, support_axis, port.world_position.z + endpoint.branch_down_offset_m, state.view().cache_state());
        (void)mount_world;
        endpoint.support_world = tip_world;
      }
    }
    return endpoint;
  }

CurveConstraint make_curve_constraint_from_support_layout(const SupportLayoutEndpoint& endpoint, const Pole* owner_pole,
                                                          double basis_length, double effective_sag_ratio,
                                                          double bend_stiffness_hint, double min_bend_radius_hint_m,
                                                          CableContinuityPolicyHint continuity_preference,
                                                          CurvePassMode pass_mode,
                                                          ConnectionContext connection_context) {
  CurveConstraint constraint{};
  constraint.point = endpoint.endpoint_world;
  constraint.tangent_dir = endpoint.departure_dir;
  constraint.tangent_strength = 1.0;
  constraint.tangent_length_hint_m = basis_length * 0.30;
  constraint.support_departure_length_m = endpoint.local_departure_length_m;
  constraint.bend_stiffness_hint = bend_stiffness_hint;
  constraint.min_bend_radius_hint_m = min_bend_radius_hint_m;
  constraint.endpoint_offset = endpoint.endpoint_offset;
  constraint.sag_hint = effective_sag_ratio * 0.5;
  constraint.slack_hint = 0.0;
  constraint.continuity_preference = continuity_preference;
  constraint.pass_mode = pass_mode;
  constraint.endpoint_mode = endpoint.endpoint_mode;
  constraint.corner_pass =
      (connection_context == ConnectionContext::kCornerPass) && owner_pole != nullptr &&
      owner_pole->context.kind == PoleContextKind::kCorner && owner_pole->context.corner_angle_deg > 1e-6;
  constraint.corner_angle_deg = (owner_pole == nullptr) ? 0.0 : owner_pole->context.corner_angle_deg;
  return constraint;
}

std::uint64_t fallback_variation_flow_key_for_span(const Span& span) {
  std::uint64_t key = hash_combine(span.generation.generation_session_id, static_cast<std::uint64_t>(span.bundle_id));
  key = hash_combine(key, static_cast<std::uint64_t>(span.endpoint_node_a_id));
  key = hash_combine(key, static_cast<std::uint64_t>(span.endpoint_node_b_id));
  key = hash_combine(key, static_cast<std::uint64_t>(span.placement_context));
  return key;
}

std::uint64_t variation_flow_key_for_span(const SpanRuntimeState* runtime, const Span& span) {
  if (runtime != nullptr && runtime->variation_flow_key != 0) {
    return runtime->variation_flow_key;
  }
  return fallback_variation_flow_key_for_span(span);
}

// Support layout is the authoritative aggregation point for span endpoint constraints.
// Detail-curve generation must consume the same resolved span-level policy inputs so
// branch/main/attachment behavior is not reinterpreted through a parallel code path.
ResolvedSpanCurveInputs resolve_span_curve_inputs(const CoreState& state, const Span& span, const Port& port_a,
                                                  const Port& port_b, const Pole* pole_a, const Pole* pole_b,
                                                  const Vec3d& a, const Vec3d& b, double distance) {
  const CoreView view = state.view();
  const Bundle* bundle = view.bundles().find(span.bundle_id);
  const BundleTemplate* bundle_template = nullptr;
  if (bundle != nullptr) {
    const auto it_bundle_template = view.bundle_templates().find(bundle->bundle_template_id);
    if (it_bundle_template != view.bundle_templates().end()) {
      bundle_template = &it_bundle_template->second;
    }
  }
  const CableTemplate* cable_template = nullptr;
  if (bundle_template != nullptr) {
    const auto it_cable = view.cable_templates().find(bundle_template->cable_template_id);
    if (it_cable != view.cable_templates().end()) {
      cable_template = &it_cable->second;
    }
  }

  const double sag_ratio = (cable_template == nullptr) ? view.geometry_settings().sag_factor
                                                       : (cable_template->sag_factor + cable_template->slack_factor);
  const bool use_reference_length = true;

  const SpanSupportLayoutEntry* existing_layout = state.find_span_support_layout(span.id);

  ResolvedSpanCurveInputs inputs{};
  inputs.basis_length =
      (use_reference_length && span.reference_length_m > kZeroLengthEps) ? span.reference_length_m : distance;
  if (existing_layout != nullptr) {
    inputs.flow_kind = existing_layout->flow_kind;
  } else {
    inputs.flow_kind = support_layout_flow_kind_for_span(span, port_a, port_b);
  }
  inputs.pass_mode = curve_pass_mode_from_context(span.placement_context);
  inputs.endpoint_mode = curve_endpoint_mode_for_template(cable_template, bundle, bundle_template);
  inputs.continuity_preference =
      (cable_template == nullptr) ? CableContinuityPolicyHint::kAuto : cable_template->continuity_policy;
  inputs.bend_stiffness_hint = (cable_template == nullptr) ? 1.0 : cable_template->bend_stiffness;
  inputs.min_bend_radius_hint_m = (cable_template == nullptr) ? 0.0 : cable_template->min_bend_radius_m;

  const SpanRuntimeState* runtime = view.find_span_runtime_state(span.id);
  inputs.variation_flow_key = variation_flow_key_for_span(runtime, span);
  VariationContext sag_variation_context{};
  sag_variation_context.world_position = {(a.x + b.x) * 0.5, (a.y + b.y) * 0.5, (a.z + b.z) * 0.5};
  sag_variation_context.flow_key = inputs.variation_flow_key;
  sag_variation_context.pole_id = (pole_a == nullptr) ? kInvalidObjectId : pole_a->id;
  sag_variation_context.secondary_pole_id = (pole_b == nullptr) ? kInvalidObjectId : pole_b->id;
  sag_variation_context.local_key = static_cast<std::uint64_t>(span.id);
  inputs.sag_variation = EvaluateHierarchicalVariation(view.variation_settings(), sag_variation_context);
  const double sag_multiplier =
      std::max(0.0, 1.0 + inputs.sag_variation.final_value * view.variation_settings().sag_variation_scale);
  inputs.effective_sag_ratio =
      (view.geometry_settings().sag_enabled && inputs.basis_length > kZeroLengthEps) ? (sag_ratio * sag_multiplier) : 0.0;
  return inputs;
}
}

const CurveCacheEntry* CoreState::find_curve_cache(ObjectId span_id) const {
  auto it = cache_state_.curve_cache.by_span.find(span_id);
  if (it == cache_state_.curve_cache.by_span.end()) {
    return nullptr;
  }
  return &it->second;
}

const BoundsCacheEntry* CoreState::find_bounds_cache(ObjectId span_id) const {
  auto it = cache_state_.bounds_cache.by_span.find(span_id);
  if (it == cache_state_.bounds_cache.by_span.end()) {
    return nullptr;
  }
  return &it->second;
}

const SpanSupportLayoutEntry* CoreState::find_span_support_layout(ObjectId span_id) const {
  auto it = cache_state_.support_layout_cache.by_span.find(span_id);
  if (it == cache_state_.support_layout_cache.by_span.end()) {
    return nullptr;
  }
  return &it->second;
}

const SpanVisualCacheEntry* CoreState::find_span_visual_cache(ObjectId span_id) const {
  auto it = cache_state_.visual_cache.by_span.find(span_id);
  if (it == cache_state_.visual_cache.by_span.end()) {
    return nullptr;
  }
  return &it->second;
}

const SpanRenderCacheEntry* CoreState::find_span_render_cache(ObjectId span_id) const {
  auto it = cache_state_.render_cache.by_span.find(span_id);
  if (it == cache_state_.render_cache.by_span.end()) {
    return nullptr;
  }
  return &it->second;
}

RecalcStats CoreState::ProcessDirtyQueues() {
  RecalcStats stats{};

  std::unordered_set<ObjectId> processed_topology;
  std::unordered_set<ObjectId> processed_geometry;
  std::unordered_set<ObjectId> processed_bounds;
  std::unordered_set<ObjectId> processed_render;
  std::unordered_set<ObjectId> processed_raycast;

  for (ObjectId span_id : dirty_queue_.topology_dirty_span_ids) {
    if (!processed_topology.insert(span_id).second) {
      continue;
    }
    auto it = span_runtime_states_.find(span_id);
    if (it == span_runtime_states_.end() || !any(it->second.dirty_bits, DirtyBits::kTopology)) {
      continue;
    }
    it->second.dirty_bits = it->second.dirty_bits & ~DirtyBits::kTopology;
    ++stats.topology_processed;
  }

  for (ObjectId span_id : dirty_queue_.geometry_dirty_span_ids) {
    if (!processed_geometry.insert(span_id).second) {
      continue;
    }
    auto it = span_runtime_states_.find(span_id);
    if (it == span_runtime_states_.end() || !any(it->second.dirty_bits, DirtyBits::kGeometry)) {
      continue;
    }

    std::string error_message;
    if (!rebuild_span_curve(span_id, &error_message)) {
      continue;
    }
    it->second.geometry_version = it->second.data_version;
    it->second.dirty_bits = it->second.dirty_bits & ~DirtyBits::kGeometry;
    ++stats.geometry_processed;
    mark_span_dirty(span_id, DirtyBits::kBounds | DirtyBits::kRender, false);
  }

  std::vector<ObjectId> bounds_queue = dirty_queue_.bounds_dirty_span_ids;
  for (ObjectId span_id : bounds_queue) {
    if (!processed_bounds.insert(span_id).second) {
      continue;
    }
    auto it = span_runtime_states_.find(span_id);
    if (it == span_runtime_states_.end() || !any(it->second.dirty_bits, DirtyBits::kBounds)) {
      continue;
    }

    std::string error_message;
    if (!rebuild_span_bounds(span_id, &error_message)) {
      continue;
    }
    it->second.bounds_version = it->second.data_version;
    it->second.dirty_bits = it->second.dirty_bits & ~DirtyBits::kBounds;
    ++stats.bounds_processed;
  }

  std::vector<ObjectId> render_queue = dirty_queue_.render_dirty_span_ids;
  for (ObjectId span_id : render_queue) {
    if (!processed_render.insert(span_id).second) {
      continue;
    }
    auto it = span_runtime_states_.find(span_id);
    if (it == span_runtime_states_.end() || !any(it->second.dirty_bits, DirtyBits::kRender)) {
      continue;
    }
    std::string error_message;
    if (!rebuild_span_visual(span_id, &error_message)) {
      continue;
    }
    it->second.render_version = it->second.data_version;
    it->second.dirty_bits = it->second.dirty_bits & ~DirtyBits::kRender;
    ++stats.render_processed;
  }

  for (ObjectId span_id : dirty_queue_.raycast_dirty_span_ids) {
    if (!processed_raycast.insert(span_id).second) {
      continue;
    }
    auto it = span_runtime_states_.find(span_id);
    if (it == span_runtime_states_.end() || !any(it->second.dirty_bits, DirtyBits::kRaycast)) {
      continue;
    }
    it->second.raycast_version = it->second.data_version;
    it->second.dirty_bits = it->second.dirty_bits & ~DirtyBits::kRaycast;
    ++stats.raycast_processed;
  }

  dirty_queue_ = DirtyQueue{};
  last_recalc_stats_ = stats;
  return stats;
}

CommitResult CoreState::Commit() {
  return Commit(CommitOptions{});
}

CommitResult CoreState::Commit(const CommitOptions& options) {
  CommitResult out{};
  if (options.run_recalc) {
    out.recalc_stats = ProcessDirtyQueues();
  }
  if (options.run_validate_fast) {
    out.validation = ValidateFast();
  }
  if (options.run_validate) {
    out.validation = Validate();
  }
  return out;
}

const SpanRuntimeState* CoreState::find_span_runtime_state(ObjectId span_id) const {
  auto it = span_runtime_states_.find(span_id);
  if (it == span_runtime_states_.end()) {
    return nullptr;
  }
  return &it->second;
}

void CoreState::remove_span_from_indexes(const Span& span) {
  index_remove(connection_index_.spans_by_port, span.port_a_id, span.id);
  index_remove(connection_index_.spans_by_port, span.port_b_id, span.id);
  if (span.anchor_a_id != kInvalidObjectId) {
    index_remove(connection_index_.spans_by_anchor, span.anchor_a_id, span.id);
  }
  if (span.anchor_b_id != kInvalidObjectId) {
    index_remove(connection_index_.spans_by_anchor, span.anchor_b_id, span.id);
  }
  if (span.bundle_id != kInvalidObjectId) {
    index_remove(relation_index_.spans_by_bundle, span.bundle_id, span.id);
  }
}

void CoreState::add_span_to_index(const Span& span) {
  index_add(connection_index_.spans_by_port, span.port_a_id, span.id);
  index_add(connection_index_.spans_by_port, span.port_b_id, span.id);
  if (span.anchor_a_id != kInvalidObjectId) {
    index_add(connection_index_.spans_by_anchor, span.anchor_a_id, span.id);
  }
  if (span.anchor_b_id != kInvalidObjectId) {
    index_add(connection_index_.spans_by_anchor, span.anchor_b_id, span.id);
  }
  if (span.bundle_id != kInvalidObjectId) {
    index_add(relation_index_.spans_by_bundle, span.bundle_id, span.id);
  }
}

void CoreState::initialize_span_runtime_state(ObjectId span_id) {
  SpanRuntimeState runtime{};
  runtime.span_id = span_id;
  runtime.data_version = 0;
  runtime.geometry_version = 0;
  runtime.bounds_version = 0;
  runtime.render_version = 0;
  runtime.raycast_version = 0;
  runtime.variation_flow_key = 0;
  runtime.dirty_bits = DirtyBits::kNone;
  span_runtime_states_[span_id] = runtime;
}

void CoreState::mark_span_dirty(ObjectId span_id, DirtyBits dirty_bits, bool bump_data_version) {
  if (edit_state_.spans.find(span_id) == nullptr) {
    return;
  }
  SpanRuntimeState& runtime = span_runtime_states_[span_id];
  if (runtime.span_id == kInvalidObjectId) {
    runtime.span_id = span_id;
  }
  if (bump_data_version || runtime.data_version == 0) {
    runtime.data_version = next_data_version_++;
  }
  runtime.dirty_bits |= dirty_bits;
  add_dirty_queue(span_id, dirty_bits);
}

void CoreState::add_dirty_queue(ObjectId span_id, DirtyBits dirty_bits) {
  if (any(dirty_bits, DirtyBits::kTopology))
    dirty_queue_.topology_dirty_span_ids.push_back(span_id);
  if (any(dirty_bits, DirtyBits::kGeometry))
    dirty_queue_.geometry_dirty_span_ids.push_back(span_id);
  if (any(dirty_bits, DirtyBits::kBounds))
    dirty_queue_.bounds_dirty_span_ids.push_back(span_id);
  if (any(dirty_bits, DirtyBits::kRender))
    dirty_queue_.render_dirty_span_ids.push_back(span_id);
  if (any(dirty_bits, DirtyBits::kRaycast))
    dirty_queue_.raycast_dirty_span_ids.push_back(span_id);
}

void CoreState::mark_connected_spans_dirty_from_port(ObjectId port_id, DirtyBits dirty_bits, ChangeSet* change_set) {
  auto it = connection_index_.spans_by_port.find(port_id);
  if (it == connection_index_.spans_by_port.end()) {
    return;
  }
  for (ObjectId span_id : it->second) {
    mark_span_dirty(span_id, dirty_bits, true);
    if (change_set != nullptr) {
      add_unique_id(change_set->dirty_span_ids, span_id);
      add_unique_id(change_set->updated_ids, span_id);
    }
  }
}

void CoreState::mark_connected_spans_dirty_from_anchor(ObjectId anchor_id, DirtyBits dirty_bits,
                                                       ChangeSet* change_set) {
  auto it = connection_index_.spans_by_anchor.find(anchor_id);
  if (it == connection_index_.spans_by_anchor.end()) {
    return;
  }
  for (ObjectId span_id : it->second) {
    mark_span_dirty(span_id, dirty_bits, true);
    if (change_set != nullptr) {
      add_unique_id(change_set->dirty_span_ids, span_id);
      add_unique_id(change_set->updated_ids, span_id);
    }
  }
}

bool CoreState::rebuild_span_curve(ObjectId span_id, std::string* error_message) {
  const Span* span = edit_state_.spans.find(span_id);
  if (span == nullptr) {
    if (error_message != nullptr) {
      *error_message = "span not found";
    }
    return false;
  }

  SpanSupportLayoutEntry support_layout = generate_span_support_layout(*span, error_message);
  if (support_layout.span_id == kInvalidObjectId) {
    return false;
  }

  DetailCurve detail = generate_span_curve(*span, support_layout, error_message);
  if (detail.sample_points.size() < 2) {
    if (error_message != nullptr && error_message->empty()) {
      *error_message = "generated points are invalid";
    }
    return false;
  }

  CurveCacheEntry entry{};
  entry.detail = std::move(detail);
  entry.points = entry.detail.sample_points;
  const SpanRuntimeState* runtime = find_span_runtime_state(span_id);
  entry.source_version = (runtime == nullptr) ? 0 : runtime->data_version;
  support_layout.source_version = entry.source_version;
  cache_span_support_layout(std::move(support_layout));
  cache_state_.curve_cache.by_span[span_id] = std::move(entry);
  return true;
}

void CoreState::cache_span_support_layout(SpanSupportLayoutEntry layout) {
  const ObjectId span_id = layout.span_id;
  cache_state_.support_layout_cache.by_span[span_id] = std::move(layout);
  rebuild_lowered_support_groups_for_span(span_id);
}

void CoreState::erase_cached_span_support_layout(ObjectId span_id) {
  ObjectId bundle_id = kInvalidObjectId;
  if (const Span* span = edit_state_.spans.find(span_id); span != nullptr) {
    bundle_id = span->bundle_id;
  }
  cache_state_.support_layout_cache.by_span.erase(span_id);
  if (bundle_id != kInvalidObjectId) {
    rebuild_lowered_support_groups_for_bundle(bundle_id);
  }
}

void CoreState::rebuild_lowered_support_groups_for_span(ObjectId span_id) {
  const Span* span = edit_state_.spans.find(span_id);
  if (span == nullptr) {
    return;
  }
  if (span->bundle_id != kInvalidObjectId) {
    rebuild_lowered_support_groups_for_bundle(span->bundle_id);
    return;
  }
  auto it = cache_state_.support_layout_cache.by_span.find(span_id);
  if (it == cache_state_.support_layout_cache.by_span.end()) {
    return;
  }
  std::vector<const SpanSupportLayoutEntry*> layouts{&it->second};
  it->second.lowered_support_groups = build_lowered_support_group_placements(edit_state_, cache_state_, layouts);
}

void CoreState::rebuild_lowered_support_groups_for_bundle(ObjectId bundle_id) {
  const std::vector<ObjectId> span_ids = GetSpansByBundle(bundle_id);
  if (span_ids.empty()) {
    return;
  }
  std::vector<SpanSupportLayoutEntry*> layouts{};
  layouts.reserve(span_ids.size());
  for (ObjectId span_id : span_ids) {
    auto it = cache_state_.support_layout_cache.by_span.find(span_id);
    if (it != cache_state_.support_layout_cache.by_span.end()) {
      layouts.push_back(&it->second);
    }
  }
  if (layouts.empty()) {
    return;
  }
  std::vector<const SpanSupportLayoutEntry*> readonly_layouts{};
  readonly_layouts.reserve(layouts.size());
  for (SpanSupportLayoutEntry* layout : layouts) {
    readonly_layouts.push_back(layout);
  }
  const std::vector<LoweredSupportGroupPlacement> groups =
      build_lowered_support_group_placements(edit_state_, cache_state_, readonly_layouts);
  for (SpanSupportLayoutEntry* layout : layouts) {
    layout->lowered_support_groups = groups;
  }
}

bool CoreState::rebuild_span_bounds(ObjectId span_id, std::string* error_message) {
  const Span* span = edit_state_.spans.find(span_id);
  if (span == nullptr) {
    if (error_message != nullptr) {
      *error_message = "span not found";
    }
    return false;
  }
  auto curve_it = cache_state_.curve_cache.by_span.find(span_id);
  if (curve_it == cache_state_.curve_cache.by_span.end()) {
    if (error_message != nullptr) {
      *error_message = "curve cache missing";
    }
    return false;
  }
  const std::vector<Vec3d>& points = curve_it->second.points;
  if (points.size() < 2) {
    if (error_message != nullptr) {
      *error_message = "curve cache has too few points";
    }
    return false;
  }

  BoundsCacheEntry bounds{};
  bounds.whole = build_aabb_from_points(points);
  bounds.segments.reserve(points.size() - 1);
  for (std::size_t i = 0; i + 1 < points.size(); ++i) {
    bounds.segments.push_back(build_aabb_from_two_points(points[i], points[i + 1]));
  }
  const SpanRuntimeState* runtime = find_span_runtime_state(span_id);
  bounds.source_version = (runtime == nullptr) ? 0 : runtime->data_version;
  cache_state_.bounds_cache.by_span[span_id] = std::move(bounds);
  return true;
}

bool CoreState::rebuild_span_visual(ObjectId span_id, std::string* error_message) {
  const Span* span = edit_state_.spans.find(span_id);
  if (span == nullptr) {
    if (error_message != nullptr) {
      *error_message = "span not found";
    }
    return false;
  }
  const Port* a = edit_state_.ports.find(span->port_a_id);
  const Port* b = edit_state_.ports.find(span->port_b_id);
  if (a == nullptr || b == nullptr) {
    if (error_message != nullptr) {
      *error_message = "span endpoint port is missing";
    }
    return false;
  }

  const Bundle* bundle = edit_state_.bundles.find(span->bundle_id);
  const BundleTemplate* bundle_template =
      (bundle == nullptr) ? nullptr : find_bundle_template(bundle->bundle_template_id);
  const CableTemplate* cable_template =
      (bundle_template == nullptr) ? nullptr : find_cable_template(bundle_template->cable_template_id);
  const bool requires_insulator = (cable_template != nullptr) ? cable_template->requires_insulator : false;
  const SpanRuntimeState* runtime = find_span_runtime_state(span_id);
  const std::uint64_t flow_key = variation_flow_key_for_span(runtime, *span);
  const SpanSupportLayoutEntry* support_layout = find_span_support_layout(span_id);

  auto append_parts_for_port = [&](const Port& port, const SupportLayoutEndpoint* layout_endpoint) {
    const EndpointContinuityDecision* endpoint_decision =
        (layout_endpoint == nullptr) ? nullptr : &layout_endpoint->decision;
    ObjectId support_pole_id = port.owner_pole_id;
    if (support_pole_id == kInvalidObjectId && layout_endpoint != nullptr) {
      support_pole_id = layout_endpoint->owner_pole_id;
    }
    const Pole* pole = edit_state_.poles.find(support_pole_id);
    if (pole == nullptr) {
      return;
    }
    const double dx = port.world_position.x - pole->world_transform.position.x;
    const double dy = port.world_position.y - pole->world_transform.position.y;
    const double planar = std::sqrt(dx * dx + dy * dy);
    Vec3d radial{
        (planar <= 1e-9) ? 1.0 : (dx / planar),
        (planar <= 1e-9) ? 0.0 : (dy / planar),
        0.0,
    };
    const BackboneLoweringKind layout_lowering_kind =
        (support_layout == nullptr) ? BackboneLoweringKind::kNone : support_layout->lowering_kind;
    const bool uses_lowered_support_visual =
      layout_endpoint != nullptr && UsesGroupedLoweredSupport(*layout_endpoint, layout_lowering_kind);
    const Port& other_port = (port.id == span->port_a_id) ? *b : *a;
    if (uses_lowered_support_visual) {
      radial = ResolveSupportAxisForEndpoint(port, *span, other_port, *pole, layout_endpoint, edit_state_);
      if (layout_endpoint != nullptr && endpoint_uses_grouped_lowered_support(layout_endpoint, layout_lowering_kind)) {
        radial = CanonicalSharedSupportAxis(radial, layout_endpoint->has_side_axis ? layout_endpoint->side_axis : radial);
      }
    }

    SpanVisualCacheEntry& entry = cache_state_.visual_cache.by_span[span_id];
    if (cache_state_.visual_settings.enable_support_structures &&
        !uses_lowered_support_visual &&
        port.template_side != SlotSide::kCenter &&
        planar > cache_state_.visual_settings.support_center_threshold_m + 1e-9) {
      VisualPart arm{};
      arm.kind = VisualPartKind::kSupportArm;
      arm.a = {pole->world_transform.position.x, pole->world_transform.position.y, port.world_position.z};
      arm.b = {
          port.world_position.x + radial.x * cache_state_.visual_settings.support_arm_extra_m,
          port.world_position.y + radial.y * cache_state_.visual_settings.support_arm_extra_m,
          port.world_position.z,
      };
      arm.radius_m = 0.03;
      entry.parts.push_back(arm);
    }

    if (cache_state_.visual_settings.enable_insulators && requires_insulator) {
      VisualPart ins{};
      ins.kind = VisualPartKind::kInsulator;
      ins.a = port.world_position;
      ins.b = {
          port.world_position.x + radial.x * cache_state_.visual_settings.insulator_length_m,
          port.world_position.y + radial.y * cache_state_.visual_settings.insulator_length_m,
          port.world_position.z,
      };
      ins.radius_m = cache_state_.visual_settings.insulator_radius_m;
      entry.parts.push_back(ins);
    }

    if (!uses_lowered_support_visual) {
      return;
    }
    const double down_offset_m = (layout_endpoint == nullptr) ? 0.0 : layout_endpoint->branch_down_offset_m;
    Vec3d tip_world{};
    if (layout_endpoint != nullptr && endpoint_uses_grouped_lowered_support(layout_endpoint, layout_lowering_kind)) {
      const auto anchors = shared_support_anchor_points(*pole, radial, port.world_position.z + down_offset_m, cache_state_);
      tip_world = anchors.second;
    } else {
      tip_world = {
          port.world_position.x,
          port.world_position.y,
          port.world_position.z + down_offset_m,
      };
    }

    if (cache_state_.visual_settings.enable_support_structures) {
      VisualPart hanger{};
      hanger.kind = VisualPartKind::kFitting;
      hanger.a = tip_world;
      hanger.b = port.world_position;
      hanger.radius_m = 0.02;
      entry.parts.push_back(hanger);
    }
  };

  SpanVisualCacheEntry entry{};
  entry.source_version = (runtime == nullptr) ? 0 : runtime->data_version;
  cache_state_.visual_cache.by_span[span_id] = std::move(entry);
  const SupportLayoutEndpoint* start_layout =
      (support_layout != nullptr && support_layout->start.port_id == a->id) ? &support_layout->start : nullptr;
  const SupportLayoutEndpoint* end_layout =
      (support_layout != nullptr && support_layout->end.port_id == b->id) ? &support_layout->end : nullptr;
  append_parts_for_port(*a, start_layout);
  append_parts_for_port(*b, end_layout);

  SpanRenderCacheEntry render{};
  render.source_version = (runtime == nullptr) ? 0 : runtime->data_version;
  if (cable_template != nullptr) {
    render.wire_radius_m = std::max(0.0005, cable_template->outer_diameter_m * 0.5);
    render.color_rgba = cable_template->color_rgba;
    render.material_style = cable_template->material_style;
  }
  const auto curve_it = cache_state_.curve_cache.by_span.find(span_id);
  if (curve_it != cache_state_.curve_cache.by_span.end()) {
    render.arc_length_m_by_point = curve_it->second.detail.distance_attributes.arc_length_m;
    render.arc_length_normalized_by_point = curve_it->second.detail.distance_attributes.arc_length_normalized;
    render.segment_length_m = curve_it->second.detail.distance_attributes.segment_length_m;
  }
  cache_state_.render_cache.by_span[span_id] = render;
  return true;
}

void apply_attachment_line_effects_to_curve(const CoreState& state, ObjectId span_id, DetailCurve* curve) {
  if (curve == nullptr || curve->Length() <= kZeroLengthEps) {
    return;
  }
  const auto attachments_it = state.view().relation_index().attachments_by_span.find(span_id);
  if (attachments_it == state.view().relation_index().attachments_by_span.end()) {
    return;
  }

  std::vector<CurveLengthInterval> hidden{};
  std::vector<CurveLengthInterval> replaced{};
  std::vector<DetailReplacementPath> replacement_paths{};

  for (ObjectId attachment_id : attachments_it->second) {
    const Attachment* attachment = state.view().attachments().find(attachment_id);
    if (attachment == nullptr) {
      continue;
    }
    const AttachmentTemplate* attachment_template = state.find_attachment_template(attachment->template_id);
    if (attachment_template == nullptr ||
        attachment_template->line_interaction_mode == AttachmentLineInteractionMode::kPassThrough) {
      continue;
    }

    const double center_s = std::clamp(curve->Length() * attachment->t, 0.0, curve->Length());
    Vec3d origin = curve->PositionAtLength(center_s);
    OffsetAlongWorldUp(&origin, attachment->display_offset_m);
    const Vec3d tangent = curve->EvaluateTangent(curve->LengthToU(center_s));
    Vec3d forward{};
    Vec3d lateral{};
    Vec3d up{};
    if (!build_attachment_frame(tangent, &forward, &lateral, &up)) {
      continue;
    }

    const AttachmentSocketTemplate* socket_a = nullptr;
    const AttachmentSocketTemplate* socket_b = nullptr;
    const AttachmentInternalPathTemplate* internal_path = nullptr;
    if (!resolve_attachment_socket_pair(*attachment_template, &socket_a, &socket_b, &internal_path) ||
        socket_a == nullptr || socket_b == nullptr) {
      continue;
    }

    const double start_s = std::clamp(center_s + std::min(socket_a->local_position.x, socket_b->local_position.x), 0.0,
                                      curve->Length());
    const double end_s = std::clamp(center_s + std::max(socket_a->local_position.x, socket_b->local_position.x), 0.0,
                                    curve->Length());
    if (end_s - start_s <= kZeroLengthEps) {
      continue;
    }

    hidden.push_back({start_s, end_s});

    if (attachment_template->line_interaction_mode != AttachmentLineInteractionMode::kReplaceWithInternalPath ||
        internal_path == nullptr) {
      continue;
    }

    DetailReplacementPath replacement{};
    replacement.attachment_id = attachment->id;
    replacement.attachment_template_id = attachment_template->id;
    replacement.interaction_mode = attachment_template->line_interaction_mode;
    replacement.replaced_interval = {start_s, end_s};
    replacement.points.push_back(attachment_local_to_world(origin, forward, lateral, up, socket_a->local_position));
    for (const Vec3d& local_point : internal_path->local_points) {
      replacement.points.push_back(attachment_local_to_world(origin, forward, lateral, up, local_point));
    }
    replacement.points.push_back(attachment_local_to_world(origin, forward, lateral, up, socket_b->local_position));
    if (replacement.points.size() >= 2) {
      replaced.push_back(replacement.replaced_interval);
      replacement_paths.push_back(std::move(replacement));
    }
  }

  curve->hidden_intervals = merged_intervals(std::move(hidden), curve->Length());
  curve->replacement_intervals = merged_intervals(std::move(replaced), curve->Length());
  curve->visible_intervals = visible_intervals_from_hidden(curve->hidden_intervals, curve->Length());
  if (curve->visible_intervals.empty() && curve->Length() > kZeroLengthEps) {
    curve->visible_intervals.push_back({0.0, curve->Length()});
  }
  curve->replacement_paths = std::move(replacement_paths);
}

SpanSupportLayoutEntry CoreState::generate_span_support_layout(const Span& span, std::string* error_message) const {
  const Port* port_a = edit_state_.ports.find(span.port_a_id);
  const Port* port_b = edit_state_.ports.find(span.port_b_id);
  if (port_a == nullptr || port_b == nullptr) {
    if (error_message != nullptr) {
      *error_message = "span endpoint port is missing";
    }
    return {};
  }

  const Vec3d a = port_a->world_position;
  const Vec3d b = port_b->world_position;
  const Pole* pole_a = edit_state_.poles.find(port_a->owner_pole_id);
  const Pole* pole_b = edit_state_.poles.find(port_b->owner_pole_id);
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
  layout.variation_flow_key = inputs.variation_flow_key;
  const SpanSupportLayoutEntry* existing_layout = find_span_support_layout(span.id);
  const SpanSupportLayoutEntry* authoritative_layout = existing_layout;
  const int resolved_socket_a = resolve_span_endpoint_socket_id(span, true);
  const int resolved_socket_b = resolve_span_endpoint_socket_id(span, false);
  const bool socket_override_a = has_span_endpoint_socket_override(span.id, true);
  const bool socket_override_b = has_span_endpoint_socket_override(span.id, false);
  HierarchicalVariationSample down_offset_variation_a{};
  HierarchicalVariationSample down_offset_variation_b{};
  const auto automatic_branch_down_offset_for_endpoint =
      [&](const Port& port, bool is_start_endpoint, HierarchicalVariationSample* variation) {
        if (authoritative_layout != nullptr) {
          const SupportLayoutEndpoint& endpoint = is_start_endpoint ? authoritative_layout->start : authoritative_layout->end;
          if (variation != nullptr) {
            *variation = endpoint.down_offset_variation;
          }
          return endpoint.automatic_branch_down_offset_m;
        }
        return branch_down_offset_for_port(*this, port, inputs.variation_flow_key, variation);
      };
  const double automatic_branch_down_offset_a =
      automatic_branch_down_offset_for_endpoint(*port_a, true, &down_offset_variation_a);
  const double automatic_branch_down_offset_b =
      automatic_branch_down_offset_for_endpoint(*port_b, false, &down_offset_variation_b);
  const double resolved_branch_down_offset_a = resolve_span_branch_down_offset_m(span, automatic_branch_down_offset_a);
  const double resolved_branch_down_offset_b = resolve_span_branch_down_offset_m(span, automatic_branch_down_offset_b);
  layout.start = build_support_layout_endpoint(
      *this, span, *port_a, pole_a, chord_dir, inputs.basis_length, endpoint_offset_m, inputs.effective_sag_ratio,
      inputs.bend_stiffness_hint, inputs.min_bend_radius_hint_m, inputs.continuity_preference, inputs.pass_mode,
      inputs.endpoint_mode, inputs.variation_flow_key, inputs.flow_kind, resolved_socket_a, socket_override_a,
      automatic_branch_down_offset_a, down_offset_variation_a, resolved_branch_down_offset_a, true);
  layout.end = build_support_layout_endpoint(
      *this, span, *port_b, pole_b, chord_dir, inputs.basis_length, endpoint_offset_m, inputs.effective_sag_ratio,
      inputs.bend_stiffness_hint, inputs.min_bend_radius_hint_m, inputs.continuity_preference, inputs.pass_mode,
      inputs.endpoint_mode, inputs.variation_flow_key, inputs.flow_kind, resolved_socket_b, socket_override_b,
      automatic_branch_down_offset_b, down_offset_variation_b, resolved_branch_down_offset_b, false);
  if (authoritative_layout != nullptr) {
    layout.flow_kind = authoritative_layout->flow_kind;
    layout.bundle_order_policy = authoritative_layout->bundle_order_policy;
    layout.relation_a = authoritative_layout->relation_a;
    layout.relation_b = authoritative_layout->relation_b;
    layout.continuity_class = authoritative_layout->continuity_class;
    layout.default_lower_required = authoritative_layout->default_lower_required;
    layout.same_level_feasible = authoritative_layout->same_level_feasible;
    layout.same_level_reason = authoritative_layout->same_level_reason;
    layout.projected_spacing_topview_m = authoritative_layout->projected_spacing_topview_m;
    layout.required_clearance_m = authoritative_layout->required_clearance_m;
    layout.lowering_blocked_by_policy = authoritative_layout->lowering_blocked_by_policy;
    layout.unresolved_same_level_conflict = authoritative_layout->unresolved_same_level_conflict;
    layout.solver_used_same_level_constraint = authoritative_layout->solver_used_same_level_constraint;
    layout.used_special_case_ports = authoritative_layout->used_special_case_ports;
    layout.lowering_kind = authoritative_layout->lowering_kind;
    apply_endpoint_decision_to_layout_endpoint(authoritative_layout->start.decision, &layout.start);
    apply_endpoint_decision_to_layout_endpoint(authoritative_layout->end.decision, &layout.end);
    layout.start.flow_kind = authoritative_layout->flow_kind;
    layout.end.flow_kind = authoritative_layout->flow_kind;
    layout.start.relation_kind = authoritative_layout->start.relation_kind;
    layout.end.relation_kind = authoritative_layout->end.relation_kind;
    layout.start.continuity_class = authoritative_layout->start.continuity_class;
    layout.end.continuity_class = authoritative_layout->end.continuity_class;
    layout.start.default_lower_required = authoritative_layout->start.default_lower_required;
    layout.end.default_lower_required = authoritative_layout->end.default_lower_required;
    layout.start.origin = authoritative_layout->start.origin;
    layout.end.origin = authoritative_layout->end.origin;
    layout.start.bundle_order_policy = authoritative_layout->start.bundle_order_policy;
    layout.end.bundle_order_policy = authoritative_layout->end.bundle_order_policy;
    layout.start.bundle_order_choice = authoritative_layout->start.bundle_order_choice;
    layout.end.bundle_order_choice = authoritative_layout->end.bundle_order_choice;
    layout.start.bundle_order_choice_reason = authoritative_layout->start.bundle_order_choice_reason;
    layout.end.bundle_order_choice_reason = authoritative_layout->end.bundle_order_choice_reason;
  }
  return layout;
}

DetailCurve CoreState::generate_span_curve(const Span& span, const SpanSupportLayoutEntry& support_layout,
                                           std::string* error_message) const {
  const Port* port_a = edit_state_.ports.find(span.port_a_id);
  const Port* port_b = edit_state_.ports.find(span.port_b_id);
  if (port_a == nullptr || port_b == nullptr) {
    if (error_message != nullptr) {
      *error_message = "span endpoint port is missing";
    }
    return {};
  }

  const int samples = std::max(2, cache_state_.geometry_settings.curve_samples);

  const Vec3d a = port_a->world_position;
  const Vec3d b = port_b->world_position;
  const Pole* pole_a = edit_state_.poles.find(port_a->owner_pole_id);
  const Pole* pole_b = edit_state_.poles.find(port_b->owner_pole_id);
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  const double dz = b.z - a.z;
  const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
  const ResolvedSpanCurveInputs inputs =
      resolve_span_curve_inputs(*this, span, *port_a, *port_b, pole_a, pole_b, a, b, distance);

  CurveConstraint start = make_curve_constraint_from_support_layout(
      support_layout.start, pole_a, inputs.basis_length, inputs.effective_sag_ratio, inputs.bend_stiffness_hint,
      inputs.min_bend_radius_hint_m, inputs.continuity_preference, support_layout.pass_mode, span.placement_context);
  CurveConstraint end = make_curve_constraint_from_support_layout(
      support_layout.end, pole_b, inputs.basis_length, inputs.effective_sag_ratio, inputs.bend_stiffness_hint,
      inputs.min_bend_radius_hint_m, inputs.continuity_preference, support_layout.pass_mode, span.placement_context);
  DetailCurve curve = BuildDetailCurve(start, end, samples);
  curve.quality.sag_variation = inputs.sag_variation;
  apply_attachment_line_effects_to_curve(*this, span.id, &curve);
  return curve;
}

AABBd CoreState::build_aabb_from_points(const std::vector<Vec3d>& points) {
  AABBd box{};
  if (points.empty()) {
    return box;
  }
  box.min = points.front();
  box.max = points.front();
  double min_height = HeightAlongWorldUp(points.front());
  double max_height = min_height;
  for (const Vec3d& p : points) {
    box.min.x = std::min(box.min.x, p.x);
    box.min.y = std::min(box.min.y, p.y);
    box.max.x = std::max(box.max.x, p.x);
    box.max.y = std::max(box.max.y, p.y);
    min_height = std::min(min_height, HeightAlongWorldUp(p));
    max_height = std::max(max_height, HeightAlongWorldUp(p));
  }
  SetHeightAlongWorldUp(&box.min, min_height);
  SetHeightAlongWorldUp(&box.max, max_height);
  return box;
}

AABBd CoreState::build_aabb_from_two_points(const Vec3d& a, const Vec3d& b) {
  AABBd box{};
  box.min = {
      std::min(a.x, b.x),
      std::min(a.y, b.y),
      std::min(a.z, b.z),
  };
  box.max = {
      std::max(a.x, b.x),
      std::max(a.y, b.y),
      0.0,
  };
  SetHeightAlongWorldUp(&box.min, std::min(HeightAlongWorldUp(a), HeightAlongWorldUp(b)));
  SetHeightAlongWorldUp(&box.max, std::max(HeightAlongWorldUp(a), HeightAlongWorldUp(b)));
  return box;
}

void CoreState::remove_span_from_caches(ObjectId span_id) {
  cache_state_.curve_cache.by_span.erase(span_id);
  cache_state_.bounds_cache.by_span.erase(span_id);
  erase_cached_span_support_layout(span_id);
  cache_state_.visual_cache.by_span.erase(span_id);
  cache_state_.render_cache.by_span.erase(span_id);
}

} // namespace wire::core
