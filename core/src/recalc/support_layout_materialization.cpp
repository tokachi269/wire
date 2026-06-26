#include "support_layout_materialization.hpp"

#include "wire/core/core_state.hpp"
#include "wire/core/core_view.hpp"
#include "wire/core/coord_utils.hpp"
#include "detail_curve_input_resolution.hpp"
#include "support_layout_projection_internal.hpp"
#include "support_layout_world_geometry_internal.hpp"
#include "../generation/support_policy.hpp"
#include "../state/internal_services.hpp"
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

int default_attachment_socket_id(const AttachmentTemplate& attachment_template) {
  if (!attachment_template.internal_paths.empty()) {
    return attachment_template.internal_paths.front().start_socket_id;
  }
  if (!attachment_template.sockets.empty()) {
    return attachment_template.sockets.front().id;
  }
  return -1;
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

struct ResolvedEndpointSocketDecision {
  int resolved_socket_id = -1;
  bool socket_override_active = false;
};

struct MaterializedEndpointSocketPair {
  ResolvedEndpointSocketDecision start{};
  ResolvedEndpointSocketDecision end{};
};

struct MaterializedBranchDownOffsetPair {
  double automatic_start_m = 0.0;
  double automatic_end_m = 0.0;
  HierarchicalVariationSample start_variation{};
  HierarchicalVariationSample end_variation{};
  double resolved_start_m = 0.0;
  double resolved_end_m = 0.0;
};

struct ResolvedSpanSupportLayoutMaterializationInputs {
  MaterializedEndpointSocketPair sockets{};
  MaterializedBranchDownOffsetPair branch_down_offsets{};
};

bool endpoint_requires_pair_authority_in_normal_path(const LayoutEndpoint& endpoint) {
  return !UsesAuthoritativeGroupedLoweredSupport(endpoint) &&
         endpoint.continuity_class == ContinuityCategoryClass::kPointLike && HasAuthoritativeSupportPair(endpoint) &&
         endpoint.relation_kind != JunctionRelationKind::kThroughMain;
}

bool validate_materialized_endpoint_normal_path(const LayoutEndpoint& endpoint, std::string* error_message) {
  auto set_error = [&](const char* message) {
    if (error_message != nullptr && error_message->empty()) {
      *error_message = message;
    }
    return false;
  };
  if (endpoint.endpoint_source == LayoutEndpointSourceKind::kFallback) {
    return set_error("support layout endpoint fallback sourcing reached materialization");
  }
  if (endpoint.origin == LayoutOriginKind::kFallback) {
    return set_error("support layout origin fallback reached materialization");
  }
  if (endpoint.port_source == PortPlacementSourceKind::kUnknown) {
    return set_error("support layout requires explicit port placement source before materialization");
  }
  if (endpoint.attachment_request.kind == EndpointAttachmentRequestKind::kNone && endpoint.resolved_socket_id.has_value()) {
    return set_error("support layout resolved a socket without an attachment request");
  }
  if (endpoint.attachment_request.kind == EndpointAttachmentRequestKind::kAttachmentSocket &&
      !endpoint.resolved_socket_id.has_value()) {
    return set_error("attachment-socket support layout request did not resolve a socket");
  }
  if (endpoint.attachment_request.requested_socket_id.has_value() && endpoint.resolved_socket_id.has_value() &&
      *endpoint.attachment_request.requested_socket_id != *endpoint.resolved_socket_id) {
    return set_error("support layout reinterpreted the chosen endpoint socket");
  }
  const bool pair_normal_authority =
      endpoint.side_assignment_rule == SideAssignmentRuleKind::kThroughPairNormal &&
      endpoint.support_orientation_rule == SupportOrientationRuleKind::kThroughPairNormal &&
      endpoint.used_junction_pair_side_assignment && endpoint.has_side_axis &&
      std::abs(endpoint.chosen_side_sign) > 1e-9;
  const bool bisector_pair_authority = endpoint.side_assignment_rule == SideAssignmentRuleKind::kBisector &&
                                       endpoint.support_orientation_rule == SupportOrientationRuleKind::kBisector &&
                                       endpoint.used_junction_pair_side_assignment && endpoint.has_side_axis;
  if (endpoint_requires_pair_authority_in_normal_path(endpoint) &&
      !(pair_normal_authority || bisector_pair_authority)) {
    return set_error("pair-authoritative endpoint fell back to endpoint-local support rules");
  }
  if (UsesAuthoritativeGroupedLoweredSupport(endpoint) &&
      endpoint.support_orientation_basis == SupportOrientationBasisKind::kRadial) {
    return set_error("grouped lowered support kept a radial orientation basis");
  }
  return true;
}

bool validate_materialized_layout_normal_path(const SpanLayoutEntry& layout, std::string* error_message) {
  return validate_materialized_endpoint_normal_path(layout.start, error_message) &&
         validate_materialized_endpoint_normal_path(layout.end, error_message);
}

bool resolve_materialized_endpoint_socket(const CoreState& state, const Span& span,
                                         SpanSupportLayoutAuthorityView authority, bool is_start_endpoint,
                                         ResolvedEndpointSocketDecision* out, std::string* error_message) {
  if (out == nullptr) {
    return false;
  }
  *out = {};

  const ObjectId attachment_id = is_start_endpoint ? span.endpoint_attachment_a_id : span.endpoint_attachment_b_id;
  out->socket_override_active =
      state_internal::OverrideResolutionService::HasSpanEndpointSocketOverride(state, span.id, is_start_endpoint);
  if (out->socket_override_active) {
    out->resolved_socket_id =
        state_internal::OverrideResolutionService::ResolveSpanEndpointSocketId(state, span, is_start_endpoint);
  } else if (authority.seed != nullptr) {
    const SupportLayoutDecisionSeedEndpoint& endpoint = is_start_endpoint ? authority.seed->start
                                                                          : authority.seed->end;
    out->resolved_socket_id = endpoint.resolved_socket_id.value_or(-1);
  }

  if (attachment_id == kInvalidObjectId) {
    return true;
  }
  if (out->resolved_socket_id < 0) {
    const Attachment* attachment = state.view().attachments().find(attachment_id);
    if (attachment != nullptr) {
      const AttachmentTemplate* attachment_template = state.find_attachment_template(attachment->template_id);
      if (attachment_template != nullptr) {
        out->resolved_socket_id = default_attachment_socket_id(*attachment_template);
      }
    }
  }
  if (out->resolved_socket_id >= 0) {
    return true;
  }
  if (error_message != nullptr && error_message->empty()) {
    *error_message = "attachment endpoint requires explicit socket authority before materialization";
  }
  return false;
}

bool resolve_materialized_endpoint_sockets(const CoreState& state, const Span& span,
                                           SpanSupportLayoutAuthorityView authority,
                                           MaterializedEndpointSocketPair* out, std::string* error_message) {
  if (out == nullptr) {
    return false;
  }
  MaterializedEndpointSocketPair sockets{};
  if (!resolve_materialized_endpoint_socket(state, span, authority, true, &sockets.start, error_message) ||
      !resolve_materialized_endpoint_socket(state, span, authority, false, &sockets.end, error_message)) {
    return false;
  }
  *out = sockets;
  return true;
}

bool endpoint_uses_grouped_lowered_support(const LayoutEndpoint* endpoint) {
  return endpoint != nullptr && UsesAuthoritativeGroupedLoweredSupport(*endpoint);
}

double template_layer_base_z_for_port_category(const CoreState& state, const Pole& pole, ConnectionCategory category) {
  return state.view().port_category_base_z_for_pole(pole, category);
}

LayoutOriginKind support_layout_origin_from_port(const Port& port) {
  switch (port.placement_source) {
  case PortPlacementSourceKind::kBranchSupport:
    return LayoutOriginKind::kBranchSupport;
  case PortPlacementSourceKind::kAerialBranch:
    return LayoutOriginKind::kAerialBranch;
  case PortPlacementSourceKind::kPlacementBandConstrained:
    return LayoutOriginKind::kPlacementConstraint;
  case PortPlacementSourceKind::kPlacementBand:
  case PortPlacementSourceKind::kGenerated:
  case PortPlacementSourceKind::kManualEdit:
  case PortPlacementSourceKind::kUnknown:
  default:
    return LayoutOriginKind::kMainSupport;
  }
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

CurveEndpointMode curve_endpoint_mode_for_attachment_style(CableAttachmentStyleHint attachment_style, const Bundle* bundle,
                                                           const BundleTemplate* bundle_template) {
  if (bundle_prefers_offset_endpoint_mode(bundle, bundle_template)) {
    return CurveEndpointMode::kOffsetEndpoint;
  }
  switch (attachment_style) {
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

CurveEndpointMode curve_endpoint_mode_for_template(const CableTemplate* cable_template, const Bundle* bundle,
                                                   const BundleTemplate* bundle_template) {
  const CableAttachmentStyleHint attachment_style =
      (cable_template == nullptr) ? CableAttachmentStyleHint::kDirectThrough : cable_template->attachment_style;
  return curve_endpoint_mode_for_attachment_style(attachment_style, bundle, bundle_template);
}

LayoutEndpoint build_support_layout_endpoint(
    const CoreState& state, const Span& span, const Port& port, const Pole* owner_pole, const Vec3d& chord_dir,
    double basis_length, double endpoint_offset_m, double effective_sag_ratio, double bend_stiffness_hint,
    double min_bend_radius_hint_m, CableContinuityPolicyHint continuity_preference, CurvePassMode pass_mode,
    CurveEndpointMode endpoint_mode, double endpoint_vertical_attachment_offset_m, BackboneFlowKind flow_kind,
    int resolved_socket_id, bool socket_override_active,
    double automatic_branch_down_offset_m, const HierarchicalVariationSample& down_offset_variation,
    double resolved_branch_down_offset_m, bool is_start_endpoint) {
  LayoutEndpoint endpoint{};
  endpoint.endpoint_node_id = is_start_endpoint ? span.endpoint_node_a_id : span.endpoint_node_b_id;
  endpoint.owner_pole_id = port.owner_pole_id;
  endpoint.port_id = port.id;
  const ObjectId attachment_id = is_start_endpoint ? span.endpoint_attachment_a_id : span.endpoint_attachment_b_id;
  endpoint.flow_kind = flow_kind;
  endpoint.origin = support_layout_origin_from_port(port);
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
  if (endpoint_vertical_attachment_offset_m > 1e-9) {
    OffsetAlongWorldUp(&constraint.point, endpoint_vertical_attachment_offset_m);
  }
  endpoint.attachment_request =
      make_endpoint_attachment_request(attachment_id, resolved_socket_id, socket_override_active);
  if (resolved_socket_id >= 0) {
    endpoint.resolved_socket_id = resolved_socket_id;
  }
  const bool applied_attachment_socket =
      endpoint.attachment_request.attachment_id.has_value() && endpoint.resolved_socket_id.has_value() &&
      apply_endpoint_attachment_socket(state, *endpoint.attachment_request.attachment_id, *endpoint.resolved_socket_id,
                                       &constraint);
  if (applied_attachment_socket) {
    endpoint.endpoint_source = socket_override_active
                                   ? LayoutEndpointSourceKind::kAttachmentSocketOverride
                                   : LayoutEndpointSourceKind::kAttachmentSocket;
  } else {
    endpoint.endpoint_source = LayoutEndpointSourceKind::kPlainSupport;
  }

  endpoint.endpoint_mode = constraint.endpoint_mode;
  endpoint.endpoint_world = constraint.point;
  endpoint.support_world = constraint.point;
  endpoint.departure_dir = constraint.tangent_dir;
  endpoint.endpoint_offset = constraint.endpoint_offset;
  endpoint.local_departure_length_m = constraint.support_departure_length_m;
  return endpoint;
}

namespace {

void append_unique_group_key(std::vector<LoweredSupportGroupKey>* keys, const LoweredSupportGroupKey& key) {
  if (keys == nullptr || key.owner_pole_id == kInvalidObjectId || key.support_group_id < 0) {
    return;
  }
  if (std::find(keys->begin(), keys->end(), key) == keys->end()) {
    keys->push_back(key);
  }
}

std::unordered_map<LoweredSupportGroupKey, SupportGroupDecision, LoweredSupportGroupKeyHash>
build_support_group_decision_view_from_seeds(const SupportLayoutCache& span_layout_cache) {
  std::unordered_map<LoweredSupportGroupKey, SupportGroupDecision, LoweredSupportGroupKeyHash> groups{};
  span_layout_cache.for_each_authority_seed([&](ObjectId, SpanSupportLayoutAuthorityView, const SpanSupportLayoutDecisionSeed& seed) {
    for (const auto& [key, group_copy] : seed.support_group_decisions) {
      if (key.owner_pole_id == kInvalidObjectId || key.support_group_id < 0) {
        continue;
      }
      groups[key] = group_copy;
    }
  });
  return groups;
}

std::unordered_map<LoweredSupportGroupKey, SupportGroupDecision, LoweredSupportGroupKeyHash>
build_support_group_decision_view_for_keys_from_seeds(const SupportLayoutCache& span_layout_cache,
                                                      const std::vector<LoweredSupportGroupKey>& keys) {
  std::unordered_set<LoweredSupportGroupKey, LoweredSupportGroupKeyHash> key_set(keys.begin(), keys.end());
  std::unordered_map<LoweredSupportGroupKey, SupportGroupDecision, LoweredSupportGroupKeyHash> groups{};
  span_layout_cache.for_each_authority_seed([&](ObjectId, SpanSupportLayoutAuthorityView, const SpanSupportLayoutDecisionSeed& seed) {
    for (const auto& [key, group_copy] : seed.support_group_decisions) {
      if (key_set.find(key) == key_set.end()) {
        continue;
      }
      if (key.owner_pole_id == kInvalidObjectId || key.support_group_id < 0) {
        continue;
      }
      groups[key] = group_copy;
    }
  });
  return groups;
}

} // namespace

std::vector<LoweredSupportGroupKey> collect_support_group_keys_for_seed(const SpanSupportLayoutDecisionSeed& seed) {
  std::vector<LoweredSupportGroupKey> keys{};
  keys.reserve(seed.support_group_decisions.size());
  for (const auto& [key, _] : seed.support_group_decisions) {
    append_unique_group_key(&keys, key);
  }
  return keys;
}

struct LoweredSupportGroupObservation {
  std::optional<ConnectionCategory> category{};
  std::vector<Vec3d> attachment_worlds{};
  std::optional<double> down_offset_m{};
  std::optional<HierarchicalVariationSample> down_offset_variation{};
};

using LoweredSupportGroupObservationMap =
    std::unordered_map<LoweredSupportGroupKey, LoweredSupportGroupObservation, LoweredSupportGroupKeyHash>;

void clear_lowered_support_group_derivatives(
    SupportLayoutCache* cache, const std::vector<LoweredSupportGroupKey>& keys) {
  if (cache == nullptr) {
    return;
  }
  for (const LoweredSupportGroupKey& key : keys) {
    cache->support_groups.authority.by_key.erase(key);
    cache->support_groups.placement.by_key.erase(key);
  }
}

void store_lowered_support_group_authority(
    SupportLayoutCache* cache,
    const std::unordered_map<LoweredSupportGroupKey, SupportGroupDecision, LoweredSupportGroupKeyHash>& authority_by_key) {
  if (cache == nullptr) {
    return;
  }
  for (const auto& [key, authority] : authority_by_key) {
    cache->support_groups.authority.by_key[key] = authority;
  }
}

LoweredSupportGroupObservationMap collect_lowered_support_group_observations(
    const EditState& edit_state, SupportLayoutCache* cache, const std::vector<LoweredSupportGroupKey>& keys) {
  LoweredSupportGroupObservationMap observations{};
  if (cache == nullptr) {
    return observations;
  }

  const std::unordered_set<LoweredSupportGroupKey, LoweredSupportGroupKeyHash> key_set(keys.begin(), keys.end());
  cache->for_each_projected_record([&](ObjectId, SupportLayoutCacheRecord&, SpanLayoutEntry& layout) {
    layout.lowered_support_group_keys.erase(
        std::remove_if(layout.lowered_support_group_keys.begin(), layout.lowered_support_group_keys.end(),
                       [&](const LoweredSupportGroupKey& key) { return key_set.contains(key); }),
        layout.lowered_support_group_keys.end());

    const auto observe_endpoint = [&](const LayoutEndpoint& endpoint) {
      if (!endpoint_uses_grouped_lowered_support(&endpoint)) {
        return;
      }
      const LoweredSupportGroupKey key = LoweredSupportGroupKeyFromDecision(endpoint);
      if (!key_set.contains(key)) {
        return;
      }

      LoweredSupportGroupObservation& observation = observations[key];
      if (!observation.category.has_value()) {
        if (const Port* port = edit_state.ports.find(endpoint.port_id); port != nullptr) {
          observation.category = port->category;
        }
      }
      observation.attachment_worlds.push_back(endpoint.endpoint_world);
      if (!observation.down_offset_m.has_value()) {
        observation.down_offset_m = endpoint.branch_down_offset_m;
      }
      if (!observation.down_offset_variation.has_value()) {
        observation.down_offset_variation = endpoint.down_offset_variation;
      }
    };

    observe_endpoint(layout.start);
    observe_endpoint(layout.end);
  });

  return observations;
}

void materialize_lowered_support_group_placements(
    const CoreState& state, const EditState& edit_state, CacheState* cache_state,
    const std::unordered_map<LoweredSupportGroupKey, SupportGroupDecision, LoweredSupportGroupKeyHash>& authority_by_key,
    const LoweredSupportGroupObservationMap& observations) {
  if (cache_state == nullptr) {
    return;
  }

  for (const auto& [key, authority] : authority_by_key) {
    const auto it_observation = observations.find(key);
    const LoweredSupportGroupObservation* observation =
        (it_observation == observations.end()) ? nullptr : &it_observation->second;
    LoweredSupportGroupPlacement placement = build_grouped_support_placement_from_decision(
        state, authority, edit_state, *cache_state,
        (observation != nullptr && observation->category.has_value()) ? &*observation->category : nullptr,
        (observation != nullptr) ? &observation->attachment_worlds : nullptr);
    if (observation != nullptr && observation->down_offset_m.has_value()) {
      placement.down_offset_m = *observation->down_offset_m;
    }
    if (observation != nullptr && observation->down_offset_variation.has_value()) {
      placement.down_offset_variation = *observation->down_offset_variation;
    }
    cache_state->span_layout_cache.support_groups.placement.by_key[key] = std::move(placement);
  }
}

void project_lowered_support_group_placements(
    SupportLayoutCache* cache,
    const std::unordered_map<LoweredSupportGroupKey, SupportGroupDecision, LoweredSupportGroupKeyHash>& authority_by_key,
    const std::vector<LoweredSupportGroupKey>& keys) {
  if (cache == nullptr) {
    return;
  }

  const std::unordered_set<LoweredSupportGroupKey, LoweredSupportGroupKeyHash> key_set(keys.begin(), keys.end());
  cache->for_each_projected_record([&](ObjectId, SupportLayoutCacheRecord&, SpanLayoutEntry& layout) {
    const auto reproject_endpoint = [&](LayoutEndpoint* endpoint) {
      if (endpoint == nullptr || !endpoint_uses_grouped_lowered_support(endpoint)) {
        return;
      }
      const LoweredSupportGroupKey key = LoweredSupportGroupKeyFromDecision(*endpoint);
      if (!key_set.contains(key)) {
        return;
      }
      const auto authority_it = authority_by_key.find(key);
      const auto placement_it = cache->support_groups.placement.by_key.find(key);
      if (authority_it == authority_by_key.end() || placement_it == cache->support_groups.placement.by_key.end()) {
        return;
      }
      append_unique_group_key(&layout.lowered_support_group_keys, key);
      apply_grouped_support_placement_to_layout_endpoint(authority_it->second, placement_it->second, endpoint);
    };
    reproject_endpoint(&layout.start);
    reproject_endpoint(&layout.end);
  });
}

void rebuild_lowered_support_groups_for_keys(const CoreState& state, const EditState& edit_state, CacheState* cache_state,
                                             const std::vector<LoweredSupportGroupKey>& keys) {
  if (cache_state == nullptr) {
    return;
  }
  std::vector<LoweredSupportGroupKey> filtered_keys{};
  filtered_keys.reserve(keys.size());
  for (const LoweredSupportGroupKey& key : keys) {
    append_unique_group_key(&filtered_keys, key);
  }
  if (filtered_keys.empty()) {
    return;
  }

  const auto authority_by_key =
      build_support_group_decision_view_for_keys_from_seeds(cache_state->span_layout_cache, filtered_keys);
  clear_lowered_support_group_derivatives(&cache_state->span_layout_cache, filtered_keys);
  store_lowered_support_group_authority(&cache_state->span_layout_cache, authority_by_key);

  const LoweredSupportGroupObservationMap observations =
      collect_lowered_support_group_observations(edit_state, &cache_state->span_layout_cache, filtered_keys);
  materialize_lowered_support_group_placements(state, edit_state, cache_state, authority_by_key, observations);
  project_lowered_support_group_placements(&cache_state->span_layout_cache, authority_by_key, filtered_keys);
}

namespace {

double automatic_branch_down_offset_from_authority(const CoreState& state, const Port& port,
                                                   SpanSupportLayoutAuthorityView authority, bool is_start_endpoint,
                                                   HierarchicalVariationSample* variation) {
  (void)state;
  (void)port;
  if (authority.seed != nullptr) {
    const SupportLayoutDecisionSeedEndpoint& endpoint =
        is_start_endpoint ? authority.seed->start : authority.seed->end;
    if (variation != nullptr) {
      *variation = endpoint.down_offset_variation;
    }
    return endpoint.automatic_branch_down_offset_m;
  }
  if (variation != nullptr) {
    *variation = {};
  }
  return 0.0;
}

MaterializedBranchDownOffsetPair resolve_materialized_branch_down_offsets(const CoreState& state, const Span& span,
                                                                          const Port& port_a, const Port& port_b,
                                                                          SpanSupportLayoutAuthorityView authority) {
  MaterializedBranchDownOffsetPair offsets{};
  offsets.automatic_start_m =
      automatic_branch_down_offset_from_authority(state, port_a, authority, true, &offsets.start_variation);
  offsets.automatic_end_m =
      automatic_branch_down_offset_from_authority(state, port_b, authority, false, &offsets.end_variation);
  offsets.resolved_start_m =
      state_internal::OverrideResolutionService::ResolveSpanBranchDownOffsetM(state, span, offsets.automatic_start_m);
  offsets.resolved_end_m =
      state_internal::OverrideResolutionService::ResolveSpanBranchDownOffsetM(state, span, offsets.automatic_end_m);
  return offsets;
}

ResolvedSpanSupportLayoutMaterializationInputs resolve_span_support_layout_materialization_inputs(
    const CoreState& state, const Span& span, const Port& port_a, const Port& port_b,
    SpanSupportLayoutAuthorityView authority, std::string* error_message) {
  ResolvedSpanSupportLayoutMaterializationInputs inputs{};
  if (!resolve_materialized_endpoint_sockets(state, span, authority, &inputs.sockets, error_message)) {
    return {};
  }
  inputs.branch_down_offsets = resolve_materialized_branch_down_offsets(state, span, port_a, port_b, authority);
  return inputs;
}

void apply_consumed_support_layout_authority(SpanSupportLayoutAuthorityView authority, SpanLayoutEntry* layout) {
  if (layout == nullptr) {
    return;
  }
  if (authority.seed != nullptr) {
    apply_support_layout_decision_seed(*authority.seed, layout);
  }
}

void project_materialized_layout_authority(SpanSupportLayoutAuthorityView authority, SpanLayoutEntry* layout) {
  apply_consumed_support_layout_authority(authority, layout);
}

void materialize_layout_world_geometry(const CoreState& state, const Port& port_a, const Port& port_b, const Pole* pole_a,
                                       const Pole* pole_b, const Vec3d& chord_dir, SpanLayoutEntry* layout) {
  if (layout == nullptr) {
    return;
  }
  apply_materialized_visual_arm_geometry(state, port_a, pole_a, &layout->start);
  apply_materialized_visual_arm_geometry(state, port_b, pole_b, &layout->end);
  finalize_support_layout_materialization(chord_dir, layout);
}

SpanLayoutEntry materialize_span_support_layout(const CoreState& state, const Span& span,
                                                       const ResolvedSpanCurveInputs& inputs, const Port& port_a,
                                                       const Port& port_b, const Pole* pole_a, const Pole* pole_b,
                                                       const Vec3d& chord_dir,
                                                       SpanSupportLayoutAuthorityView authority,
                                                       const MaterializedEndpointSocketPair& sockets,
                                                       const MaterializedBranchDownOffsetPair& branch_down_offsets) {
  const double endpoint_offset_m = std::min(std::max(0.02, inputs.basis_length * 0.03), 0.35);

  SpanLayoutEntry layout{};
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

  layout.start = build_support_layout_endpoint(
      state, span, port_a, pole_a, chord_dir, inputs.basis_length, endpoint_offset_m, inputs.effective_sag_ratio,
      inputs.bend_stiffness_hint, inputs.min_bend_radius_hint_m, inputs.continuity_preference, inputs.pass_mode,
      inputs.endpoint_mode, inputs.endpoint_vertical_attachment_offset_m, inputs.flow_kind, sockets.start.resolved_socket_id,
      sockets.start.socket_override_active, branch_down_offsets.automatic_start_m, branch_down_offsets.start_variation,
      branch_down_offsets.resolved_start_m, true);
  layout.end = build_support_layout_endpoint(
      state, span, port_b, pole_b, chord_dir, inputs.basis_length, endpoint_offset_m, inputs.effective_sag_ratio,
      inputs.bend_stiffness_hint, inputs.min_bend_radius_hint_m, inputs.continuity_preference, inputs.pass_mode,
      inputs.endpoint_mode, inputs.endpoint_vertical_attachment_offset_m, inputs.flow_kind, sockets.end.resolved_socket_id,
      sockets.end.socket_override_active, branch_down_offsets.automatic_end_m, branch_down_offsets.end_variation,
      branch_down_offsets.resolved_end_m, false);
  project_materialized_layout_authority(authority, &layout);
  materialize_layout_world_geometry(state, port_a, port_b, pole_a, pole_b, chord_dir, &layout);
  return layout;
}

} // namespace

SpanLayoutEntry CoreState::generate_span_support_layout(const Span& span, std::string* error_message) const {
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
  if (port_a->placement_source == PortPlacementSourceKind::kUnknown ||
      port_b->placement_source == PortPlacementSourceKind::kUnknown) {
    if (error_message != nullptr && error_message->empty()) {
      *error_message = "support layout requires explicit port placement source before materialization";
    }
    return {};
  }

  Vec3d chord_dir = b - a;
  if (!Normalize(&chord_dir)) {
    chord_dir = WorldForward();
  }
  const SpanSupportLayoutAuthorityView authority = runtime_.cache_state.span_layout_cache.authority_view(span.id);
  if (authority.required && !authority.has_authority()) {
    if (error_message != nullptr && error_message->empty()) {
      *error_message = "support layout decision seed is missing";
    }
    return {};
  }

  const ResolvedSpanSupportLayoutMaterializationInputs resolved_inputs =
      resolve_span_support_layout_materialization_inputs(*this, span, *port_a, *port_b, authority, error_message);
  if ((span.endpoint_attachment_a_id != kInvalidObjectId && resolved_inputs.sockets.start.resolved_socket_id < 0) ||
      (span.endpoint_attachment_b_id != kInvalidObjectId && resolved_inputs.sockets.end.resolved_socket_id < 0)) {
    return {};
  }
  SpanLayoutEntry layout =
      materialize_span_support_layout(*this, span, inputs, *port_a, *port_b, pole_a, pole_b, chord_dir, authority,
                                      resolved_inputs.sockets, resolved_inputs.branch_down_offsets);
  if (!validate_materialized_layout_normal_path(layout, error_message)) {
    return {};
  }
  return layout;
}

} // namespace wire::core
