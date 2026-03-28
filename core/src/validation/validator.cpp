#include "wire/core/core_state.hpp"
#include "wire/core/core_view.hpp"
#include "wire/core/coord_utils.hpp"
#include "../generation/support_policy.hpp"
#include "../support_orientation_utils.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace wire::core {

namespace {

template <typename TKey>
std::unordered_map<TKey, std::vector<ObjectId>>
canonical_index_map(const std::unordered_map<TKey, std::vector<ObjectId>>& map) {
  auto out = map;
  for (auto& [_, ids] : out) {
    std::sort(ids.begin(), ids.end());
    ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
  }
  return out;
}

bool has_duplicate_ids(const std::vector<ObjectId>& ids) {
  std::unordered_set<ObjectId> seen{};
  seen.reserve(ids.size());
  for (ObjectId id : ids) {
    if (!seen.insert(id).second) {
      return true;
    }
  }
  return false;
}

bool endpoint_uses_grouped_lowered_support_for_validation(const SupportLayoutEndpoint& endpoint) {
  return UsesAuthoritativeGroupedLoweredSupport(endpoint.decision);
}

bool almost_equal_validation(double a, double b, double eps = 1e-9) { return std::abs(a - b) <= eps; }

bool almost_equal_validation(const Vec3d& a, const Vec3d& b, double eps = 1e-9) {
  return almost_equal_validation(a.x, b.x, eps) && almost_equal_validation(a.y, b.y, eps) &&
         almost_equal_validation(a.z, b.z, eps);
}

double template_layer_base_z_for_validation(const CoreView& core, const Pole& pole, ConnectionCategory category) {
  double best_z = -std::numeric_limits<double>::infinity();
  const int target_layer = generation::detail::TemplateLayerForCategory(category);
  const auto pole_type_it = core.pole_types().find(pole.pole_type_id);
  if (pole_type_it != core.pole_types().end()) {
    for (const PortPlacementBand& band : pole_type_it->second.port_bands) {
      if (!band.enabled) {
        continue;
      }
      if (band.layer == target_layer) {
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

} // namespace

bool ValidationResult::has_errors() const {
  for (const ValidationIssue& issue : issues) {
    if (issue.severity == ValidationSeverity::kError) {
      return true;
    }
  }
  return false;
}

ValidationResult CoreState::ValidateFast() const {
  ValidationResult result;
  const CoreView core = view();
  const EditState& edit_state = core.edit_state();

  for (const Port& port : edit_state.ports.items()) {
    if (port.owner_pole_id != kInvalidObjectId && edit_state.poles.find(port.owner_pole_id) == nullptr) {
      result.issues.push_back({ValidationSeverity::kError, "PortOwnerMissing", "Port owner pole is missing", port.id});
    }
  }

  for (const Span& span : edit_state.spans.items()) {
    if (edit_state.ports.find(span.port_a_id) == nullptr || edit_state.ports.find(span.port_b_id) == nullptr) {
      result.issues.push_back({ValidationSeverity::kError, "SpanPortMissing", "Span references missing port", span.id});
    }
    if (span.bundle_id != kInvalidObjectId && edit_state.bundles.find(span.bundle_id) == nullptr) {
      result.issues.push_back({ValidationSeverity::kError, "SpanBundleMissing", "Span bundle is missing", span.id});
    }
  }

  for (const Bundle& bundle : edit_state.bundles.items()) {
    if (!core.bundle_templates().contains(bundle.bundle_template_id)) {
      result.issues.push_back(
          {ValidationSeverity::kError, "BundleTemplateMissing", "Bundle references unknown BundleTemplate", bundle.id});
    }
  }

  for (const auto& [pole_id, override_value] : authoritative_.override_state.pole_orientation_by_pole) {
    if (edit_state.poles.find(pole_id) == nullptr) {
      result.issues.push_back(
          {ValidationSeverity::kError, "PoleOverrideTargetMissing", "Pole orientation override target is missing", pole_id});
    } else if (override_value.manual_yaw_deg.has_value() && !std::isfinite(*override_value.manual_yaw_deg)) {
      result.issues.push_back(
          {ValidationSeverity::kError, "PoleOverrideInvalid", "Pole orientation override has non-finite yaw", pole_id});
    }
  }

  for (const auto& [span_id, override_value] : authoritative_.override_state.span_endpoint_by_span) {
    if (edit_state.spans.find(span_id) == nullptr) {
      result.issues.push_back({ValidationSeverity::kError, "SpanEndpointOverrideTargetMissing",
                               "Span endpoint override target is missing", span_id});
    } else if ((override_value.socket_a_id.has_value() && *override_value.socket_a_id < -1) ||
               (override_value.socket_b_id.has_value() && *override_value.socket_b_id < -1)) {
      result.issues.push_back({ValidationSeverity::kError, "SpanEndpointOverrideInvalid",
                               "Span endpoint socket override must be -1 or a valid socket id", span_id});
    }
  }

  for (const auto& [span_id, override_value] : authoritative_.override_state.span_support_by_span) {
    if (edit_state.spans.find(span_id) == nullptr) {
      result.issues.push_back({ValidationSeverity::kError, "SpanSupportOverrideTargetMissing",
                               "Span support override target is missing", span_id});
    } else if (override_value.branch_down_offset_m.has_value() &&
               (!std::isfinite(*override_value.branch_down_offset_m) || *override_value.branch_down_offset_m < 0.0)) {
      result.issues.push_back({ValidationSeverity::kError, "SpanSupportOverrideInvalid",
                               "Span support override branch down offset must be finite and >= 0", span_id});
    }
  }

  return result;
}

ValidationResult CoreState::Validate() const {
  ValidationResult result;
  const CoreView core = view();
  const EditState& edit_state = core.edit_state();
  const LayoutSettings& layout_settings = core.layout_settings();
  const CacheState& cache_state = core.cache_state();
  const ConnectionIndex& connection_index = core.connection_index();
  const RelationIndex& relation_index = core.relation_index();
  const auto& span_runtime_states = core.span_runtime_states();
  const auto& pole_types = core.pole_types();
  const auto& cable_templates = core.cable_templates();
  const auto& bundle_templates = core.bundle_templates();
  const auto& attachment_templates = core.attachment_templates();
  const auto& port_resolution_debug_records = core.port_resolution_debug_records();

  for (const Pole& pole : edit_state.poles.items()) {
    if (pole.pole_type_id != kInvalidPoleTypeId && !pole_types.contains(pole.pole_type_id)) {
      result.issues.push_back(
          {ValidationSeverity::kError, "PoleTypeMissing", "Pole references unknown PoleType", pole.id});
    }
    if (!std::isfinite(pole.context.corner_angle_deg) || !std::isfinite(pole.context.corner_turn_sign) ||
        !std::isfinite(pole.context.side_scale) || !std::isfinite(pole.context.sharp_theta_deg) ||
        !std::isfinite(pole.context.sharp_bisector_dir.x) || !std::isfinite(pole.context.sharp_bisector_dir.y) ||
        !std::isfinite(pole.context.sharp_bisector_dir.z) || !std::isfinite(pole.context.sharp_side_dir.x) ||
        !std::isfinite(pole.context.sharp_side_dir.y) || !std::isfinite(pole.context.sharp_side_dir.z)) {
      result.issues.push_back(
          {ValidationSeverity::kError, "PoleContextInvalid", "Pole context has non-finite value", pole.id});
    }
    if (pole.context.corner_turn_sign < -1.0 - 1e-9 || pole.context.corner_turn_sign > 1.0 + 1e-9) {
      result.issues.push_back({
          ValidationSeverity::kWarning,
          "PoleTurnSignOutOfRange",
          "Pole corner_turn_sign is out of range",
          pole.id,
      });
    }
    if (pole.context.side_scale < layout_settings.min_side_scale - 1e-9 ||
        pole.context.side_scale > layout_settings.max_side_scale + 1e-9) {
      result.issues.push_back({ValidationSeverity::kWarning, "PoleSideScaleOutOfRange",
                               "Pole side_scale is out of configured range", pole.id});
    }
    if (pole.placement_mode == PlacementMode::kManual && !pole.user_edited) {
      result.issues.push_back({
          ValidationSeverity::kWarning,
          "PoleManualWithoutUserEdited",
          "Manual pole should have user_edited=true",
          pole.id,
      });
    }
  }

  for (const auto& [pole_id, override_value] : authoritative_.override_state.pole_orientation_by_pole) {
    if (edit_state.poles.find(pole_id) == nullptr) {
      result.issues.push_back(
          {ValidationSeverity::kError, "PoleOverrideTargetMissing", "Pole orientation override target is missing", pole_id});
      continue;
    }
    if (override_value.manual_yaw_deg.has_value() && !std::isfinite(*override_value.manual_yaw_deg)) {
      result.issues.push_back(
          {ValidationSeverity::kError, "PoleOverrideInvalid", "Pole orientation override has non-finite yaw", pole_id});
    }
  }

  for (const Port& port : edit_state.ports.items()) {
    if (port.owner_pole_id != kInvalidObjectId && edit_state.poles.find(port.owner_pole_id) == nullptr) {
      result.issues.push_back({ValidationSeverity::kError, "PortOwnerMissing", "Port owner pole is missing", port.id});
    }
    if (!is_valid_slot_side(port.template_side) || !is_valid_slot_role(port.template_role)) {
      result.issues.push_back({
          ValidationSeverity::kError,
          "PortTemplateAttributeInvalid",
          "Port template side/role contains invalid value",
          port.id,
      });
    }
    if (port.generated_from_template && port.owner_pole_id != kInvalidObjectId) {
      const Pole* owner_pole = edit_state.poles.find(port.owner_pole_id);
      if (owner_pole != nullptr) {
        const auto pole_type_it = pole_types.find(owner_pole->pole_type_id);
        if (pole_type_it != pole_types.end()) {
          bool matched_hint = false;
          const PoleFrame frame =
              BuildPoleFrame(owner_pole->world_transform, effective_pole_layout_yaw_deg(*owner_pole));
          const Vec3d local = WorldPointToLocal(frame, port.world_position);
          for (const PortPlacementBand& band : pole_type_it->second.port_bands) {
            if (!band.enabled) {
              continue;
            }
            const bool same_template_band = band.category == port.category && band.layer == port.template_layer &&
                                            band.side == port.template_side && band.role == port.template_role;
            const bool inside_band_range = local.y >= band.lateral_min_m - 1e-6 && local.y <= band.lateral_max_m + 1e-6 &&
                                           local.z >= band.height_min_m - 1e-6 && local.z <= band.height_max_m + 1e-6;
            const bool constrained_overflow_match =
                band.overflow_policy == BandOverflowPolicy::kConstrainedFallback &&
                local.y >= band.lateral_min_m - 1e-6 && local.y <= band.lateral_max_m + 1e-6 &&
                local.z >= band.height_max_m - 1e-6;
            if (same_template_band && (inside_band_range || constrained_overflow_match)) {
              matched_hint = true;
              break;
            }
          }
          if (!matched_hint) {
            result.issues.push_back({
                ValidationSeverity::kWarning,
                "PortTemplateHintMissing",
                "Template-owned port does not match any enabled placement hint on owner pole type",
                port.id,
            });
          }
        }
      }
    }
    if (!std::isfinite(port.world_position.x) || !std::isfinite(port.world_position.y) ||
        !std::isfinite(port.world_position.z) || !std::isfinite(port.side_scale_applied)) {
      result.issues.push_back(
          {ValidationSeverity::kError, "PortTransformInvalid", "Port position or side_scale is non-finite", port.id});
    }
    if (port.side_scale_applied < layout_settings.min_side_scale - 1e-9 ||
        port.side_scale_applied > layout_settings.max_side_scale + 1e-9) {
      result.issues.push_back({ValidationSeverity::kWarning, "PortSideScaleOutOfRange",
                               "Port side_scale_applied is out of range", port.id});
    }
    if (port.position_mode == PortPositionMode::kManual) {
      if (!port.user_edited_position) {
        result.issues.push_back({
            ValidationSeverity::kWarning,
            "PortManualWithoutUserEdited",
            "Manual port should have user_edited_position=true",
            port.id,
        });
      }
      if (port.placement_source != PortPlacementSourceKind::kManualEdit) {
        result.issues.push_back({
            ValidationSeverity::kWarning,
            "PortManualSourceMismatch",
            "Manual port should have placement_source=ManualEdit",
            port.id,
        });
      }
    } else {
      if (port.user_edited_position) {
        result.issues.push_back({
            ValidationSeverity::kWarning,
            "PortAutoWithUserEdited",
            "Auto port should not keep user_edited_position=true",
            port.id,
        });
      }
    }
  }

  for (const Anchor& anchor : edit_state.anchors.items()) {
    if (anchor.owner_pole_id != kInvalidObjectId && edit_state.poles.find(anchor.owner_pole_id) == nullptr) {
      result.issues.push_back(
          {ValidationSeverity::kError, "AnchorOwnerMissing", "Anchor owner pole is missing", anchor.id});
    }
  }

  for (const auto& [bundle_template_id, bundle_template] : bundle_templates) {
    (void)bundle_template_id;
    if (cable_templates.find(bundle_template.cable_template_id) == cable_templates.end()) {
      result.issues.push_back({ValidationSeverity::kError, "BundleTemplateCableMissing",
                               "BundleTemplate references unknown CableTemplate", kInvalidObjectId});
    }
    if (!std::isfinite(bundle_template.grouped_support_fanout_spacing_m) ||
        bundle_template.grouped_support_fanout_spacing_m < 0.0) {
      result.issues.emplace_back(ValidationIssue{ValidationSeverity::kError, "BundleTemplateGroupedSupportFanoutInvalid",
                                                 "BundleTemplate grouped support fanout spacing must be finite and >= 0",
                                                 kInvalidObjectId});
    }
  }

  for (const auto& [cable_template_id, cable_template] : cable_templates) {
    (void)cable_template_id;
    if (!std::isfinite(cable_template.default_grouped_support_fanout_spacing_m) ||
        cable_template.default_grouped_support_fanout_spacing_m < 0.0) {
      result.issues.emplace_back(ValidationIssue{ValidationSeverity::kError, "CableTemplateGroupedSupportFanoutInvalid",
                                                 "CableTemplate grouped support default fanout spacing must be finite and >= 0",
                                                 kInvalidObjectId});
    }
    if (cable_template.default_endpoint_attachment_template_id != kInvalidAttachmentTemplateId &&
        attachment_templates.find(cable_template.default_endpoint_attachment_template_id) == attachment_templates.end()) {
      result.issues.emplace_back(
          ValidationIssue{ValidationSeverity::kError, "CableTemplateAttachmentTemplateMissing",
                          "CableTemplate default endpoint attachment template must exist", kInvalidObjectId});
    }
  }

  for (const Bundle& bundle : edit_state.bundles.items()) {
    if (bundle_templates.find(bundle.bundle_template_id) == bundle_templates.end()) {
      result.issues.push_back(
          {ValidationSeverity::kError, "BundleTemplateMissing", "Bundle references unknown BundleTemplate", bundle.id});
    }
  }

  for (const auto& [attachment_template_id, attachment_template] : attachment_templates) {
    (void)attachment_template_id;
    std::unordered_set<int> socket_ids{};
    for (const AttachmentSocketTemplate& socket : attachment_template.sockets) {
      if (!socket_ids.insert(socket.id).second) {
        result.issues.push_back({ValidationSeverity::kError, "AttachmentTemplateDuplicateSocket",
                                 "AttachmentTemplate contains duplicate socket ids", kInvalidObjectId});
      }
    }
    for (const AttachmentInternalPathTemplate& path : attachment_template.internal_paths) {
      if (!socket_ids.contains(path.start_socket_id) || !socket_ids.contains(path.end_socket_id)) {
        result.issues.push_back({ValidationSeverity::kError, "AttachmentTemplatePathSocketMissing",
                                 "AttachmentTemplate internal path references missing socket", kInvalidObjectId});
      }
      if (attachment_template.line_interaction_mode != AttachmentLineInteractionMode::kReplaceWithInternalPath) {
        result.issues.push_back({ValidationSeverity::kError, "AttachmentTemplatePathModeMismatch",
                                 "AttachmentTemplate internal paths require ReplaceWithInternalPath interaction mode",
                                 kInvalidObjectId});
      }
      if (path.profile_kind != AttachmentInternalPathTemplate::ProfileKind::kExplicitPolyline &&
          !path.local_points.empty()) {
        result.issues.push_back({ValidationSeverity::kError, "AttachmentTemplateGeneratedPathHasExplicitPoints",
                                 "Generated attachment path profiles must not carry explicit local points",
                                 kInvalidObjectId});
      }
      if (path.profile_kind == AttachmentInternalPathTemplate::ProfileKind::kCoiledCable) {
        if (!std::isfinite(path.coil_radius_m) || path.coil_radius_m <= 1e-6 || path.coil_turn_count < 1 ||
            path.coil_samples_per_turn < 4) {
          result.issues.push_back({ValidationSeverity::kError, "AttachmentTemplateCoilProfileInvalid",
                                   "Coiled attachment path profile requires finite positive radius, turn count >= 1, and samples per turn >= 4",
                                   kInvalidObjectId});
        }
      } else if (path.coil_turn_count != 0 || std::abs(path.coil_radius_m) > 1e-12) {
        result.issues.push_back({ValidationSeverity::kError, "AttachmentTemplateCoilParamsUnused",
                                 "Only CoiledCable attachment path profiles may set coil parameters",
                                 kInvalidObjectId});
      }
    }
  }

  for (const Attachment& attachment : edit_state.attachments.items()) {
    if (edit_state.spans.find(attachment.span_id) == nullptr) {
      result.issues.push_back(
          {ValidationSeverity::kError, "AttachmentSpanMissing", "Attachment references missing span", attachment.id});
    }
    if (attachment_templates.find(attachment.template_id) == attachment_templates.end()) {
      result.issues.push_back({ValidationSeverity::kError, "AttachmentTemplateMissing",
                               "Attachment references unknown AttachmentTemplate", attachment.id});
    }
  }

  for (const Span& span : edit_state.spans.items()) {
    const Port* port_a = edit_state.ports.find(span.port_a_id);
    const Port* port_b = edit_state.ports.find(span.port_b_id);
    if (port_a == nullptr || port_b == nullptr) {
      result.issues.push_back({ValidationSeverity::kError, "SpanPortMissing", "Span references missing port", span.id});
      continue;
    }
    if (span.port_a_id == span.port_b_id) {
      result.issues.push_back({ValidationSeverity::kError, "SpanSelfReference", "Span has same endpoint", span.id});
    }
    if (has_zero_length(*port_a, *port_b)) {
      result.issues.push_back({ValidationSeverity::kWarning, "SpanZeroLength", "Span endpoints overlap", span.id});
    }
    if (span.bundle_id != kInvalidObjectId && edit_state.bundles.find(span.bundle_id) == nullptr) {
      result.issues.push_back({ValidationSeverity::kError, "SpanBundleMissing", "Span bundle is missing", span.id});
    }
    if (span.anchor_a_id != kInvalidObjectId && edit_state.anchors.find(span.anchor_a_id) == nullptr) {
      result.issues.push_back({ValidationSeverity::kError, "SpanAnchorMissing", "Span anchorA is missing", span.id});
    }
    if (span.anchor_b_id != kInvalidObjectId && edit_state.anchors.find(span.anchor_b_id) == nullptr) {
      result.issues.push_back({ValidationSeverity::kError, "SpanAnchorMissing", "Span anchorB is missing", span.id});
    }
    if (span.endpoint_attachment_a_id != kInvalidObjectId &&
        edit_state.attachments.find(span.endpoint_attachment_a_id) == nullptr) {
      result.issues.push_back(
          {ValidationSeverity::kError, "SpanEndpointAttachmentMissing", "Span endpoint A attachment is missing", span.id});
    }
    if (span.endpoint_attachment_b_id != kInvalidObjectId &&
        edit_state.attachments.find(span.endpoint_attachment_b_id) == nullptr) {
      result.issues.push_back(
          {ValidationSeverity::kError, "SpanEndpointAttachmentMissing", "Span endpoint B attachment is missing", span.id});
    }
  }

  for (const auto& [span_id, override_value] : authoritative_.override_state.span_endpoint_by_span) {
    if (edit_state.spans.find(span_id) == nullptr) {
      result.issues.push_back({ValidationSeverity::kError, "SpanEndpointOverrideTargetMissing",
                               "Span endpoint override target is missing", span_id});
      continue;
    }
    if ((override_value.socket_a_id.has_value() && *override_value.socket_a_id < -1) ||
        (override_value.socket_b_id.has_value() && *override_value.socket_b_id < -1)) {
      result.issues.push_back({ValidationSeverity::kError, "SpanEndpointOverrideInvalid",
                               "Span endpoint socket override must be -1 or a valid socket id", span_id});
    }
  }

  for (const auto& [span_id, override_value] : authoritative_.override_state.span_support_by_span) {
    if (edit_state.spans.find(span_id) == nullptr) {
      result.issues.push_back({ValidationSeverity::kError, "SpanSupportOverrideTargetMissing",
                               "Span support override target is missing", span_id});
      continue;
    }
    if (override_value.branch_down_offset_m.has_value() &&
        (!std::isfinite(*override_value.branch_down_offset_m) || *override_value.branch_down_offset_m < 0.0)) {
      result.issues.push_back({ValidationSeverity::kError, "SpanSupportOverrideInvalid",
                               "Span support override branch down offset must be finite and >= 0", span_id});
    }
  }

  std::unordered_map<LoweredSupportGroupKey, ConnectionCategory, LoweredSupportGroupKeyHash> support_group_category_by_key{};
  std::unordered_map<LoweredSupportGroupKey, std::pair<ObjectId, ObjectId>, LoweredSupportGroupKeyHash>
      support_group_pair_by_key{};
  auto authoritative_pair_for_group = [](const SupportGroupDecision& group) {
    return std::pair<ObjectId, ObjectId>{group.decision.support_pair_peer_low, group.decision.support_pair_peer_high};
  };
  for (const auto& [span_id, layout] : cache_state.support_layout_cache.by_span) {
    const auto validate_endpoint = [&](const SupportLayoutEndpoint& endpoint, const char* code) {
      const Pole* endpoint_pole = edit_state.poles.find(endpoint.owner_pole_id);
      const Port* endpoint_port = edit_state.ports.find(endpoint.port_id);
      if (endpoint_pole != nullptr && endpoint_port != nullptr &&
          endpoint.decision.continuity_class == ContinuityCategoryClass::kBundleLike) {
        const double template_z = template_layer_base_z_for_validation(core, *endpoint_pole, endpoint_port->category);
        if (endpoint.decision.relation_kind == JunctionRelationKind::kThroughMain) {
          if (endpoint.decision.lower_required || endpoint.branch_down_offset_m > 1e-9 ||
              !almost_equal_validation(endpoint.support_world.z, template_z)) {
            result.issues.push_back({ValidationSeverity::kError, "ThroughMainHeightMismatch",
                                     "ThroughMain endpoint must stay at template height with no lowering offset",
                                     span_id});
          }
        } else if (endpoint.decision.lower_required && !endpoint.decision.lowering_blocked_by_policy) {
          if (endpoint.branch_down_offset_m <= 1e-9) {
            result.issues.push_back({ValidationSeverity::kError, "LoweredEndpointOffsetMissing",
                                     "Lowered non-through endpoint must carry a positive one-step down offset",
                                     span_id});
          } else {
            const double expected_z = template_z - endpoint.branch_down_offset_m;
            if (!almost_equal_validation(endpoint.support_world.z, expected_z)) {
              result.issues.push_back({ValidationSeverity::kError, "LoweredEndpointHeightNotTwoState",
                                       "Lowered non-through endpoint height must equal template height minus one-step down offset",
                                       span_id});
            }
          }
        } else if (endpoint.decision.lowering_blocked_by_policy &&
                   (!almost_equal_validation(endpoint.support_world.z, template_z) ||
                    endpoint.branch_down_offset_m > 1e-9)) {
          result.issues.push_back({ValidationSeverity::kError, "PolicyBlockedEndpointHeightMismatch",
                                   "Policy-blocked endpoint must stay at template height with no materialized lowering offset",
                                   span_id});
        }
      }
      if (endpoint.decision.support_orientation_basis != SupportOrientationBasisKind::kRadial &&
          (!endpoint.decision.has_side_axis || !std::isfinite(endpoint.decision.side_axis.x) ||
           !std::isfinite(endpoint.decision.side_axis.y))) {
        result.issues.push_back({ValidationSeverity::kError, code,
                                 "Non-radial support orientation must carry a finite authoritative side axis", span_id});
      }
      if (endpoint_uses_grouped_lowered_support_for_validation(endpoint) &&
          endpoint.decision.support_orientation_basis == SupportOrientationBasisKind::kRadial) {
        result.issues.push_back({ValidationSeverity::kError, "LoweredBundleLikeRadialBasis",
                                 "Grouped lowered support must not keep a radial orientation basis", span_id});
      }
    };
    validate_endpoint(layout.start, "SupportLayoutStartAxisMissing");
    validate_endpoint(layout.end, "SupportLayoutEndAxisMissing");

    const auto validate_grouped_endpoint_alignment = [&](const SupportLayoutEndpoint& endpoint) {
      if (!endpoint_uses_grouped_lowered_support_for_validation(endpoint)) {
        return;
      }
      if (endpoint.decision.owner_pole_id != endpoint.owner_pole_id) {
        result.issues.push_back({ValidationSeverity::kError, "SupportGroupOwnerMismatch",
                                 "Grouped-lowered endpoint decision owner does not match endpoint owner pole", span_id});
        return;
      }
      if (endpoint.decision.support_group_id < 0) {
        result.issues.push_back({ValidationSeverity::kError, "SupportGroupEndpointMissing",
                                 "Grouped-lowered endpoint does not resolve to a support group placement", span_id});
        return;
      }
      const LoweredSupportGroupKey key = LoweredSupportGroupKeyFromDecision(endpoint.decision);
      if (key.owner_pole_id != endpoint.decision.owner_pole_id || key.support_group_id != endpoint.decision.support_group_id) {
        result.issues.push_back({ValidationSeverity::kError, "SupportGroupKeyMismatch",
                                 "Grouped-lowered endpoint key must match authoritative decision owner/group id", span_id});
        return;
      }
      if (std::find(layout.lowered_support_group_keys.begin(), layout.lowered_support_group_keys.end(), key) ==
          layout.lowered_support_group_keys.end()) {
        result.issues.push_back({ValidationSeverity::kError, "SupportGroupEndpointMissing",
                                 "Grouped-lowered endpoint does not reference its support group placement", span_id});
        return;
      }
      const auto decision_it = cache_state.support_layout_cache.support_group_decisions.find(key);
      if (decision_it == cache_state.support_layout_cache.support_group_decisions.end()) {
        result.issues.push_back({ValidationSeverity::kError, "SupportGroupDecisionMissing",
                                 "Grouped-lowered endpoint references a missing support group decision", span_id});
        return;
      }
      if (const Port* endpoint_port = edit_state.ports.find(endpoint.port_id); endpoint_port != nullptr) {
        const auto [category_it, inserted] = support_group_category_by_key.emplace(key, endpoint_port->category);
        if (!inserted && category_it->second != endpoint_port->category) {
          result.issues.push_back({ValidationSeverity::kError, "SupportGroupCategoryMismatch",
                                   "Grouped-lowered support must not mix categories inside one support group",
                                   span_id});
        }
      }
      const SupportGroupDecision& authority = decision_it->second;
      if (authority.decision.continuity_class == ContinuityCategoryClass::kBundleLike) {
        if (!HasAuthoritativeSupportPair(decision_it->second.decision)) {
          result.issues.push_back({ValidationSeverity::kError, "SupportGroupPairMissing",
                                   "Grouped-lowered bundle support-group decision must carry one authoritative pole-incident pair",
                                   span_id});
        }
      }
      const auto it = cache_state.support_layout_cache.lowered_support_groups.find(key);
      if (it == cache_state.support_layout_cache.lowered_support_groups.end()) {
        result.issues.push_back({ValidationSeverity::kError, "SupportGroupEndpointMissing",
                                 "Grouped-lowered endpoint references a missing support group placement", span_id});
        return;
      }
      const LoweredSupportGroupPlacement& group = it->second;
      if (authority.decision.relation_kind == JunctionRelationKind::kThroughMain &&
          (authority.decision.side_assignment_rule != SideAssignmentRuleKind::kBisector ||
           authority.decision.support_orientation_rule != SupportOrientationRuleKind::kBisector ||
           authority.decision.support_orientation_basis !=
               CanonicalSupportOrientationBasis(SupportOrientationRuleKind::kBisector))) {
        result.issues.push_back({ValidationSeverity::kError, "SupportGroupThroughMainNotPairBased",
                                 "Grouped-lowered ThroughMain support-group decision must keep pair-based bisector orientation",
                                 span_id});
      }
      if (!almost_equal_validation(endpoint.support_world.z, group.tip_world.z)) {
        result.issues.push_back({ValidationSeverity::kError, "SupportGroupHeightMismatch",
                                 "Grouped-lowered endpoint support height must match grouped placement tip height",
                                 span_id});
      }
      if (endpoint.branch_down_offset_m <= 1e-9 ||
          !almost_equal_validation(endpoint.branch_down_offset_m, group.down_offset_m)) {
        result.issues.push_back({ValidationSeverity::kError, "SupportGroupOffsetMismatch",
                                 "Grouped-lowered endpoint must carry the authoritative one-step down offset",
                                 span_id});
      }
    };
    validate_grouped_endpoint_alignment(layout.start);
    validate_grouped_endpoint_alignment(layout.end);
  }

  for (const auto& [key, group_decision] : cache_state.support_layout_cache.support_group_decisions) {
    if (group_decision.decision.owner_pole_id != key.owner_pole_id ||
        group_decision.decision.support_group_id != key.support_group_id) {
      result.issues.push_back({ValidationSeverity::kError, "SupportGroupDecisionKeyMismatch",
                               "Support-group decision key must match authoritative decision owner/group id",
                               key.owner_pole_id});
    }
    if (!UsesAuthoritativeGroupedLoweredSupport(group_decision.decision)) {
      result.issues.push_back({ValidationSeverity::kError, "SupportGroupDecisionNotAuthoritative",
                               "Support-group decision must be backed by an authoritative lowered decision",
                               key.owner_pole_id});
    }
    if (group_decision.decision.continuity_class == ContinuityCategoryClass::kBundleLike &&
        !HasAuthoritativeSupportPair(group_decision.decision)) {
      result.issues.push_back({ValidationSeverity::kError, "SupportGroupPairMissing",
                               "Support-group decision must keep the authoritative support pair decision",
                               key.owner_pole_id});
    }
    if (group_decision.decision.support_orientation_basis == SupportOrientationBasisKind::kRadial ||
        group_decision.decision.support_orientation_rule == SupportOrientationRuleKind::kRadial ||
        group_decision.decision.side_assignment_rule == SideAssignmentRuleKind::kPoleLocal ||
        !group_decision.decision.has_side_axis || !std::isfinite(group_decision.decision.side_axis.x) ||
        !std::isfinite(group_decision.decision.side_axis.y)) {
      result.issues.push_back({ValidationSeverity::kError, "SupportGroupDecisionIncomplete",
                               "Support-group decision must carry non-radial authoritative orientation/axis fields",
                               key.owner_pole_id});
    }
    const auto [pair_it, inserted] = support_group_pair_by_key.emplace(key, authoritative_pair_for_group(group_decision));
    if (!inserted && pair_it->second != authoritative_pair_for_group(group_decision)) {
      result.issues.push_back({ValidationSeverity::kError, "SupportGroupPairMismatch",
                               "Same support group must share one authoritative support-group pair decision",
                               key.owner_pole_id});
    }
    if (group_decision.decision.relation_kind == JunctionRelationKind::kThroughMain &&
        (group_decision.decision.side_assignment_rule != SideAssignmentRuleKind::kBisector ||
         group_decision.decision.support_orientation_rule != SupportOrientationRuleKind::kBisector ||
         group_decision.decision.support_orientation_basis !=
             CanonicalSupportOrientationBasis(SupportOrientationRuleKind::kBisector))) {
      result.issues.push_back({ValidationSeverity::kError, "SupportGroupThroughMainNotPairBased",
                               "Grouped-lowered ThroughMain support-group decision must keep pair-based bisector orientation",
                               key.owner_pole_id});
    }
    if (!std::isfinite(group_decision.support_world.z)) {
      result.issues.push_back({ValidationSeverity::kError, "SupportGroupHeightInvalid",
                               "Support-group decision must carry a finite authoritative support z", key.owner_pole_id});
    }
    const Pole* pole = edit_state.poles.find(key.owner_pole_id);
    const auto category_it = support_group_category_by_key.find(key);
    if (pole != nullptr && category_it != support_group_category_by_key.end() && group_decision.down_offset_m > 1e-9) {
      const double expected_support_z =
          template_layer_base_z_for_validation(core, *pole, category_it->second) - group_decision.down_offset_m;
      if (!almost_equal_validation(group_decision.support_world.z, expected_support_z)) {
        result.issues.push_back({ValidationSeverity::kError, "SupportGroupHeightNotTwoState",
                                 "Support-group decision height must equal template height minus one-step down offset",
                                 key.owner_pole_id});
      }
    }
  }

  for (const auto& [key, group] : cache_state.support_layout_cache.lowered_support_groups) {
    const auto decision_it = cache_state.support_layout_cache.support_group_decisions.find(key);
    if (decision_it == cache_state.support_layout_cache.support_group_decisions.end()) {
      result.issues.push_back({ValidationSeverity::kError, "SupportGroupDecisionMissing",
                               "Grouped placement must have a matching support-group decision", key.owner_pole_id});
      continue;
    }
    const SupportGroupDecision& authority = decision_it->second;
    if (group.grouped_port_count != static_cast<int>(group.attachment_worlds.size())) {
      result.issues.push_back({ValidationSeverity::kError, "SupportGroupAttachmentCountMismatch",
                               "Grouped lowered support must carry one attachment world per grouped port",
                               key.owner_pole_id});
    }
    if (!almost_equal_validation(group.mount_world.z, authority.support_world.z) ||
        !almost_equal_validation(group.tip_world.z, authority.support_world.z)) {
      result.issues.push_back({ValidationSeverity::kError, "SupportGroupHeightMismatch",
                               "Grouped placement mount/tip z must match support-group decision support z",
                               key.owner_pole_id});
    }
    const Pole* pole = edit_state.poles.find(key.owner_pole_id);
    const auto category_it = support_group_category_by_key.find(key);
    if (pole != nullptr && category_it != support_group_category_by_key.end() && group.down_offset_m > 1e-9) {
      const double expected_support_z =
          template_layer_base_z_for_validation(core, *pole, category_it->second) - group.down_offset_m;
      if (!almost_equal_validation(group.mount_world.z, expected_support_z) ||
          !almost_equal_validation(group.tip_world.z, expected_support_z)) {
        result.issues.push_back({ValidationSeverity::kError, "SupportGroupHeightNotTwoState",
                                 "Grouped placement height must equal template height minus one-step down offset",
                                 key.owner_pole_id});
      }
    }
    Vec3d support_axis = group.tip_world - group.mount_world;
    support_axis.z = 0.0;
    if (Normalize(&support_axis) && IsFiniteXY(support_axis)) {
      Vec3d authoritative_axis = AuthoritativeSupportAxisForGroup(authority);
      if (!Normalize(&authoritative_axis) || !IsFiniteXY(authoritative_axis)) {
        result.issues.push_back({ValidationSeverity::kError, "SupportGroupAxisMissing",
                                 "Grouped lowered support must carry a finite authoritative axis",
                                 key.owner_pole_id});
      } else {
        const double alignment = support_axis.x * authoritative_axis.x + support_axis.y * authoritative_axis.y;
        if (alignment < 1.0 - 1e-6) {
        result.issues.push_back({ValidationSeverity::kError, "SupportGroupAxisReinterpreted",
                                 "Grouped lowered support mount/tip must stay aligned with the authoritative axis",
                                 key.owner_pole_id});
        }
      }
    }
  }

  if (cache_state.support_layout_cache.support_group_decisions.size() !=
      cache_state.support_layout_cache.lowered_support_groups.size()) {
    result.issues.push_back({ValidationSeverity::kError, "SupportGroupPlacementCountMismatch",
                             "support_group_decisions and lowered_support_groups must stay 1:1",
                             kInvalidObjectId});
  }

  for (const auto& [span_id, layout] : cache_state.support_layout_cache.by_span) {
    std::unordered_set<LoweredSupportGroupKey, LoweredSupportGroupKeyHash> seen_group_keys{};
    for (const LoweredSupportGroupKey& key : layout.lowered_support_group_keys) {
      if (!seen_group_keys.insert(key).second) {
        result.issues.push_back({ValidationSeverity::kError, "SupportGroupPlacementDuplicateRef",
                                 "Support layout must not reference the same grouped lowered support twice", span_id});
      }
      if (cache_state.support_layout_cache.support_group_decisions.find(key) ==
          cache_state.support_layout_cache.support_group_decisions.end()) {
        result.issues.push_back({ValidationSeverity::kError, "SupportGroupDecisionMissing",
                                 "Support layout references a missing support-group decision", span_id});
      }
      if (cache_state.support_layout_cache.lowered_support_groups.find(key) ==
          cache_state.support_layout_cache.lowered_support_groups.end()) {
        result.issues.push_back({ValidationSeverity::kError, "SupportGroupPlacementMissing",
                                 "Support layout references a missing grouped lowered support placement", span_id});
      }
    }
  }

  const auto expected_port_index = canonical_index_map(make_expected_port_index(edit_state));
  const auto expected_anchor_index = canonical_index_map(make_expected_anchor_index(edit_state));
  const auto expected_pole_port_index = canonical_index_map(make_expected_pole_port_index(edit_state));
  const auto expected_pole_anchor_index = canonical_index_map(make_expected_pole_anchor_index(edit_state));
  const auto expected_bundle_span_index = canonical_index_map(make_expected_bundle_span_index(edit_state));
  const auto expected_span_attachment_index = canonical_index_map(make_expected_span_attachment_index(edit_state));
  const auto actual_port_index = canonical_index_map(connection_index.spans_by_port);
  const auto actual_anchor_index = canonical_index_map(connection_index.spans_by_anchor);
  const auto actual_pole_port_index = canonical_index_map(relation_index.ports_by_pole);
  const auto actual_pole_anchor_index = canonical_index_map(relation_index.anchors_by_pole);
  const auto actual_bundle_span_index = canonical_index_map(relation_index.spans_by_bundle);
  const auto actual_span_attachment_index = canonical_index_map(relation_index.attachments_by_span);

  if (expected_port_index != actual_port_index) {
    result.issues.push_back(
        {ValidationSeverity::kError, "PortIndexMismatch", "Port->Span index mismatch", kInvalidObjectId});
  }
  if (expected_anchor_index != actual_anchor_index) {
    result.issues.push_back(
        {ValidationSeverity::kError, "AnchorIndexMismatch", "Anchor->Span index mismatch", kInvalidObjectId});
  }
  if (expected_pole_port_index != actual_pole_port_index) {
    result.issues.push_back(
        {ValidationSeverity::kError, "PolePortIndexMismatch", "Pole->Port index mismatch", kInvalidObjectId});
  }
  if (expected_pole_anchor_index != actual_pole_anchor_index) {
    result.issues.push_back(
        {ValidationSeverity::kError, "PoleAnchorIndexMismatch", "Pole->Anchor index mismatch", kInvalidObjectId});
  }
  if (expected_bundle_span_index != actual_bundle_span_index) {
    result.issues.push_back(
        {ValidationSeverity::kError, "BundleSpanIndexMismatch", "Bundle->Span index mismatch", kInvalidObjectId});
  }
  if (expected_span_attachment_index != actual_span_attachment_index) {
    result.issues.push_back({ValidationSeverity::kError, "SpanAttachmentIndexMismatch",
                             "Span->Attachment index mismatch", kInvalidObjectId});
  }
  for (const auto& [port_id, span_ids] : connection_index.spans_by_port) {
    if (has_duplicate_ids(span_ids)) {
      result.issues.push_back({
          ValidationSeverity::kError,
          "PortIndexDuplicateEntries",
          "Port->Span index contains duplicate span ids",
          port_id,
      });
    }
  }
  for (const auto& [anchor_id, span_ids] : connection_index.spans_by_anchor) {
    if (has_duplicate_ids(span_ids)) {
      result.issues.push_back({
          ValidationSeverity::kError,
          "AnchorIndexDuplicateEntries",
          "Anchor->Span index contains duplicate span ids",
          anchor_id,
      });
    }
  }
  for (const auto& [pole_id, port_ids] : relation_index.ports_by_pole) {
    if (has_duplicate_ids(port_ids)) {
      result.issues.push_back({
          ValidationSeverity::kError,
          "PolePortIndexDuplicateEntries",
          "Pole->Port index contains duplicate port ids",
          pole_id,
      });
    }
  }
  for (const auto& [pole_id, anchor_ids] : relation_index.anchors_by_pole) {
    if (has_duplicate_ids(anchor_ids)) {
      result.issues.push_back({
          ValidationSeverity::kError,
          "PoleAnchorIndexDuplicateEntries",
          "Pole->Anchor index contains duplicate anchor ids",
          pole_id,
      });
    }
  }
  for (const auto& [bundle_id, span_ids] : relation_index.spans_by_bundle) {
    if (has_duplicate_ids(span_ids)) {
      result.issues.push_back({
          ValidationSeverity::kError,
          "BundleSpanIndexDuplicateEntries",
          "Bundle->Span index contains duplicate span ids",
          bundle_id,
      });
    }
  }
  for (const auto& [span_id, attachment_ids] : relation_index.attachments_by_span) {
    if (has_duplicate_ids(attachment_ids)) {
      result.issues.push_back({ValidationSeverity::kError, "SpanAttachmentIndexDuplicateEntries",
                               "Span->Attachment index contains duplicate attachment ids", span_id});
    }
  }
  for (const auto& [port_id, span_ids] : connection_index.spans_by_port) {
    if (edit_state.ports.find(port_id) == nullptr) {
      result.issues.push_back({
          ValidationSeverity::kError,
          "PortIndexDanglingPort",
          "Port index references removed port",
          port_id,
      });
      continue;
    }
    for (ObjectId span_id : span_ids) {
      if (edit_state.spans.find(span_id) == nullptr) {
        result.issues.push_back({
            ValidationSeverity::kError,
            "PortIndexDanglingSpan",
            "Port index references removed span",
            port_id,
        });
        break;
      }
    }
  }
  for (const auto& [anchor_id, span_ids] : connection_index.spans_by_anchor) {
    if (edit_state.anchors.find(anchor_id) == nullptr) {
      result.issues.push_back({
          ValidationSeverity::kError,
          "AnchorIndexDanglingAnchor",
          "Anchor index references removed anchor",
          anchor_id,
      });
      continue;
    }
    for (ObjectId span_id : span_ids) {
      if (edit_state.spans.find(span_id) == nullptr) {
        result.issues.push_back({
            ValidationSeverity::kError,
            "AnchorIndexDanglingSpan",
            "Anchor index references removed span",
            anchor_id,
        });
        break;
      }
    }
  }
  for (const auto& [pole_id, port_ids] : relation_index.ports_by_pole) {
    if (edit_state.poles.find(pole_id) == nullptr) {
      result.issues.push_back({
          ValidationSeverity::kError,
          "PolePortIndexDanglingPole",
          "Pole->Port index references removed pole",
          pole_id,
      });
      continue;
    }
    for (ObjectId port_id : port_ids) {
      const Port* port = edit_state.ports.find(port_id);
      if (port == nullptr || port->owner_pole_id != pole_id) {
        result.issues.push_back({
            ValidationSeverity::kError,
            "PolePortIndexDanglingPort",
            "Pole->Port index references removed or mismatched port",
            pole_id,
        });
        break;
      }
    }
  }
  for (const auto& [pole_id, anchor_ids] : relation_index.anchors_by_pole) {
    if (edit_state.poles.find(pole_id) == nullptr) {
      result.issues.push_back({
          ValidationSeverity::kError,
          "PoleAnchorIndexDanglingPole",
          "Pole->Anchor index references removed pole",
          pole_id,
      });
      continue;
    }
    for (ObjectId anchor_id : anchor_ids) {
      const Anchor* anchor = edit_state.anchors.find(anchor_id);
      if (anchor == nullptr || anchor->owner_pole_id != pole_id) {
        result.issues.push_back({
            ValidationSeverity::kError,
            "PoleAnchorIndexDanglingAnchor",
            "Pole->Anchor index references removed or mismatched anchor",
            pole_id,
        });
        break;
      }
    }
  }
  for (const auto& [bundle_id, span_ids] : relation_index.spans_by_bundle) {
    if (edit_state.bundles.find(bundle_id) == nullptr) {
      result.issues.push_back({
          ValidationSeverity::kError,
          "BundleSpanIndexDanglingBundle",
          "Bundle->Span index references removed bundle",
          bundle_id,
      });
      continue;
    }
    for (ObjectId span_id : span_ids) {
      const Span* span = edit_state.spans.find(span_id);
      if (span == nullptr || span->bundle_id != bundle_id) {
        result.issues.push_back({
            ValidationSeverity::kError,
            "BundleSpanIndexDanglingSpan",
            "Bundle->Span index references removed or mismatched span",
            bundle_id,
        });
        break;
      }
    }
  }
  for (const auto& [span_id, attachment_ids] : relation_index.attachments_by_span) {
    if (edit_state.spans.find(span_id) == nullptr) {
      result.issues.push_back({ValidationSeverity::kError, "SpanAttachmentIndexDanglingSpan",
                               "Span->Attachment index references removed span", span_id});
      continue;
    }
    for (ObjectId attachment_id : attachment_ids) {
      const Attachment* attachment = edit_state.attachments.find(attachment_id);
      if (attachment == nullptr || attachment->span_id != span_id) {
        result.issues.push_back({ValidationSeverity::kError, "SpanAttachmentIndexDanglingAttachment",
                                 "Span->Attachment index references removed or mismatched attachment", span_id});
        break;
      }
    }
  }

  for (const Span& span : edit_state.spans.items()) {
    auto it = span_runtime_states.find(span.id);
    if (it == span_runtime_states.end()) {
      result.issues.push_back(
          {ValidationSeverity::kError, "SpanRuntimeMissing", "Span runtime state missing", span.id});
      continue;
    }
    if (it->second.span_id != span.id) {
      result.issues.push_back({ValidationSeverity::kError, "SpanRuntimeCorrupt", "Span runtime id mismatch", span.id});
    }
  }
  for (const auto& [span_id, runtime] : span_runtime_states) {
    if (edit_state.spans.find(span_id) == nullptr || runtime.span_id != span_id) {
      result.issues.push_back(
          {ValidationSeverity::kError, "SpanRuntimeDangling", "Runtime state points to removed span", span_id});
    }
  }

  for (const Span& span : edit_state.spans.items()) {
    const auto runtime_it = span_runtime_states.find(span.id);
    if (runtime_it == span_runtime_states.end()) {
      continue;
    }
    const SpanRuntimeState& runtime = runtime_it->second;

    auto curve_it = cache_state.curve_cache.by_span.find(span.id);
    if (curve_it != cache_state.curve_cache.by_span.end()) {
      const CurveCacheEntry& curve = curve_it->second;
      if (curve.points.size() < 2) {
        result.issues.push_back({
            ValidationSeverity::kError,
            "CurveSampleCountInvalid",
            "Curve cache has less than 2 points",
            span.id,
        });
      }
      if (curve.source_version != runtime.geometry_version) {
        result.issues.push_back({
            ValidationSeverity::kWarning,
            "GeometryVersionMismatch",
            "Curve cache sourceVersion does not match geometryVersion",
            span.id,
        });
      }
    }

    auto bounds_it = cache_state.bounds_cache.by_span.find(span.id);
    if (bounds_it != cache_state.bounds_cache.by_span.end()) {
      const BoundsCacheEntry& bounds = bounds_it->second;
      if (bounds.whole.min.x > bounds.whole.max.x || bounds.whole.min.y > bounds.whole.max.y ||
          bounds.whole.min.z > bounds.whole.max.z) {
        result.issues.push_back({
            ValidationSeverity::kError,
            "BoundsInvalid",
            "Whole bounds has min > max",
            span.id,
        });
      }
      for (const AABBd& segment : bounds.segments) {
        if (segment.min.x > segment.max.x || segment.min.y > segment.max.y || segment.min.z > segment.max.z) {
          result.issues.push_back({
              ValidationSeverity::kError,
              "SegmentBoundsInvalid",
              "Segment bounds has min > max",
              span.id,
          });
          break;
        }
      }
      if (bounds.source_version != runtime.bounds_version) {
        result.issues.push_back({
            ValidationSeverity::kWarning,
            "BoundsVersionMismatch",
            "Bounds cache sourceVersion does not match boundsVersion",
            span.id,
        });
      }
    }
  }

  for (const PortResolutionDebugRecord& debug : port_resolution_debug_records) {
    if (!std::isfinite(debug.corner_turn_sign)) {
      result.issues.push_back({
          ValidationSeverity::kError,
          "PortResolutionDebugInvalid",
          "Port resolution debug corner_turn_sign is non-finite",
          debug.pole_id,
      });
    }
    if (debug.selected_port_id != kInvalidObjectId) {
      if (debug.created_new_port) {
        continue;
      }
      bool found = false;
      for (const PlacementCandidateDebug& c : debug.candidates) {
        if (c.resolved_port_id == debug.selected_port_id) {
          found = true;
          if (c.total_score != c.category_score + c.context_score + c.layer_score + c.side_score + c.role_score +
                                    c.priority_score + c.usage_score + c.congestion_score + (c.tie_breaker / 16)) {
            result.issues.push_back({
                ValidationSeverity::kWarning,
                "PortResolutionScoreInconsistent",
                "Placement candidate score breakdown does not match total_score",
                debug.pole_id,
            });
          }
          break;
        }
      }
      if (!found) {
        result.issues.push_back({
            ValidationSeverity::kError,
            "PortResolutionDebugMismatch",
            "Selected port id does not exist in candidate list",
            debug.pole_id,
        });
      }
    }
  }

  return result;
}

} // namespace wire::core

