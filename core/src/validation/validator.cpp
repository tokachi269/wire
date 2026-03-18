#include "wire/core/core_state.hpp"
#include "wire/core/coord_utils.hpp"
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

bool endpoint_uses_grouped_lowered_support_for_validation(const SupportLayoutEndpoint& endpoint,
                                                          BackboneLoweringKind lowering_kind) {
  return UsesGroupedLoweredSupport(endpoint, lowering_kind);
}

bool almost_equal_validation(double a, double b, double eps = 1e-9) { return std::abs(a - b) <= eps; }

bool almost_equal_validation(const Vec3d& a, const Vec3d& b, double eps = 1e-9) {
  return almost_equal_validation(a.x, b.x, eps) && almost_equal_validation(a.y, b.y, eps) &&
         almost_equal_validation(a.z, b.z, eps);
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

  for (const auto& [pole_id, override_value] : override_state_.pole_orientation_by_pole) {
    if (edit_state.poles.find(pole_id) == nullptr) {
      result.issues.push_back(
          {ValidationSeverity::kError, "PoleOverrideTargetMissing", "Pole orientation override target is missing", pole_id});
    } else if (override_value.manual_yaw_deg.has_value() && !std::isfinite(*override_value.manual_yaw_deg)) {
      result.issues.push_back(
          {ValidationSeverity::kError, "PoleOverrideInvalid", "Pole orientation override has non-finite yaw", pole_id});
    }
  }

  for (const auto& [span_id, override_value] : override_state_.span_endpoint_by_span) {
    if (edit_state.spans.find(span_id) == nullptr) {
      result.issues.push_back({ValidationSeverity::kError, "SpanEndpointOverrideTargetMissing",
                               "Span endpoint override target is missing", span_id});
    } else if ((override_value.socket_a_id.has_value() && *override_value.socket_a_id < -1) ||
               (override_value.socket_b_id.has_value() && *override_value.socket_b_id < -1)) {
      result.issues.push_back({ValidationSeverity::kError, "SpanEndpointOverrideInvalid",
                               "Span endpoint socket override must be -1 or a valid socket id", span_id});
    }
  }

  for (const auto& [span_id, override_value] : override_state_.span_support_by_span) {
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

  for (const auto& [pole_id, override_value] : override_state_.pole_orientation_by_pole) {
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

  for (const auto& [span_id, override_value] : override_state_.span_endpoint_by_span) {
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

  for (const auto& [span_id, override_value] : override_state_.span_support_by_span) {
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

  for (const auto& [span_id, layout] : cache_state.support_layout_cache.by_span) {
    const auto validate_endpoint = [&](const SupportLayoutEndpoint& endpoint, const char* code) {
      if (endpoint.decision.support_orientation_basis != SupportOrientationBasisKind::kRadial &&
          (!endpoint.has_side_axis || !std::isfinite(endpoint.side_axis.x) || !std::isfinite(endpoint.side_axis.y))) {
        result.issues.push_back({ValidationSeverity::kError, code,
                                 "Non-radial support orientation must carry a finite authoritative side axis", span_id});
      }
      if (endpoint_uses_grouped_lowered_support_for_validation(endpoint, layout.lowering_kind) &&
          endpoint.decision.support_orientation_basis == SupportOrientationBasisKind::kRadial) {
        result.issues.push_back({ValidationSeverity::kError, "LoweredBundleLikeRadialBasis",
                                 "Grouped lowered support must not keep a radial orientation basis", span_id});
      }
    };
    validate_endpoint(layout.start, "SupportLayoutStartAxisMissing");
    validate_endpoint(layout.end, "SupportLayoutEndAxisMissing");
  }

  struct SupportGroupPlacementSnapshot {
    ObjectId span_id = kInvalidObjectId;
    ObjectId owner_pole_id = kInvalidObjectId;
    int support_group_id = -1;
    SupportOrientationBasisKind basis = SupportOrientationBasisKind::kRadial;
    double chosen_side_sign = 0.0;
    Vec3d side_axis{};
    double branch_down_offset_m = 0.0;
    Vec3d mount_world{};
    Vec3d tip_world{};
  };
  struct SupportGroupKey {
    ObjectId owner_pole_id = kInvalidObjectId;
    int support_group_id = -1;
    bool operator==(const SupportGroupKey& other) const {
      return owner_pole_id == other.owner_pole_id && support_group_id == other.support_group_id;
    }
  };
  struct SupportGroupKeyHash {
    std::size_t operator()(const SupportGroupKey& key) const {
      const std::size_t h1 = std::hash<ObjectId>{}(key.owner_pole_id);
      const std::size_t h2 = std::hash<int>{}(key.support_group_id);
      return h1 ^ (h2 + 0x9e3779b9 + (h1 << 6) + (h1 >> 2));
    }
  };
  std::unordered_map<SupportGroupKey, SupportGroupPlacementSnapshot, SupportGroupKeyHash> support_groups{};
  for (const auto& [span_id, layout] : cache_state.support_layout_cache.by_span) {
    for (const LoweredSupportGroupPlacement& group : layout.lowered_support_groups) {
      const SupportGroupKey key{group.owner_pole_id, group.support_group_id};
      SupportGroupPlacementSnapshot snapshot{};
      snapshot.span_id = span_id;
      snapshot.owner_pole_id = group.owner_pole_id;
      snapshot.support_group_id = key.support_group_id;
      snapshot.basis = group.decision.support_orientation_basis;
      snapshot.chosen_side_sign = group.chosen_side_sign;
      snapshot.side_axis = group.side_axis;
      snapshot.branch_down_offset_m = group.down_offset_m;
      snapshot.mount_world = group.mount_world;
      snapshot.tip_world = group.tip_world;
      const auto [it, inserted] = support_groups.emplace(key, snapshot);
      if (inserted) {
        continue;
      }
      const SupportGroupPlacementSnapshot& first = it->second;
      const bool same_snapshot = first.basis == snapshot.basis &&
                                 almost_equal_validation(first.chosen_side_sign, snapshot.chosen_side_sign) &&
                                 almost_equal_validation(first.side_axis, snapshot.side_axis) &&
                                 almost_equal_validation(first.branch_down_offset_m, snapshot.branch_down_offset_m) &&
                                 almost_equal_validation(first.mount_world, snapshot.mount_world) &&
                                 almost_equal_validation(first.tip_world, snapshot.tip_world);
      if (!same_snapshot) {
        result.issues.push_back({ValidationSeverity::kError, "SupportGroupPlacementConflict",
                                 "Grouped lowered support id resolves to multiple conflicting placements", span_id});
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
