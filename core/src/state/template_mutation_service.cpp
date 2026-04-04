#include "internal_services.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_set>

namespace wire::core::state_internal {

namespace {

template <typename TValue> void append_unique(std::vector<TValue>& dst, const std::vector<TValue>& src) {
  for (const TValue& value : src) {
    if (std::find(dst.begin(), dst.end(), value) == dst.end()) {
      dst.push_back(value);
    }
  }
}

bool attachment_socket_equals(const AttachmentSocketTemplate& a, const AttachmentSocketTemplate& b) {
  const auto same_vec = [](const Vec3d& lhs, const Vec3d& rhs) {
    return std::abs(lhs.x - rhs.x) <= 1e-12 && std::abs(lhs.y - rhs.y) <= 1e-12 && std::abs(lhs.z - rhs.z) <= 1e-12;
  };
  return a.id == b.id && same_vec(a.local_position, b.local_position) && same_vec(a.tangent_dir, b.tangent_dir) &&
         a.has_normal == b.has_normal && same_vec(a.normal_dir, b.normal_dir) &&
         a.has_binormal == b.has_binormal && same_vec(a.binormal_dir, b.binormal_dir) && a.kind == b.kind;
}

bool attachment_internal_path_equals(const AttachmentInternalPathTemplate& a, const AttachmentInternalPathTemplate& b) {
  if (a.start_socket_id != b.start_socket_id || a.end_socket_id != b.end_socket_id ||
      a.profile_kind != b.profile_kind || a.local_points.size() != b.local_points.size() ||
      std::abs(a.coil_radius_m - b.coil_radius_m) > 1e-12 || a.coil_turn_count != b.coil_turn_count ||
      a.coil_samples_per_turn != b.coil_samples_per_turn) {
    return false;
  }
  for (std::size_t i = 0; i < a.local_points.size(); ++i) {
    const Vec3d& lhs = a.local_points[i];
    const Vec3d& rhs = b.local_points[i];
    if (std::abs(lhs.x - rhs.x) > 1e-12 || std::abs(lhs.y - rhs.y) > 1e-12 || std::abs(lhs.z - rhs.z) > 1e-12) {
      return false;
    }
  }
  return true;
}

bool attachment_template_equals(const AttachmentTemplate& a, const AttachmentTemplate& b) {
  if (a.id != b.id || a.name != b.name || a.kind != b.kind || a.line_interaction_mode != b.line_interaction_mode ||
      a.sockets.size() != b.sockets.size() || a.internal_paths.size() != b.internal_paths.size()) {
    return false;
  }
  for (std::size_t i = 0; i < a.sockets.size(); ++i) {
    if (!attachment_socket_equals(a.sockets[i], b.sockets[i])) {
      return false;
    }
  }
  for (std::size_t i = 0; i < a.internal_paths.size(); ++i) {
    if (!attachment_internal_path_equals(a.internal_paths[i], b.internal_paths[i])) {
      return false;
    }
  }
  return true;
}

bool cable_supplemental_path_equals(const CableSupplementalPathTemplate& a, const CableSupplementalPathTemplate& b) {
  return a.anchor_mode == b.anchor_mode && a.profile_kind == b.profile_kind && a.pole_band_id == b.pole_band_id &&
         std::abs(a.endpoint_trim_m - b.endpoint_trim_m) <= 1e-12 &&
         std::abs(a.lateral_offset_m - b.lateral_offset_m) <= 1e-12 &&
         std::abs(a.vertical_offset_m - b.vertical_offset_m) <= 1e-12 &&
         std::abs(a.coil_radius_m - b.coil_radius_m) <= 1e-12 &&
         std::abs(a.coil_turns_per_meter - b.coil_turns_per_meter) <= 1e-12 &&
         a.coil_samples_per_turn == b.coil_samples_per_turn;
}

} // namespace

EditResult<bool> TemplateMutationService::UpdateCableTemplate(CoreState& state, const CableTemplate& cable_template,
                                                              const std::vector<ObjectId>& preferred_visible_span_ids) {
  EditResult<bool> result;
  auto it = state.authoritative_.cable_templates.find(cable_template.id);
  if (it == state.authoritative_.cable_templates.end()) {
    result.error = "cable template not found";
    return result;
  }

  CableTemplate normalized = cable_template;
  normalized.outer_diameter_m = std::max(0.0, normalized.outer_diameter_m);
  normalized.default_grouped_support_fanout_spacing_m = std::max(0.0, normalized.default_grouped_support_fanout_spacing_m);
  normalized.bend_stiffness = std::max(0.0, normalized.bend_stiffness);
  normalized.min_bend_radius_m = std::max(0.0, normalized.min_bend_radius_m);
  normalized.insulator_attachment_height_m = std::max(0.0, normalized.insulator_attachment_height_m);
  normalized.sag_factor = std::max(0.0, normalized.sag_factor);
  normalized.slack_factor = std::max(0.0, normalized.slack_factor);
  for (auto& path : normalized.supplemental_paths) {
    path.endpoint_trim_m = std::max(0.0, path.endpoint_trim_m);
    path.coil_radius_m = std::max(0.0, path.coil_radius_m);
    path.coil_turns_per_meter = std::max(0.0, path.coil_turns_per_meter);
    path.coil_samples_per_turn = std::max(4, path.coil_samples_per_turn);
  }
  normalized.version = it->second.version;
  bool supplemental_paths_changed = normalized.supplemental_paths.size() != it->second.supplemental_paths.size();
  if (!supplemental_paths_changed) {
    for (std::size_t i = 0; i < normalized.supplemental_paths.size(); ++i) {
      if (!cable_supplemental_path_equals(normalized.supplemental_paths[i], it->second.supplemental_paths[i])) {
        supplemental_paths_changed = true;
        break;
      }
    }
  }
  const bool metadata_change = normalized.name != it->second.name;
  const bool geometry_change =
      std::abs(normalized.outer_diameter_m - it->second.outer_diameter_m) > 1e-12 ||
      std::abs(normalized.default_grouped_support_fanout_spacing_m -
               it->second.default_grouped_support_fanout_spacing_m) > 1e-12 ||
      std::abs(normalized.bend_stiffness - it->second.bend_stiffness) > 1e-12 ||
      std::abs(normalized.min_bend_radius_m - it->second.min_bend_radius_m) > 1e-12 ||
      std::abs(normalized.sag_factor - it->second.sag_factor) > 1e-12 ||
      std::abs(normalized.slack_factor - it->second.slack_factor) > 1e-12 || supplemental_paths_changed;
  const bool render_change =
      normalized.material_style != it->second.material_style || normalized.color_rgba != it->second.color_rgba ||
      normalized.requires_insulator != it->second.requires_insulator ||
      std::abs(normalized.insulator_attachment_height_m - it->second.insulator_attachment_height_m) > 1e-12 ||
      normalized.attachment_style != it->second.attachment_style;
  const bool decision_change =
      normalized.continuity_policy != it->second.continuity_policy ||
      normalized.default_endpoint_attachment_template_id != it->second.default_endpoint_attachment_template_id;
  const bool changed = metadata_change || geometry_change || render_change || decision_change;
  if (!changed) {
    result.ok = true;
    result.value = false;
    return result;
  }

  normalized.version += 1;
  it->second = normalized;
  result.ok = true;
  result.value = true;

  std::vector<ObjectId> ordered_target_span_ids{};
  std::unordered_set<ObjectId> target_span_ids{};
  ordered_target_span_ids.reserve(preferred_visible_span_ids.size() + state.runtime_.relation_index.spans_by_bundle.size());
  for (ObjectId span_id : preferred_visible_span_ids) {
    if (target_span_ids.insert(span_id).second) {
      ordered_target_span_ids.push_back(span_id);
    }
  }
  for (const auto& [bundle_id, span_ids] : state.runtime_.relation_index.spans_by_bundle) {
    const Bundle* bundle = state.authoritative_.edit_state.bundles.find(bundle_id);
    if (bundle == nullptr) {
      continue;
    }
    const BundleTemplate* bundle_template = state.find_bundle_template(bundle->bundle_template_id);
    if (bundle_template == nullptr || bundle_template->cable_template_id != normalized.id) {
      continue;
    }
    for (ObjectId span_id : span_ids) {
      if (target_span_ids.insert(span_id).second) {
        ordered_target_span_ids.push_back(span_id);
      }
    }
  }
  for (ObjectId span_id : ordered_target_span_ids) {
    DirtyBits bits = DirtyBits::kNone;
    if (decision_change) {
      bits |= DirtyBits::kDecision;
    } else {
      if (geometry_change) {
        bits |= DirtyBits::kGeometryRefresh;
      }
      if (render_change) {
        bits |= DirtyBits::kRenderRefresh;
      }
    }
    if (bits != DirtyBits::kNone) {
      state.mark_span_dirty(span_id, bits, true);
      CoreState::add_unique_id(result.change_set.dirty_span_ids, span_id);
      CoreState::add_unique_id(result.change_set.updated_ids, span_id);
    }
  }
  return result;
}

EditResult<bool> TemplateMutationService::UpdateBundleTemplate(CoreState& state, const BundleTemplate& bundle_template) {
  EditResult<bool> result;
  auto it = state.authoritative_.bundle_templates.find(bundle_template.id);
  if (it == state.authoritative_.bundle_templates.end()) {
    result.error = "bundle template not found";
    return result;
  }
  if (state.find_cable_template(bundle_template.cable_template_id) == nullptr) {
    result.error = "bundle template references unknown cable template";
    return result;
  }
  if (bundle_template.related_pole_type_id != kInvalidPoleTypeId &&
      state.find_pole_type(bundle_template.related_pole_type_id) == nullptr) {
    result.error = "bundle template references unknown pole type";
    return result;
  }

  BundleTemplate normalized = bundle_template;
  normalized.support_wire_pole_band_id = std::max(0, normalized.support_wire_pole_band_id);
  const CableTemplate* cable_template = state.find_cable_template(normalized.cable_template_id);
  if (normalized.grouped_support_fanout_spacing_m <= 1e-9) {
    normalized.grouped_support_fanout_spacing_m =
        (cable_template == nullptr) ? normalized.default_spacing_m : cable_template->default_grouped_support_fanout_spacing_m;
  }
  normalized.version = it->second.version;
  bool changed = false;
  const bool visual_only_change =
      normalized.cable_template_id != it->second.cable_template_id && normalized.category == it->second.category &&
      normalized.default_layer == it->second.default_layer &&
      normalized.related_pole_type_id == it->second.related_pole_type_id &&
      normalized.preserve_conductor_identity == it->second.preserve_conductor_identity &&
      normalized.count_rule == it->second.count_rule && normalized.fixed_count == it->second.fixed_count &&
      normalized.min_count == it->second.min_count && normalized.max_count == it->second.max_count &&
      normalized.default_count == it->second.default_count &&
      std::abs(normalized.default_spacing_m - it->second.default_spacing_m) <= 1e-12 &&
      std::abs(normalized.grouped_support_fanout_spacing_m - it->second.grouped_support_fanout_spacing_m) <= 1e-12 &&
      normalized.allow_mirror == it->second.allow_mirror &&
      normalized.allow_midair_node == it->second.allow_midair_node &&
      normalized.allow_midair_branch == it->second.allow_midair_branch &&
      normalized.enable_branch_down_offset == it->second.enable_branch_down_offset &&
      normalized.order_decision_policy == it->second.order_decision_policy &&
      normalized.row_layout_axis_mode == it->second.row_layout_axis_mode &&
      normalized.support_style == it->second.support_style && normalized.branch_policy == it->second.branch_policy &&
      normalized.continuity_policy == it->second.continuity_policy &&
      normalized.support_wire_pole_band_id == it->second.support_wire_pole_band_id && normalized.name == it->second.name;

  const bool detail_change = normalized.support_wire_pole_band_id != it->second.support_wire_pole_band_id;

  const bool topology_change =
      normalized.category != it->second.category || normalized.default_layer != it->second.default_layer ||
      normalized.preserve_conductor_identity != it->second.preserve_conductor_identity ||
      normalized.count_rule != it->second.count_rule || normalized.fixed_count != it->second.fixed_count ||
      normalized.min_count != it->second.min_count || normalized.max_count != it->second.max_count ||
      normalized.default_count != it->second.default_count ||
      std::abs(normalized.default_spacing_m - it->second.default_spacing_m) > 1e-12 ||
      std::abs(normalized.grouped_support_fanout_spacing_m - it->second.grouped_support_fanout_spacing_m) > 1e-12 ||
      normalized.allow_mirror != it->second.allow_mirror ||
      normalized.allow_midair_node != it->second.allow_midair_node ||
      normalized.allow_midair_branch != it->second.allow_midair_branch ||
      normalized.enable_branch_down_offset != it->second.enable_branch_down_offset ||
      normalized.order_decision_policy != it->second.order_decision_policy ||
      normalized.row_layout_axis_mode != it->second.row_layout_axis_mode ||
      normalized.support_style != it->second.support_style || normalized.branch_policy != it->second.branch_policy ||
      normalized.continuity_policy != it->second.continuity_policy;

  changed = visual_only_change || topology_change || detail_change || normalized.name != it->second.name ||
            normalized.related_pole_type_id != it->second.related_pole_type_id;
  if (!changed) {
    result.ok = true;
    result.value = false;
    return result;
  }

  normalized.version += 1;
  it->second = normalized;
  result.ok = true;
  result.value = true;

  state.authoritative_.template_dependency_state.bundles_requiring_regeneration.clear();
  state.authoritative_.template_dependency_state.sessions_requiring_regeneration.clear();

  for (const Bundle& existing_bundle : state.authoritative_.edit_state.bundles.items()) {
    if (existing_bundle.bundle_template_id != normalized.id) {
      continue;
    }
    Bundle* bundle = state.authoritative_.edit_state.bundles.find(existing_bundle.id);
    if (bundle == nullptr) {
      continue;
    }
    CoreState::add_unique_id(result.change_set.updated_ids, bundle->id);
    if (visual_only_change || detail_change) {
      auto spans_it = state.runtime_.relation_index.spans_by_bundle.find(bundle->id);
      if (spans_it == state.runtime_.relation_index.spans_by_bundle.end()) {
        continue;
      }
      for (ObjectId span_id : spans_it->second) {
        state.mark_span_dirty(span_id, DirtyBits::kDecision, true);
        CoreState::add_unique_id(result.change_set.dirty_span_ids, span_id);
        CoreState::add_unique_id(result.change_set.updated_ids, span_id);
      }
      continue;
    }
    if (topology_change) {
      bundle->regeneration_required = true;
      CoreState::add_unique_id(state.authoritative_.template_dependency_state.bundles_requiring_regeneration, bundle->id);
      auto spans_it = state.runtime_.relation_index.spans_by_bundle.find(bundle->id);
      if (spans_it == state.runtime_.relation_index.spans_by_bundle.end()) {
        continue;
      }
      for (ObjectId span_id : spans_it->second) {
        const Span* span = state.authoritative_.edit_state.spans.find(span_id);
        if (span == nullptr || span->generation.generation_session_id == 0) {
          continue;
        }
        CoreState::add_unique_id(state.authoritative_.template_dependency_state.sessions_requiring_regeneration,
                                 span->generation.generation_session_id);
      }
    }
  }
  if (topology_change) {
    for (ObjectId bundle_id : state.authoritative_.template_dependency_state.bundles_requiring_regeneration) {
      CoreState::add_unique_id(result.change_set.updated_ids, bundle_id);
    }
  }
  return result;
}

EditResult<bool> TemplateMutationService::UpdateAttachmentTemplate(CoreState& state,
                                                                   const AttachmentTemplate& attachment_template,
                                                                   bool mark_dependent_spans_dirty) {
  EditResult<bool> result;
  auto it = state.authoritative_.attachment_templates.find(attachment_template.id);
  if (it == state.authoritative_.attachment_templates.end()) {
    result.error = "attachment template not found";
    return result;
  }

  AttachmentTemplate normalized = attachment_template;
  normalized.version = it->second.version;
  const bool changed = !attachment_template_equals(normalized, it->second);
  if (!changed) {
    result.ok = true;
    result.value = false;
    return result;
  }

  normalized.version += 1;
  it->second = normalized;
  result.ok = true;
  result.value = true;

  if (!mark_dependent_spans_dirty) {
    return result;
  }
  for (const Attachment& attachment : state.authoritative_.edit_state.attachments.items()) {
    if (attachment.template_id != normalized.id) {
      continue;
    }
    state.mark_span_dirty(attachment.span_id, DirtyBits::kDecision, true);
    CoreState::add_unique_id(result.change_set.dirty_span_ids, attachment.span_id);
    CoreState::add_unique_id(result.change_set.updated_ids, attachment.span_id);
  }
  return result;
}

EditResult<bool> TemplateMutationService::ResetAllSpanReferenceLengths(CoreState& state, bool mark_all_spans_dirty) {
  EditResult<bool> result;
  bool changed = false;
  for (const Span& existing_span : state.authoritative_.edit_state.spans.items()) {
    Span* span = state.authoritative_.edit_state.spans.find(existing_span.id);
    if (span == nullptr) {
      continue;
    }
    const Port* a = state.authoritative_.edit_state.ports.find(span->port_a_id);
    const Port* b = state.authoritative_.edit_state.ports.find(span->port_b_id);
    if (a == nullptr || b == nullptr) {
      continue;
    }
    const double dx = b->world_position.x - a->world_position.x;
    const double dy = b->world_position.y - a->world_position.y;
    const double dz = b->world_position.z - a->world_position.z;
    const double length = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (std::abs(span->reference_length_m - length) <= 1e-9) {
      continue;
    }
    span->reference_length_m = length;
    changed = true;
    CoreState::add_unique_id(result.change_set.updated_ids, span->id);
  }
  if (changed && mark_all_spans_dirty) {
    for (const Span& span : state.authoritative_.edit_state.spans.items()) {
      state.mark_span_dirty(span.id, DirtyBits::kGeometryRefresh, true);
      CoreState::add_unique_id(result.change_set.dirty_span_ids, span.id);
      CoreState::add_unique_id(result.change_set.updated_ids, span.id);
    }
  }
  result.ok = true;
  result.value = changed;
  return result;
}

} // namespace wire::core::state_internal
