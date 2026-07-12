#include "../internal_services.hpp"

#include "../../collection_utils.hpp"

#include "wire/core/core_view.hpp"
#include "wire/core/coord_utils.hpp"


#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_set>

namespace wire::core::state_internal {

namespace {

using wire::core::detail::append_unique;

struct CableDecisionRegenerateScope {
  BundleTemplateId bundle_template_id = kInvalidBundleTemplateId;
  std::size_t route = 0;
  std::vector<ObjectId> edge_bundle_ids{};
};

struct BundleRegenerateScope {
  std::size_t route = 0;
  std::vector<ObjectId> edge_bundle_ids{};
};

const SavedBackboneEdge* saved_edge_by_id(const SavedBackboneGraph& graph, ObjectId edge_id) {
  const auto it = std::find_if(graph.edges.begin(), graph.edges.end(),
                               [&](const SavedBackboneEdge& edge) { return edge.edge_id == edge_id; });
  return it == graph.edges.end() ? nullptr : &*it;
}

bool collect_cable_decision_regenerate_scopes(const CoreState& state,
                                              CableTemplateId cable_template_id,
                                              std::vector<CableDecisionRegenerateScope>* scopes,
                                              std::string* error) {
  const SavedBackboneGraph& graph = state.view().backbone();
  for (const SavedBackboneEdgeBundle& edge_bundle : graph.edge_bundles) {
    const Bundle* bundle = state.view().bundles().find(edge_bundle.bundle_id);
    const auto bundle_template_it = bundle == nullptr ? state.view().bundle_templates().end()
                                                   : state.view().bundle_templates().find(bundle->bundle_template_id);
    if (bundle_template_it == state.view().bundle_templates().end() ||
        bundle_template_it->second.cable_template_id != cable_template_id) {
      continue;
    }
    const SavedBackboneEdge* edge = saved_edge_by_id(graph, edge_bundle.edge_id);
    if (edge == nullptr) {
      if (error != nullptr) {
        *error = "backbone regenerate: edge missing";
      }
      return false;
    }
    auto scope_it = std::find_if(scopes->begin(), scopes->end(),
                                 [&](const CableDecisionRegenerateScope& scope) {
                                   return scope.bundle_template_id == bundle->bundle_template_id &&
                                          scope.route == edge->route;
                                 });
    if (scope_it == scopes->end()) {
      CableDecisionRegenerateScope scope{};
      scope.bundle_template_id = bundle->bundle_template_id;
      scope.route = edge->route;
      scope.edge_bundle_ids.push_back(edge_bundle.edge_bundle_id);
      scopes->push_back(std::move(scope));
    } else {
      scope_it->edge_bundle_ids.push_back(edge_bundle.edge_bundle_id);
    }
  }
  return true;
}

bool collect_bundle_regenerate_scopes(const CoreState& state, BundleTemplateId bundle_template_id,
                                      std::vector<BundleRegenerateScope>* scopes, std::string* error) {
  const SavedBackboneGraph& graph = state.view().backbone();
  for (const SavedBackboneEdgeBundle& edge_bundle : graph.edge_bundles) {
    const Bundle* bundle = state.view().bundles().find(edge_bundle.bundle_id);
    if (bundle == nullptr || bundle->bundle_template_id != bundle_template_id) {
      continue;
    }
    const SavedBackboneEdge* edge = saved_edge_by_id(graph, edge_bundle.edge_id);
    if (edge == nullptr) {
      if (error != nullptr) {
        *error = "backbone regenerate: edge missing";
      }
      return false;
    }
    auto scope_it = std::find_if(scopes->begin(), scopes->end(),
                                 [&](const BundleRegenerateScope& scope) { return scope.route == edge->route; });
    if (scope_it == scopes->end()) {
      BundleRegenerateScope scope{};
      scope.route = edge->route;
      scope.edge_bundle_ids.push_back(edge_bundle.edge_bundle_id);
      scopes->push_back(std::move(scope));
    } else {
      scope_it->edge_bundle_ids.push_back(edge_bundle.edge_bundle_id);
    }
  }
  return true;
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

bool attachment_socket_geometry_equals(const AttachmentSocketTemplate& a, const AttachmentSocketTemplate& b) {
  const auto same_vec = [](const Vec3d& lhs, const Vec3d& rhs) {
    return std::abs(lhs.x - rhs.x) <= 1e-12 && std::abs(lhs.y - rhs.y) <= 1e-12 && std::abs(lhs.z - rhs.z) <= 1e-12;
  };
  return same_vec(a.local_position, b.local_position) && same_vec(a.tangent_dir, b.tangent_dir) &&
         same_vec(a.normal_dir, b.normal_dir) && same_vec(a.binormal_dir, b.binormal_dir);
}

bool attachment_socket_identity_equals(const AttachmentSocketTemplate& a, const AttachmentSocketTemplate& b) {
  return a.id == b.id && a.has_normal == b.has_normal && a.has_binormal == b.has_binormal && a.kind == b.kind;
}

bool attachment_internal_path_geometry_equals(const AttachmentInternalPathTemplate& a,
                                              const AttachmentInternalPathTemplate& b) {
  if (a.local_points.size() != b.local_points.size() ||
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

bool attachment_internal_path_identity_equals(const AttachmentInternalPathTemplate& a,
                                              const AttachmentInternalPathTemplate& b) {
  return a.start_socket_id == b.start_socket_id && a.end_socket_id == b.end_socket_id &&
         a.profile_kind == b.profile_kind;
}

struct AttachmentTemplateDiff {
  bool changed = false;
  bool metadata_only = false;
  bool geometry_only = false;
  bool structural = false;
};

AttachmentTemplateDiff classify_attachment_template_diff(const AttachmentTemplate& next,
                                                         const AttachmentTemplate& current) {
  AttachmentTemplateDiff diff{};
  diff.changed = !attachment_template_equals(next, current);
  if (!diff.changed) {
    return diff;
  }
  if (next.id != current.id || next.kind != current.kind ||
      next.line_interaction_mode != current.line_interaction_mode || next.sockets.size() != current.sockets.size() ||
      next.internal_paths.size() != current.internal_paths.size()) {
    diff.structural = true;
    return diff;
  }

  bool geometry_changed = false;
  for (std::size_t i = 0; i < next.sockets.size(); ++i) {
    if (!attachment_socket_identity_equals(next.sockets[i], current.sockets[i])) {
      diff.structural = true;
      return diff;
    }
    if (!attachment_socket_geometry_equals(next.sockets[i], current.sockets[i])) {
      geometry_changed = true;
    }
  }
  for (std::size_t i = 0; i < next.internal_paths.size(); ++i) {
    if (!attachment_internal_path_identity_equals(next.internal_paths[i], current.internal_paths[i])) {
      diff.structural = true;
      return diff;
    }
    if (!attachment_internal_path_geometry_equals(next.internal_paths[i], current.internal_paths[i])) {
      geometry_changed = true;
    }
  }

  diff.geometry_only = geometry_changed;
  diff.metadata_only = !geometry_changed && next.name != current.name;
  if (!diff.geometry_only && !diff.metadata_only) {
    diff.structural = true;
  }
  return diff;
}

bool cable_supplemental_path_equals(const CableSupplementalPathTemplate& a, const CableSupplementalPathTemplate& b) {
  return a.anchor_mode == b.anchor_mode && a.profile_kind == b.profile_kind && a.pole_band_id == b.pole_band_id &&
         std::abs(a.endpoint_trim_m - b.endpoint_trim_m) <= 1e-12 &&
         std::abs(a.lateral_offset_m - b.lateral_offset_m) <= 1e-12 &&
         std::abs(a.vertical_offset_m - b.vertical_offset_m) <= 1e-12;
}

bool placement_reserve_equals(const PlacementReserve& a, const PlacementReserve& b) {
  return a.reserve_id == b.reserve_id && a.pole_type_id == b.pole_type_id && a.band_id == b.band_id &&
         std::abs(a.lateral_min_m - b.lateral_min_m) <= 1e-12 &&
         std::abs(a.lateral_max_m - b.lateral_max_m) <= 1e-12 &&
         std::abs(a.height_min_m - b.height_min_m) <= 1e-12 &&
         std::abs(a.height_max_m - b.height_max_m) <= 1e-12;
}

bool population_rule_equals(const CablePopulationRule& a, const CablePopulationRule& b) {
  if (a.rule_id != b.rule_id || a.explicit_seed != b.explicit_seed || a.priority != b.priority ||
      a.min_extra_count != b.min_extra_count || a.max_extra_count != b.max_extra_count ||
      std::abs(a.min_spacing_m - b.min_spacing_m) > 1e-12 ||
      std::abs(a.lateral_min_m - b.lateral_min_m) > 1e-12 ||
      std::abs(a.lateral_max_m - b.lateral_max_m) > 1e-12 ||
      std::abs(a.height_min_m - b.height_min_m) > 1e-12 ||
      std::abs(a.height_max_m - b.height_max_m) > 1e-12 ||
      std::abs(a.randomness - b.randomness) > 1e-12 || a.reserves.size() != b.reserves.size()) {
    return false;
  }
  for (std::size_t i = 0; i < a.reserves.size(); ++i) {
    if (!placement_reserve_equals(a.reserves[i], b.reserves[i])) {
      return false;
    }
  }
  return true;
}

bool population_rules_equal(const std::vector<CablePopulationRule>& a,
                            const std::vector<CablePopulationRule>& b) {
  if (a.size() != b.size()) {
    return false;
  }
  for (std::size_t i = 0; i < a.size(); ++i) {
    if (!population_rule_equals(a[i], b[i])) {
      return false;
    }
  }
  return true;
}

enum BundleTemplateChange : std::uint32_t {
  kMetadata = 1u << 0,
  kDefinition = 1u << 1,
  kDraw = 1u << 2,
  kDetail = 1u << 3,
  kCount = 1u << 4,
  kTopology = 1u << 5,
};

std::uint32_t classify_bundle_template_changes(const BundleTemplate& before,
                                                const BundleTemplate& after) {
  std::uint32_t changes = 0;
  if (before.name != after.name) {
    changes |= kMetadata;
  }
  if (before.related_pole_type_id != after.related_pole_type_id) {
    changes |= kDefinition;
  }
  if (before.cable_template_id != after.cable_template_id) {
    changes |= kDraw;
  }
  if (before.support_wire_pole_band_id != after.support_wire_pole_band_id ||
      !population_rules_equal(before.population_rules, after.population_rules)) {
    changes |= kDetail;
  }
  if (before.count_rule != after.count_rule || before.fixed_count != after.fixed_count ||
      before.min_count != after.min_count || before.max_count != after.max_count ||
      before.default_count != after.default_count) {
    changes |= kCount;
  }
  if (before.category != after.category || before.default_layer != after.default_layer ||
      before.preserve_conductor_identity != after.preserve_conductor_identity ||
      std::abs(before.default_spacing_m - after.default_spacing_m) > 1e-12 ||
      std::abs(before.grouped_support_fanout_spacing_m - after.grouped_support_fanout_spacing_m) > 1e-12 ||
      before.allow_mirror != after.allow_mirror || before.allow_midair_node != after.allow_midair_node ||
      before.allow_midair_branch != after.allow_midair_branch ||
      before.enable_branch_down_offset != after.enable_branch_down_offset ||
      std::abs(before.branch_endpoint_offset_m - after.branch_endpoint_offset_m) > 1e-12 ||
      before.order_decision_policy != after.order_decision_policy ||
      before.row_layout_axis_mode != after.row_layout_axis_mode || before.support_style != after.support_style ||
      before.branch_policy != after.branch_policy || before.continuity_policy != after.continuity_policy) {
    changes |= kTopology;
  }
  return changes;
}

bool valid_bundle_count_policy(const BundleTemplate& bundle_template, std::string* error) {
  if (bundle_template.count_rule == BundleCountRuleKind::kFixed) {
    if (bundle_template.fixed_count > 0) {
      return true;
    }
    *error = "bundle fixed count must be positive";
    return false;
  }
  if (bundle_template.min_count <= 0 || bundle_template.max_count < bundle_template.min_count ||
      bundle_template.default_count < bundle_template.min_count ||
      bundle_template.default_count > bundle_template.max_count) {
    *error = "bundle count range is invalid";
    return false;
  }
  return true;
}

bool bundle_count_matches_policy(const Bundle& bundle, const BundleTemplate& bundle_template) {
  if (bundle_template.count_rule == BundleCountRuleKind::kFixed) {
    return bundle.conductor_count == bundle_template.fixed_count;
  }
  return bundle.conductor_count >= bundle_template.min_count && bundle.conductor_count <= bundle_template.max_count;
}

bool validate_population_rules(const CoreState& state, const std::vector<CablePopulationRule>& rules,
                               std::string* error) {
  std::unordered_set<CableSectionRuleId> rule_ids{};
  for (const CablePopulationRule& rule : rules) {
    if (rule.rule_id == 0 || !rule_ids.insert(rule.rule_id).second) {
      *error = "cable population: rule ids must be nonzero and unique per bundle template";
      return false;
    }
    if (rule.min_extra_count < 0 || rule.max_extra_count < rule.min_extra_count ||
        !std::isfinite(rule.min_spacing_m) || rule.min_spacing_m < 0.0 ||
        !std::isfinite(rule.lateral_min_m) || !std::isfinite(rule.lateral_max_m) ||
        rule.lateral_min_m > rule.lateral_max_m || !std::isfinite(rule.height_min_m) ||
        !std::isfinite(rule.height_max_m) || rule.height_min_m > rule.height_max_m ||
        !std::isfinite(rule.randomness) || rule.randomness < 0.0 || rule.randomness > 1.0) {
      *error = "cable population: invalid rule range";
      return false;
    }
    std::unordered_set<PlacementReserveId> reserve_ids{};
    for (const PlacementReserve& reserve : rule.reserves) {
      if (reserve.reserve_id == 0 || !reserve_ids.insert(reserve.reserve_id).second ||
          !std::isfinite(reserve.lateral_min_m) || !std::isfinite(reserve.lateral_max_m) ||
          reserve.lateral_min_m > reserve.lateral_max_m || !std::isfinite(reserve.height_min_m) ||
          !std::isfinite(reserve.height_max_m) || reserve.height_min_m > reserve.height_max_m) {
        *error = "cable population: invalid placement reserve";
        return false;
      }
      const auto pole_type_it = state.view().pole_types().find(reserve.pole_type_id);
      if (pole_type_it == state.view().pole_types().end()) {
        *error = "cable population: reserve references unknown pole type";
        return false;
      }
      const std::size_t matching_bands =
          static_cast<std::size_t>(std::count_if(pole_type_it->second.port_bands.begin(),
                                                pole_type_it->second.port_bands.end(),
                                                [&](const PortPlacementBand& band) {
                                                  return band.band_id == reserve.band_id;
                                                }));
      if (matching_bands != 1) {
        *error = "cable population: reserve band identity must resolve exactly once";
        return false;
      }
    }
  }
  return true;
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
  if (decision_change && !ordered_target_span_ids.empty()) {
    for (ObjectId span_id : ordered_target_span_ids) {
      if (state.runtime_.backbone_index.span_edge_bundle.find(span_id) ==
          state.runtime_.backbone_index.span_edge_bundle.end()) {
        result.error = "backbone unsupported: cable decision changes require regeneration";
        return result;
      }
    }

    std::vector<CableDecisionRegenerateScope> scopes{};
    if (!collect_cable_decision_regenerate_scopes(state, normalized.id, &scopes, &result.error)) {
      return result;
    }
    if (scopes.empty()) {
      result.error = "backbone unsupported: cable decision changes require regeneration";
      return result;
    }

    normalized.version += 1;
    CoreState trial = state;
    ChangeSet regenerated_changes{};
    for (const CableDecisionRegenerateScope& scope : scopes) {
      const auto bundle_template_it = trial.authoritative_.bundle_templates.find(scope.bundle_template_id);
      if (bundle_template_it == trial.authoritative_.bundle_templates.end()) {
        result.error = "bundle template not found";
        return result;
      }
      const BundleTemplate previous = bundle_template_it->second;
      auto regenerated = trial.regenerate_backbone_edge_bundles(scope.bundle_template_id, previous, previous,
                                                                &regenerated_changes, &normalized,
                                                                &scope.edge_bundle_ids);
      if (!regenerated.ok) {
        result.error = regenerated.error;
        return result;
      }
    }

    state.identity_ = trial.identity_;
    state.authoritative_ = trial.authoritative_;
    state.runtime_ = trial.runtime_;
    state.debug_ = trial.debug_;
    result.ok = true;
    result.value = true;
    result.change_set = std::move(regenerated_changes);
    return result;
  }
  std::vector<UpdatePlan> plans{};
  if (geometry_change || render_change) {
    const UpdateKind kind = geometry_change ? UpdateKind::kReshape : UpdateKind::kRedraw;
    plans.reserve(ordered_target_span_ids.size());
    for (ObjectId span_id : ordered_target_span_ids) {
      auto plan = state.make_update_plan({kind, UpdateTargetKind::kSpan, span_id});
      if (!plan.ok) {
        result.error = plan.error;
        return result;
      }
      plans.push_back(std::move(plan.value));
    }
  }

  normalized.version += 1;
  it->second = normalized;
  result.ok = true;
  result.value = true;

  for (const UpdatePlan& plan : plans) {
    auto updated = state.execute_update_plan(plan);
    if (!updated.ok) {
      result.ok = false;
      result.error = updated.error;
      return result;
    }
  }
  for (ObjectId span_id : ordered_target_span_ids) {
    CoreState::add_unique_id(result.change_set.updated_ids, span_id);
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
  for (CablePopulationRule& rule : normalized.population_rules) {
    if (rule.explicit_seed == 0) {
      rule.explicit_seed = 1;
    }
  }
  if (!validate_population_rules(state, normalized.population_rules, &result.error)) {
    return result;
  }
  if (!std::isfinite(normalized.branch_endpoint_offset_m)) {
    result.error = "bundle branch endpoint offset must be finite";
    return result;
  }
  const CableTemplate* cable_template = state.find_cable_template(normalized.cable_template_id);
  if (normalized.grouped_support_fanout_spacing_m <= 1e-9) {
    normalized.grouped_support_fanout_spacing_m =
        (cable_template == nullptr) ? normalized.default_spacing_m : cable_template->default_grouped_support_fanout_spacing_m;
  }
  if (!valid_bundle_count_policy(normalized, &result.error)) {
    return result;
  }
  const BundleTemplate previous = it->second;
  normalized.version = previous.version;
  const std::uint32_t changes = classify_bundle_template_changes(previous, normalized);
  if (changes == 0) {
    result.ok = true;
    result.value = false;
    return result;
  }

  const bool count_change = (changes & kCount) != 0;
  const bool range_count_policy_change =
      count_change && (previous.count_rule == BundleCountRuleKind::kRange ||
                       normalized.count_rule == BundleCountRuleKind::kRange);
  if (range_count_policy_change) {
    for (const Bundle& existing_bundle : state.authoritative_.edit_state.bundles.items()) {
      if (existing_bundle.bundle_template_id != normalized.id) {
        continue;
      }
      if (!bundle_count_matches_policy(existing_bundle, normalized)) {
        result.error = "backbone unsupported: bundle count policy excludes bundle " +
                       std::to_string(existing_bundle.id);
        return result;
      }
    }
  }

  const bool fixed_count_change =
      count_change && previous.count_rule == BundleCountRuleKind::kFixed &&
      normalized.count_rule == BundleCountRuleKind::kFixed && normalized.fixed_count > 0 &&
      normalized.fixed_count != previous.fixed_count;
  constexpr std::uint32_t kRegenerateChangeMask = kMetadata | kDefinition | kCount;
  const bool topology_regenerate = (changes & kTopology) != 0;

  std::vector<ObjectId> affected_spans{};
  std::vector<ObjectId> affected_bundle_ids{};
  for (const Bundle& existing_bundle : state.authoritative_.edit_state.bundles.items()) {
    if (existing_bundle.bundle_template_id != normalized.id) {
      continue;
    }
    affected_bundle_ids.push_back(existing_bundle.id);
    const auto spans_it = state.runtime_.relation_index.spans_by_bundle.find(existing_bundle.id);
    if (spans_it != state.runtime_.relation_index.spans_by_bundle.end()) {
      append_unique(affected_spans, spans_it->second);
    }
  }
  if (topology_regenerate) {
    for (ObjectId span_id : affected_spans) {
      if (state.runtime_.backbone_index.span_edge_bundle.find(span_id) ==
          state.runtime_.backbone_index.span_edge_bundle.end()) {
        result.error = "backbone unsupported: bundle policy changes require regeneration";
        return result;
      }
    }
    if (!affected_bundle_ids.empty()) {
      std::vector<BundleRegenerateScope> scopes{};
      if (!collect_bundle_regenerate_scopes(state, normalized.id, &scopes, &result.error)) {
        return result;
      }
      if (scopes.empty()) {
        result.error = "backbone unsupported: bundle policy changes require regeneration";
        return result;
      }

      normalized.version += 1;
      CoreState trial = state;
      ChangeSet regenerated_changes{};
      for (const BundleRegenerateScope& scope : scopes) {
        auto regenerated = trial.regenerate_backbone_edge_bundles(
            normalized.id, previous, normalized, &regenerated_changes, nullptr, &scope.edge_bundle_ids, nullptr,
            (changes & kTopology) != 0 ? CoreState::BackboneRegenerateCause::kBundleTopology
                                        : CoreState::BackboneRegenerateCause::kBundleCount);
        if (!regenerated.ok) {
          result.error = regenerated.error;
          return result;
        }
      }
      trial.authoritative_.bundle_templates[normalized.id] = normalized;
      for (ObjectId bundle_id : affected_bundle_ids) {
        CoreState::add_unique_id(regenerated_changes.updated_ids, bundle_id);
      }
      state.identity_ = trial.identity_;
      state.authoritative_ = trial.authoritative_;
      state.runtime_ = trial.runtime_;
      state.debug_ = trial.debug_;
      result.ok = true;
      result.value = true;
      result.change_set = std::move(regenerated_changes);
      return result;
    }
  }
  std::vector<UpdatePlan> plans{};
  if ((changes & (kDraw | kDetail)) != 0) {
    const UpdateKind kind = (changes & kDetail) != 0 ? UpdateKind::kReshape : UpdateKind::kRedraw;
    plans.reserve(affected_spans.size());
    for (ObjectId span_id : affected_spans) {
      auto plan = state.make_update_plan({kind, UpdateTargetKind::kSpan, span_id});
      if (!plan.ok) {
        result.error = plan.error;
        return result;
      }
      plans.push_back(std::move(plan.value));
    }
  }

  normalized.version += 1;
  if (!affected_bundle_ids.empty() && count_change && !range_count_policy_change &&
      (!fixed_count_change || (changes & ~kRegenerateChangeMask) != 0)) {
    result.error = "backbone unsupported: bundle count changes require fixed count regeneration";
    return result;
  }
  if (fixed_count_change) {
    auto regenerated = state.regenerate_backbone_edge_bundles(normalized.id, previous, normalized,
                                                              &result.change_set);
    if (!regenerated.ok) {
      result.error = regenerated.error;
      return result;
    }
  }
  auto updated_template_it = state.authoritative_.bundle_templates.find(normalized.id);
  if (updated_template_it == state.authoritative_.bundle_templates.end()) {
    result.error = "bundle template not found";
    return result;
  }
  updated_template_it->second = normalized;
  result.ok = true;
  result.value = true;
  for (const Bundle& existing_bundle : state.authoritative_.edit_state.bundles.items()) {
    if (existing_bundle.bundle_template_id == normalized.id) {
      CoreState::add_unique_id(result.change_set.updated_ids, existing_bundle.id);
    }
  }
  for (const UpdatePlan& plan : plans) {
    auto updated = state.execute_update_plan(plan);
    if (!updated.ok) {
      result.ok = false;
      result.error = updated.error;
      return result;
    }
  }
  const bool range_policy_only =
      range_count_policy_change && (changes & ~(kMetadata | kDefinition | kCount)) == 0;
  if (!range_policy_only) {
    append_unique(result.change_set.updated_ids, affected_spans);
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
  const AttachmentTemplateDiff diff = classify_attachment_template_diff(normalized, it->second);
  if (!diff.changed) {
    result.ok = true;
    result.value = false;
    return result;
  }

  std::vector<ObjectId> affected_spans{};
  for (const Attachment& attachment : state.authoritative_.edit_state.attachments.items()) {
    if (attachment.template_id == normalized.id) {
      if (diff.structural) {
        result.error = "backbone unsupported: attachment template structural changes require regeneration";
        return result;
      }
      CoreState::add_unique_id(affected_spans, attachment.span_id);
    }
  }

  std::vector<UpdatePlan> plans{};
  if (diff.geometry_only) {
    plans.reserve(affected_spans.size());
    for (ObjectId span_id : affected_spans) {
      auto plan = state.make_update_plan({UpdateKind::kReshape, UpdateTargetKind::kSpan, span_id});
      if (!plan.ok) {
        result.error = plan.error;
        return result;
      }
      plans.push_back(std::move(plan.value));
    }
  }

  normalized.version += 1;
  it->second = normalized;
  result.ok = true;
  result.value = true;
  for (const UpdatePlan& plan : plans) {
    auto updated = state.execute_update_plan(plan);
    if (!updated.ok) {
      result.ok = false;
      result.error = updated.error;
      return result;
    }
  }
  append_unique(result.change_set.updated_ids, affected_spans);

  return result;
}

EditResult<bool> TemplateMutationService::ResetAllSpanReferenceLengths(CoreState& state, bool mark_all_spans_dirty) {
  EditResult<bool> result;
  auto plan = state.make_update_plan({UpdateKind::kReshape, UpdateTargetKind::kAllSpans, kInvalidObjectId});
  if (!plan.ok) {
    result.error = plan.error;
    return result;
  }
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
    const double length = Length(b->world_position - a->world_position);
    if (std::abs(span->reference_length_m - length) <= 1e-9) {
      continue;
    }
    span->reference_length_m = length;
    changed = true;
    CoreState::add_unique_id(result.change_set.updated_ids, span->id);
  }
  if (changed) {
    const auto updated = state.execute_update_plan(plan.value);
    if (!updated.ok) {
      result.error = updated.error;
      return result;
    }
    append_unique(result.change_set.updated_ids, plan.value.affected.spans);
  }
  result.ok = true;
  result.value = changed;
  return result;
}

} // namespace wire::core::state_internal
