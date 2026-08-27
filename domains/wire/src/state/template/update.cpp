#include "../internal_services.hpp"

#include "../../collection_utils.hpp"

#include "city/wire/core_view.hpp"
#include "city/wire/coord_utils.hpp"
#include "city/wire/support/numeric_tolerances.hpp"


#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <unordered_map>
#include <unordered_set>

namespace city::wire::state_internal {

namespace {

using city::wire::detail::append_unique;

struct CableDecisionRegenerateScope {
  BundleTemplateId bundle_template_id = kInvalidBundleTemplateId;
  ObjectId bundle_id = kInvalidObjectId;
  std::vector<ObjectId> edge_bundle_ids{};
};

struct BundleRegenerateScope {
  ObjectId bundle_id = kInvalidObjectId;
  std::vector<ObjectId> edge_bundle_ids{};
};

const SavedBackboneEdgeBundle* saved_edge_bundle_by_id(const SavedBackboneGraph& graph, ObjectId edge_bundle_id) {
  const auto it = std::find_if(graph.edge_bundles.begin(), graph.edge_bundles.end(),
                               [&](const SavedBackboneEdgeBundle& edge_bundle) {
                                 return edge_bundle.edge_bundle_id == edge_bundle_id;
                               });
  return it == graph.edge_bundles.end() ? nullptr : &*it;
}

std::unordered_map<ObjectId, std::vector<ObjectId>> row_continuity_neighbors(const SavedBackboneGraph& graph) {
  std::unordered_map<ObjectId, std::vector<ObjectId>> neighbors{};
  auto add_neighbor = [&](ObjectId a, ObjectId b) {
    if (a == kInvalidObjectId || b == kInvalidObjectId || a == b) return;
    std::vector<ObjectId>& values = neighbors[a];
    if (std::find(values.begin(), values.end(), b) == values.end()) {
      values.push_back(b);
    }
  };
  for (const SavedBackboneRowContinuity& continuity : graph.row_continuities) {
    add_neighbor(continuity.a.edge_bundle_id, continuity.b.edge_bundle_id);
    add_neighbor(continuity.b.edge_bundle_id, continuity.a.edge_bundle_id);
  }
  return neighbors;
}

std::vector<ObjectId> edge_bundle_component(
    const SavedBackboneGraph& graph,
    const std::unordered_map<ObjectId, std::vector<ObjectId>>& neighbors,
    ObjectId seed_edge_bundle_id,
    ObjectId bundle_id,
    const std::function<bool(const SavedBackboneEdgeBundle&)>& accepts) {
  std::vector<ObjectId> component{};
  std::vector<ObjectId> pending{seed_edge_bundle_id};
  for (std::size_t i = 0; i < pending.size(); ++i) {
    const ObjectId current_id = pending[i];
    if (std::find(component.begin(), component.end(), current_id) != component.end()) {
      continue;
    }
    const SavedBackboneEdgeBundle* current = saved_edge_bundle_by_id(graph, current_id);
    if (current == nullptr || current->bundle_id != bundle_id || !accepts(*current)) {
      continue;
    }
    component.push_back(current_id);
    const auto neighbors_it = neighbors.find(current_id);
    if (neighbors_it == neighbors.end()) {
      continue;
    }
    for (ObjectId neighbor_id : neighbors_it->second) {
      if (std::find(pending.begin(), pending.end(), neighbor_id) == pending.end()) {
        pending.push_back(neighbor_id);
      }
    }
  }
  return component;
}

bool collect_cable_decision_regenerate_scopes(const CoreState& state,
                                              CableTemplateId cable_template_id,
                                              std::vector<CableDecisionRegenerateScope>* scopes,
                                              std::string* error) {
  const SavedBackboneGraph& graph = state.view().backbone();
  const std::unordered_map<ObjectId, std::vector<ObjectId>> neighbors = row_continuity_neighbors(graph);
  std::vector<ObjectId> covered{};
  for (const SavedBackboneEdgeBundle& edge_bundle : graph.edge_bundles) {
    if (std::find(covered.begin(), covered.end(), edge_bundle.edge_bundle_id) != covered.end()) {
      continue;
    }
    const Bundle* bundle = state.view().bundles().find(edge_bundle.bundle_id);
    const auto bundle_template_it = bundle == nullptr ? state.view().bundle_templates().end()
                                                   : state.view().bundle_templates().find(bundle->bundle_template_id);
    if (bundle_template_it == state.view().bundle_templates().end() ||
        bundle_template_it->second.cable_template_id != cable_template_id) {
      continue;
    }
    CableDecisionRegenerateScope scope{};
    scope.bundle_template_id = bundle->bundle_template_id;
    scope.bundle_id = edge_bundle.bundle_id;
    scope.edge_bundle_ids = edge_bundle_component(
        graph, neighbors, edge_bundle.edge_bundle_id, edge_bundle.bundle_id,
        [&](const SavedBackboneEdgeBundle& candidate) {
          const Bundle* candidate_bundle = state.view().bundles().find(candidate.bundle_id);
          const auto candidate_template_it =
              candidate_bundle == nullptr ? state.view().bundle_templates().end()
                                          : state.view().bundle_templates().find(candidate_bundle->bundle_template_id);
          return candidate_template_it != state.view().bundle_templates().end() &&
                 candidate_bundle->bundle_template_id == scope.bundle_template_id &&
                 candidate_template_it->second.cable_template_id == cable_template_id;
        });
    if (scope.edge_bundle_ids.empty()) {
      if (error != nullptr) {
        *error = "backbone unsupported: backbone regenerate: cable decision scope has no edge bundles";
      }
      return false;
    }
    for (ObjectId edge_bundle_id : scope.edge_bundle_ids) {
      if (std::find(covered.begin(), covered.end(), edge_bundle_id) == covered.end()) {
        covered.push_back(edge_bundle_id);
      }
    }
    scopes->push_back(std::move(scope));
  }
  return true;
}

bool collect_bundle_regenerate_scopes(const CoreState& state, BundleTemplateId bundle_template_id,
                                      std::vector<BundleRegenerateScope>* scopes, std::string* error,
                                      ObjectId exact_bundle_id = kInvalidObjectId) {
  const SavedBackboneGraph& graph = state.view().backbone();
  const std::unordered_map<ObjectId, std::vector<ObjectId>> neighbors = row_continuity_neighbors(graph);
  std::vector<ObjectId> covered{};
  for (const SavedBackboneEdgeBundle& edge_bundle : graph.edge_bundles) {
    if (std::find(covered.begin(), covered.end(), edge_bundle.edge_bundle_id) != covered.end()) {
      continue;
    }
    const Bundle* bundle = state.view().bundles().find(edge_bundle.bundle_id);
    if (bundle == nullptr || bundle->bundle_template_id != bundle_template_id ||
        (exact_bundle_id != kInvalidObjectId && bundle->id != exact_bundle_id)) {
      continue;
    }
    BundleRegenerateScope scope{};
    scope.bundle_id = edge_bundle.bundle_id;
    scope.edge_bundle_ids = edge_bundle_component(
        graph, neighbors, edge_bundle.edge_bundle_id, edge_bundle.bundle_id,
        [&](const SavedBackboneEdgeBundle& candidate) {
          const Bundle* candidate_bundle = state.view().bundles().find(candidate.bundle_id);
          return candidate_bundle != nullptr && candidate_bundle->bundle_template_id == bundle_template_id;
        });
    if (scope.edge_bundle_ids.empty()) {
      if (error != nullptr) {
        *error = "backbone unsupported: backbone regenerate: bundle scope has no edge bundles";
      }
      return false;
    }
    for (ObjectId edge_bundle_id : scope.edge_bundle_ids) {
      if (std::find(covered.begin(), covered.end(), edge_bundle_id) == covered.end()) {
        covered.push_back(edge_bundle_id);
      }
    }
    scopes->push_back(std::move(scope));
  }
  return true;
}

bool attachment_socket_equals(const AttachmentSocketTemplate& a, const AttachmentSocketTemplate& b) {
  const auto same_vec = [](const Vec3d& lhs, const Vec3d& rhs) {
    return std::abs(lhs.x - rhs.x) <= kStrictLengthToleranceM && std::abs(lhs.y - rhs.y) <= kStrictLengthToleranceM && std::abs(lhs.z - rhs.z) <= kStrictLengthToleranceM;
  };
  return a.id == b.id && same_vec(a.local_position, b.local_position) && same_vec(a.tangent_dir, b.tangent_dir) &&
         a.has_normal == b.has_normal && same_vec(a.normal_dir, b.normal_dir) &&
         a.has_binormal == b.has_binormal && same_vec(a.binormal_dir, b.binormal_dir) && a.kind == b.kind;
}

bool attachment_internal_path_equals(const AttachmentInternalPathTemplate& a, const AttachmentInternalPathTemplate& b) {
  if (a.start_socket_id != b.start_socket_id || a.end_socket_id != b.end_socket_id ||
      a.profile_kind != b.profile_kind || a.local_points.size() != b.local_points.size() ||
      std::abs(a.coil_radius_m - b.coil_radius_m) > kStrictLengthToleranceM || a.coil_turn_count != b.coil_turn_count ||
      a.coil_samples_per_turn != b.coil_samples_per_turn) {
    return false;
  }
  for (std::size_t i = 0; i < a.local_points.size(); ++i) {
    const Vec3d& lhs = a.local_points[i];
    const Vec3d& rhs = b.local_points[i];
    if (std::abs(lhs.x - rhs.x) > kStrictLengthToleranceM || std::abs(lhs.y - rhs.y) > kStrictLengthToleranceM || std::abs(lhs.z - rhs.z) > kStrictLengthToleranceM) {
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
    return std::abs(lhs.x - rhs.x) <= kStrictLengthToleranceM && std::abs(lhs.y - rhs.y) <= kStrictLengthToleranceM && std::abs(lhs.z - rhs.z) <= kStrictLengthToleranceM;
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
      std::abs(a.coil_radius_m - b.coil_radius_m) > kStrictLengthToleranceM || a.coil_turn_count != b.coil_turn_count ||
      a.coil_samples_per_turn != b.coil_samples_per_turn) {
    return false;
  }
  for (std::size_t i = 0; i < a.local_points.size(); ++i) {
    const Vec3d& lhs = a.local_points[i];
    const Vec3d& rhs = b.local_points[i];
    if (std::abs(lhs.x - rhs.x) > kStrictLengthToleranceM || std::abs(lhs.y - rhs.y) > kStrictLengthToleranceM || std::abs(lhs.z - rhs.z) > kStrictLengthToleranceM) {
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
         std::abs(a.endpoint_trim_m - b.endpoint_trim_m) <= kStrictLengthToleranceM &&
         std::abs(a.lateral_offset_m - b.lateral_offset_m) <= kStrictLengthToleranceM &&
         std::abs(a.vertical_offset_m - b.vertical_offset_m) <= kStrictLengthToleranceM;
}

bool placement_reserve_equals(const PlacementReserve& a, const PlacementReserve& b) {
  return a.reserve_id == b.reserve_id && a.pole_type_id == b.pole_type_id && a.band_id == b.band_id &&
         std::abs(a.lateral_min_m - b.lateral_min_m) <= kStrictLengthToleranceM &&
         std::abs(a.lateral_max_m - b.lateral_max_m) <= kStrictLengthToleranceM &&
         std::abs(a.height_min_m - b.height_min_m) <= kStrictLengthToleranceM &&
         std::abs(a.height_max_m - b.height_max_m) <= kStrictLengthToleranceM;
}

bool population_rule_equals(const CablePopulationRule& a, const CablePopulationRule& b) {
  if (a.rule_id != b.rule_id || a.explicit_seed != b.explicit_seed || a.priority != b.priority ||
      a.min_extra_count != b.min_extra_count || a.max_extra_count != b.max_extra_count ||
      std::abs(a.min_spacing_m - b.min_spacing_m) > kStrictLengthToleranceM ||
      std::abs(a.lateral_min_m - b.lateral_min_m) > kStrictLengthToleranceM ||
      std::abs(a.lateral_max_m - b.lateral_max_m) > kStrictLengthToleranceM ||
      std::abs(a.height_min_m - b.height_min_m) > kStrictLengthToleranceM ||
      std::abs(a.height_max_m - b.height_max_m) > kStrictLengthToleranceM ||
      std::abs(a.randomness - b.randomness) > kStrictLengthToleranceM || a.reserves.size() != b.reserves.size()) {
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

bool span_visual_assembly_equals(const SpanVisualAssemblyTemplate& a, const SpanVisualAssemblyTemplate& b) {
  return a.support_path_enabled == b.support_path_enabled &&
         a.helix_enabled == b.helix_enabled && a.helix_radius_m == b.helix_radius_m &&
         a.helix_clearance_m == b.helix_clearance_m && a.helix_turns_per_meter == b.helix_turns_per_meter &&
         a.helix_samples_per_turn == b.helix_samples_per_turn && a.endpoint_trim_m == b.endpoint_trim_m &&
         a.visual_member_count_min == b.visual_member_count_min &&
         a.visual_member_count_max == b.visual_member_count_max &&
         a.visual_member_spacing_m == b.visual_member_spacing_m &&
         a.center_wander_amplitude_m == b.center_wander_amplitude_m &&
         a.center_wander_wavelength_m == b.center_wander_wavelength_m &&
         a.member_wander_ratio == b.member_wander_ratio &&
         a.member_wander_wavelength_m == b.member_wander_wavelength_m &&
         a.member_wander_phase_bias == b.member_wander_phase_bias &&
         a.member_twist_turns_per_meter == b.member_twist_turns_per_meter &&
         a.member_twist_phase == b.member_twist_phase;
}

enum BundleTemplateChange : std::uint8_t {
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
      before.row_fixture_assembly_id != after.row_fixture_assembly_id ||
      before.endpoint_fixture_assembly_id != after.endpoint_fixture_assembly_id ||
      !population_rules_equal(before.population_rules, after.population_rules) ||
      !span_visual_assembly_equals(before.span_visual_assembly, after.span_visual_assembly)) {
    changes |= kDetail;
  }
  if (before.count_rule != after.count_rule || before.fixed_count != after.fixed_count ||
      before.min_count != after.min_count || before.max_count != after.max_count ||
      before.default_count != after.default_count) {
    changes |= kCount;
  }
  if (before.category != after.category || before.default_layer != after.default_layer ||
      before.preserve_conductor_identity != after.preserve_conductor_identity ||
      std::abs(before.default_spacing_m - after.default_spacing_m) > kStrictLengthToleranceM ||
      std::abs(before.grouped_support_fanout_spacing_m - after.grouped_support_fanout_spacing_m) > kStrictLengthToleranceM ||
      before.allow_mirror != after.allow_mirror || before.allow_midair_node != after.allow_midair_node ||
      before.allow_midair_branch != after.allow_midair_branch ||
      before.enable_branch_down_offset != after.enable_branch_down_offset ||
      std::abs(before.branch_endpoint_offset_m - after.branch_endpoint_offset_m) > kStrictLengthToleranceM ||
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
    *error = "core invalid input: bundle fixed count must be positive";
    return false;
  }
  if (bundle_template.min_count <= 0 || bundle_template.max_count < bundle_template.min_count ||
      bundle_template.default_count < bundle_template.min_count ||
      bundle_template.default_count > bundle_template.max_count) {
    *error = "core invalid input: bundle count range is invalid";
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
      *error = "cable population invalid input: cable population: rule ids must be nonzero and unique per bundle template";
      return false;
    }
    if (rule.min_extra_count < 0 || rule.max_extra_count < rule.min_extra_count ||
        !std::isfinite(rule.min_spacing_m) || rule.min_spacing_m < 0.0 ||
        !std::isfinite(rule.lateral_min_m) || !std::isfinite(rule.lateral_max_m) ||
        rule.lateral_min_m > rule.lateral_max_m || !std::isfinite(rule.height_min_m) ||
        !std::isfinite(rule.height_max_m) || rule.height_min_m > rule.height_max_m ||
        !std::isfinite(rule.randomness) || rule.randomness < 0.0 || rule.randomness > 1.0) {
      *error = "cable population invalid input: cable population: invalid rule range";
      return false;
    }
    std::unordered_set<PlacementReserveId> reserve_ids{};
    for (const PlacementReserve& reserve : rule.reserves) {
      if (reserve.reserve_id == 0 || !reserve_ids.insert(reserve.reserve_id).second ||
          !std::isfinite(reserve.lateral_min_m) || !std::isfinite(reserve.lateral_max_m) ||
          reserve.lateral_min_m > reserve.lateral_max_m || !std::isfinite(reserve.height_min_m) ||
          !std::isfinite(reserve.height_max_m) || reserve.height_min_m > reserve.height_max_m) {
        *error = "cable population invalid input: cable population: invalid placement reserve";
        return false;
      }
      const auto pole_type_it = state.view().pole_types().find(reserve.pole_type_id);
      if (pole_type_it == state.view().pole_types().end()) {
        *error = "cable population invalid input: cable population: reserve references unknown pole type";
        return false;
      }
      const std::size_t matching_bands =
          static_cast<std::size_t>(std::count_if(pole_type_it->second.port_bands.begin(),
                                                pole_type_it->second.port_bands.end(),
                                                [&](const PortPlacementBand& band) {
                                                  return band.band_id == reserve.band_id;
                                                }));
      if (matching_bands != 1) {
        *error = "cable population invalid input: cable population: reserve band identity must resolve exactly once";
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
    result.error = "core invalid input: cable template not found";
    return result;
  }

  CableTemplate normalized = cable_template;
  normalized.outer_diameter_m = std::max(0.0, normalized.outer_diameter_m);
  normalized.default_grouped_support_fanout_spacing_m = std::max(0.0, normalized.default_grouped_support_fanout_spacing_m);
  normalized.bend_stiffness = std::max(0.0, normalized.bend_stiffness);
  normalized.min_bend_radius_m = std::max(0.0, normalized.min_bend_radius_m);
  normalized.sag_factor = std::max(0.0, normalized.sag_factor);
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
      std::abs(normalized.outer_diameter_m - it->second.outer_diameter_m) > kStrictLengthToleranceM ||
      std::abs(normalized.default_grouped_support_fanout_spacing_m -
               it->second.default_grouped_support_fanout_spacing_m) > kStrictLengthToleranceM ||
      std::abs(normalized.bend_stiffness - it->second.bend_stiffness) > kStrictLengthToleranceM ||
      std::abs(normalized.min_bend_radius_m - it->second.min_bend_radius_m) > kStrictLengthToleranceM ||
      std::abs(normalized.sag_factor - it->second.sag_factor) > kStrictLengthToleranceM ||
      supplemental_paths_changed;
  const bool render_change =
      normalized.material_style != it->second.material_style || normalized.color_rgba != it->second.color_rgba ||
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
  if (normalized.id == kDefaultSupportWireCableTemplateId) {
    for (const auto& [bundle_id, span_ids] : state.runtime_.relation_index.spans_by_bundle) {
      const Bundle* bundle = state.authoritative_.edit_state.bundles.find(bundle_id);
      if (bundle == nullptr) {
        continue;
      }
      const BundleTemplate* bundle_template = state.find_bundle_template(bundle->bundle_template_id);
      if (bundle_template == nullptr || !bundle_template->span_visual_assembly.support_path_enabled) {
        continue;
      }
      for (ObjectId span_id : span_ids) {
        if (target_span_ids.insert(span_id).second) {
          ordered_target_span_ids.push_back(span_id);
        }
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
        result.error = "core invalid input: bundle template not found";
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

EditResult<bool> TemplateMutationService::UpdateBackboneBundleConductorCount(
    CoreState& state, ObjectId bundle_id, int conductor_count,
    const BundleTemplate* previous_template,
    const BundleTemplate* next_template) {
  EditResult<bool> result{};
  const Bundle* bundle = state.authoritative_.edit_state.bundles.find(bundle_id);
  if (bundle == nullptr) {
    result.error = "core invalid input: bundle not found";
    return result;
  }
  const auto stored_template_it = state.authoritative_.bundle_templates.find(bundle->bundle_template_id);
  if (stored_template_it == state.authoritative_.bundle_templates.end()) {
    result.error = "core invalid input: bundle template not found";
    return result;
  }
  const BundleTemplate& previous = previous_template == nullptr ? stored_template_it->second : *previous_template;
  const BundleTemplate& next = next_template == nullptr ? stored_template_it->second : *next_template;
  if (previous.id != bundle->bundle_template_id || next.id != bundle->bundle_template_id) {
    result.error = "backbone unsupported: bundle count update template identity mismatch";
    return result;
  }
  if (!bundle_count_matches_policy(*bundle, previous)) {
    result.error = "backbone unsupported: bundle count is not synchronized with previous template";
    return result;
  }
  const bool requested_count_is_valid = next.count_rule == BundleCountRuleKind::kFixed
                                            ? conductor_count == next.fixed_count
                                            : conductor_count >= next.min_count && conductor_count <= next.max_count;
  if (!requested_count_is_valid) {
    result.error = next.count_rule == BundleCountRuleKind::kFixed
                       ? "backbone unsupported: fixed Bundle count cannot be changed individually"
                       : "backbone unsupported: requested Bundle count is outside template range";
    return result;
  }
  if (bundle->conductor_count == conductor_count) {
    result.ok = true;
    result.value = false;
    return result;
  }

  std::vector<BundleRegenerateScope> scopes{};
  if (!collect_bundle_regenerate_scopes(state, bundle->bundle_template_id, &scopes, &result.error,
                                        bundle_id)) {
    return result;
  }
  if (scopes.empty()) {
    result.error = "backbone unsupported: Bundle count update requires saved backbone continuity";
    return result;
  }

  CoreState trial = state;
  ChangeSet changes{};
  const CoreState::BackboneLaneCountTransition transition{bundle->conductor_count, conductor_count};
  for (const BundleRegenerateScope& scope : scopes) {
    auto regenerated = trial.regenerate_backbone_edge_bundles(
        bundle->bundle_template_id, previous, next, &changes, nullptr, &scope.edge_bundle_ids, nullptr,
        CoreState::BackboneRegenerateCause::kBundleCount, &transition);
    if (!regenerated.ok) {
      result.error = regenerated.error;
      return result;
    }
  }
  CoreState::add_unique_id(changes.updated_ids, bundle_id);
  state.identity_ = trial.identity_;
  state.authoritative_ = trial.authoritative_;
  state.runtime_ = trial.runtime_;
  state.debug_ = trial.debug_;
  result.ok = true;
  result.value = true;
  result.change_set = std::move(changes);
  return result;
}

EditResult<bool> TemplateMutationService::UpdateBundleTemplate(CoreState& state, const BundleTemplate& bundle_template) {
  EditResult<bool> result;
  auto it = state.authoritative_.bundle_templates.find(bundle_template.id);
  if (it == state.authoritative_.bundle_templates.end()) {
    result.error = "core invalid input: bundle template not found";
    return result;
  }
  if (state.find_cable_template(bundle_template.cable_template_id) == nullptr) {
    result.error = "core invalid input: bundle template references unknown cable template";
    return result;
  }
  if (bundle_template.related_pole_type_id != kInvalidPoleTypeId &&
      state.find_pole_type(bundle_template.related_pole_type_id) == nullptr) {
    result.error = "core invalid input: bundle template references unknown pole type";
    return result;
  }
  const auto assembly_for = [&](ModelAssemblyTemplateId assembly_id) -> const ModelAssemblyTemplate* {
    if (assembly_id == kInvalidModelAssemblyTemplateId) {
      return nullptr;
    }
    const auto assembly_it = state.authoritative_.model_assembly_templates.find(assembly_id);
    return assembly_it == state.authoritative_.model_assembly_templates.end() ? nullptr : &assembly_it->second;
  };
  if (bundle_template.row_fixture_assembly_id != kInvalidModelAssemblyTemplateId) {
    const ModelAssemblyTemplate* row_assembly = assembly_for(bundle_template.row_fixture_assembly_id);
    if (row_assembly == nullptr) {
      result.error = "core invalid input: bundle template references unknown row fixture assembly";
      return result;
    }
    if (row_assembly->wire_socket.has_value()) {
      result.error = "core invalid input: row fixture assembly must not own a wire socket";
      return result;
    }
  }
  if (bundle_template.endpoint_fixture_assembly_id != kInvalidModelAssemblyTemplateId) {
    const ModelAssemblyTemplate* endpoint_assembly = assembly_for(bundle_template.endpoint_fixture_assembly_id);
    if (endpoint_assembly == nullptr) {
      result.error = "core invalid input: bundle template references unknown endpoint fixture assembly";
      return result;
    }
    if (!endpoint_assembly->wire_socket.has_value()) {
      result.error = "core invalid input: endpoint fixture assembly requires a wire socket";
      return result;
    }
  }

  BundleTemplate normalized = bundle_template;
  normalized.support_wire_pole_band_id = std::max(0, normalized.support_wire_pole_band_id);
  const SpanVisualAssemblyTemplate& assembly = normalized.span_visual_assembly;
  if (!std::isfinite(assembly.helix_radius_m) || assembly.helix_radius_m < 0.0 ||
      !std::isfinite(assembly.helix_clearance_m) || assembly.helix_clearance_m < 0.0 ||
      !std::isfinite(assembly.helix_turns_per_meter) || assembly.helix_turns_per_meter < 0.0 ||
      assembly.helix_samples_per_turn < 4 || !std::isfinite(assembly.endpoint_trim_m) ||
      assembly.endpoint_trim_m < 0.0 || assembly.visual_member_count_min < 1 ||
      assembly.visual_member_count_max < assembly.visual_member_count_min ||
      !std::isfinite(assembly.visual_member_spacing_m) || assembly.visual_member_spacing_m < 0.0 ||
      (assembly.visual_member_count_max > 1 && assembly.visual_member_spacing_m <= 0.0) ||
      !std::isfinite(assembly.center_wander_amplitude_m) || assembly.center_wander_amplitude_m < 0.0 ||
      !std::isfinite(assembly.center_wander_wavelength_m) ||
      (assembly.center_wander_amplitude_m > 0.0 &&
       assembly.center_wander_wavelength_m <= 0.0) ||
      !std::isfinite(assembly.member_wander_ratio) ||
      assembly.member_wander_ratio < 0.0 || assembly.member_wander_ratio > 1.0 ||
      !std::isfinite(assembly.member_wander_wavelength_m) ||
      !std::isfinite(assembly.member_wander_phase_bias) ||
      !std::isfinite(assembly.member_twist_turns_per_meter) || !std::isfinite(assembly.member_twist_phase) ||
      (assembly.member_wander_ratio > 0.0 && assembly.member_wander_wavelength_m <= 0.0)) {
    result.error = "core invalid input: span visual assembly settings are invalid";
    return result;
  }
  if (assembly.helix_enabled &&
      (!assembly.support_path_enabled || assembly.helix_turns_per_meter <= 0.0)) {
    result.error = "core invalid input: enabled span visual assembly requires a support path and positive helix turns";
    return result;
  }
  if (assembly.helix_enabled) {
    if (normalized.support_wire_pole_band_id > 0) {
      const PoleTypeDefinition* pole_type = state.find_pole_type(normalized.related_pole_type_id);
      const bool has_support_band = pole_type != nullptr && std::any_of(
          pole_type->port_bands.begin(), pole_type->port_bands.end(), [&](const PortPlacementBand& band) {
            return band.band_id == normalized.support_wire_pole_band_id && band.enabled;
          });
      if (!has_support_band) {
        result.error = "core invalid input: enabled span visual assembly requires an enabled support band on the related pole type";
        return result;
      }
    }
    const CableTemplate* assembly_cable = state.find_cable_template(normalized.cable_template_id);
    const double minimum_radius =
        (assembly_cable == nullptr ? 0.0 : assembly_cable->outer_diameter_m) + assembly.helix_clearance_m;
    if (assembly.helix_radius_m > 0.0 && assembly.helix_radius_m + kLengthToleranceM < minimum_radius) {
      result.error = "core invalid input: span visual assembly helix radius cannot contain the wire diameter";
      return result;
    }
  }
  for (CablePopulationRule& rule : normalized.population_rules) {
    if (rule.explicit_seed == 0) {
      rule.explicit_seed = 1;
    }
  }
  if (!validate_population_rules(state, normalized.population_rules, &result.error)) {
    return result;
  }
  if (!std::isfinite(normalized.branch_endpoint_offset_m)) {
    result.error = "core invalid input: bundle branch endpoint offset must be finite";
    return result;
  }
  const CableTemplate* cable_template = state.find_cable_template(normalized.cable_template_id);
  if (normalized.grouped_support_fanout_spacing_m <= kLengthToleranceM) {
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
  const bool scoped_bundle_regenerate =
      topology_regenerate ||
      (fixed_count_change && (changes & ~kRegenerateChangeMask) == 0);

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
  if (scoped_bundle_regenerate) {
    for (ObjectId span_id : affected_spans) {
      if (state.runtime_.backbone_index.span_edge_bundle.find(span_id) ==
          state.runtime_.backbone_index.span_edge_bundle.end()) {
        result.error = "backbone unsupported: bundle policy changes require regeneration";
        return result;
      }
    }
    if (!affected_bundle_ids.empty()) {
      normalized.version += 1;
      CoreState trial = state;
      ChangeSet regenerated_changes{};
      if (fixed_count_change) {
        for (ObjectId bundle_id : affected_bundle_ids) {
          auto regenerated = UpdateBackboneBundleConductorCount(
              trial, bundle_id, normalized.fixed_count, &previous, &normalized);
          if (!regenerated.ok) {
            result.error = regenerated.error;
            return result;
          }
          append_unique(regenerated_changes.created_ids, regenerated.change_set.created_ids);
          append_unique(regenerated_changes.updated_ids, regenerated.change_set.updated_ids);
          append_unique(regenerated_changes.deleted_ids, regenerated.change_set.deleted_ids);
        }
      } else {
        std::vector<BundleRegenerateScope> scopes{};
        if (!collect_bundle_regenerate_scopes(state, normalized.id, &scopes, &result.error) || scopes.empty()) {
          if (result.error.empty()) {
            result.error = "backbone unsupported: bundle policy changes require regeneration";
          }
          return result;
        }
        for (const BundleRegenerateScope& scope : scopes) {
          auto regenerated = trial.regenerate_backbone_edge_bundles(
              normalized.id, previous, normalized, &regenerated_changes, nullptr, &scope.edge_bundle_ids, nullptr,
              CoreState::BackboneRegenerateCause::kBundleTopology);
          if (!regenerated.ok) {
            result.error = regenerated.error;
            return result;
          }
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
  auto updated_template_it = state.authoritative_.bundle_templates.find(normalized.id);
  if (updated_template_it == state.authoritative_.bundle_templates.end()) {
    result.error = "core invalid input: bundle template not found";
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
    result.error = "core invalid input: attachment template not found";
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
    if (std::abs(span->reference_length_m - length) <= kLengthToleranceM) {
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

} // namespace city::wire::state_internal
