#include "fixtures.hpp"
#include "cases.hpp"

#include "../../src/generation/backbone/population.hpp"
#include "city/wire/core_test_hook.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace backbone_tests {
namespace {

city::wire::PoleFrame identity_frame(const city::wire::Vec3d& origin) {
  city::wire::PoleFrame frame{};
  frame.origin = origin;
  frame.forward = {1.0, 0.0, 0.0};
  frame.lateral = {0.0, 1.0, 0.0};
  frame.up = {0.0, 0.0, 1.0};
  return frame;
}

city::wire::generation::backbone::CablePopulationInput population_input(
    city::wire::ObjectId span_id = 101, std::uint64_t seed = 77) {
  using namespace city::wire;
  using namespace city::wire::generation::backbone;
  CablePopulationInput input{};
  input.key.logical_span_id = span_id;
  input.key.edge_bundle_id = 202;
  input.key.rule_owner_id = static_cast<std::uint64_t>(kDefaultCommunicationBundleTemplateId);
  input.key.rule_id = 303;
  input.rule.rule_id = 303;
  input.rule.explicit_seed = seed;
  input.rule.min_extra_count = 3;
  input.rule.max_extra_count = 5;
  input.rule.min_spacing_m = 0.05;
  input.rule.lateral_min_m = -0.5;
  input.rule.lateral_max_m = 0.5;
  input.rule.height_min_m = 5.5;
  input.rule.height_max_m = 6.5;
  input.rule.randomness = 0.6;
  input.endpoint_a.valid = true;
  input.endpoint_a.pole_type_id = 1;
  input.endpoint_a.band_id = 10;
  input.endpoint_a.frame = identity_frame({0.0, 0.0, 0.0});
  input.endpoint_a.original_local = {0.0, 0.0, 6.0};
  input.endpoint_a.lateral_min_m = -0.5;
  input.endpoint_a.lateral_max_m = 0.5;
  input.endpoint_a.height_min_m = 5.5;
  input.endpoint_a.height_max_m = 6.5;
  input.endpoint_b = input.endpoint_a;
  input.endpoint_b.frame = identity_frame({12.0, 0.0, 0.0});
  input.occupied_a_local = {input.endpoint_a.original_local};
  input.occupied_b_local = {input.endpoint_b.original_local};
  return input;
}

bool same_instances(const std::vector<city::wire::CableSectionLayout>& lhs,
                    const std::vector<city::wire::CableSectionLayout>& rhs) {
  if (lhs.size() != rhs.size()) {
    return false;
  }
  for (std::size_t i = 0; i < lhs.size(); ++i) {
    if (lhs[i].key.logical_span_id != rhs[i].key.logical_span_id ||
        lhs[i].key.edge_bundle_id != rhs[i].key.edge_bundle_id ||
        lhs[i].key.rule_id != rhs[i].key.rule_id ||
        lhs[i].key.instance_index != rhs[i].key.instance_index ||
        dist2(lhs[i].endpoint_a, rhs[i].endpoint_a) > 1e-18 ||
        dist2(lhs[i].endpoint_b, rhs[i].endpoint_b) > 1e-18) {
      return false;
    }
  }
  return true;
}

city::wire::CablePopulationRule lv_population_rule(std::uint64_t seed = 1234) {
  city::wire::CablePopulationRule rule{};
  rule.rule_id = 1;
  rule.explicit_seed = seed;
  rule.priority = 10;
  rule.min_extra_count = 1;
  rule.max_extra_count = 1;
  rule.min_spacing_m = 0.04;
  rule.lateral_min_m = -2.0;
  rule.lateral_max_m = 2.0;
  rule.height_min_m = 0.0;
  rule.height_max_m = 20.0;
  rule.randomness = 0.4;
  return rule;
}

struct DuplicateLvTemplates {
  city::wire::BundleTemplateId a = 9001;
  city::wire::BundleTemplateId b = 9002;
};

DuplicateLvTemplates install_duplicate_low_voltage_templates(city::wire::CoreState& state) {
  DuplicateLvTemplates ids{};
  city::wire::BundleTemplate a =
      state.view().bundle_templates().at(city::wire::kDefaultLowVoltageBundleTemplateId);
  city::wire::BundleTemplate b = a;
  a.id = ids.a;
  a.kind = city::wire::BundleKind::kLowVoltage;
  a.name = "LV duplicate A";
  a.fixed_count = 1;
  a.default_count = 1;
  b.id = ids.b;
  b.kind = city::wire::BundleKind::kLowVoltage;
  b.name = "LV duplicate B";
  b.fixed_count = 2;
  b.default_count = 2;
  auto& templates = city::wire::CoreStateTestHook::bundle_templates(state);
  templates[a.id] = a;
  templates[b.id] = b;
  return ids;
}

city::wire::BackboneSpec duplicate_template_line_req(city::wire::CoreState& state,
                                                     const DuplicateLvTemplates& ids) {
  city::wire::BackboneSpec request = line_req(state);
  request.bundles.clear();
  city::wire::BackboneBundleSpec a{};
  a.bundle_template_id = ids.a;
  city::wire::BackboneBundleSpec b{};
  b.bundle_template_id = ids.b;
  request.bundles = {a, b};
  return request;
}

const city::wire::Bundle* find_bundle_by_template(const city::wire::CoreState& state,
                                                  const city::wire::EditResult<city::wire::GenerateBundleFromPathResult>& result,
                                                  city::wire::BundleTemplateId template_id) {
  if (!result.ok) {
    return nullptr;
  }
  for (city::wire::ObjectId bundle_id : result.value.bundle_ids) {
    const city::wire::Bundle* bundle = state.view().bundles().find(bundle_id);
    if (bundle != nullptr && bundle->bundle_template_id == template_id) {
      return bundle;
    }
  }
  return nullptr;
}

const city::wire::Bundle* find_current_bundle_by_template(const city::wire::CoreState& state,
                                                          city::wire::BundleTemplateId template_id) {
  for (const city::wire::Bundle& bundle : state.view().bundles().items()) {
    if (bundle.bundle_template_id == template_id) {
      return &bundle;
    }
  }
  return nullptr;
}

bool source_contains_any(const std::filesystem::path& path, const std::vector<std::string>& banned) {
  std::ifstream in(path);
  if (!in.is_open()) {
    return true;
  }
  std::ostringstream buffer;
  buffer << in.rdbuf();
  const std::string text = buffer.str();
  return std::any_of(banned.begin(), banned.end(), [&](const std::string& pattern) {
    return text.find(pattern) != std::string::npos;
  });
}

bool has_extra_visual_curve(const city::wire::CoreState& state) {
  return std::any_of(
      state.view().visual_curve_parts().parts.begin(), state.view().visual_curve_parts().parts.end(),
      [](const city::wire::VisualCurvePart& part) {
        return part.kind == city::wire::VisualCurvePartKind::kEdgeBody &&
               part.has_section_key && !part.section_key.is_base();
      });
}

} // namespace

bool C648_population_same_seed_is_stable() {
  const auto input = population_input();
  const auto first = city::wire::generation::backbone::populate_cable_sections(input);
  const auto second = city::wire::generation::backbone::populate_cable_sections(input);
  if (!first.ok || !second.ok ||
      first.value.diagnostic.extra_count_requested != second.value.diagnostic.extra_count_requested ||
      !same_instances(first.value.sections, second.value.sections)) {
    return false;
  }
  return std::all_of(first.value.sections.begin(), first.value.sections.end(), [](const auto& section) {
    return std::abs(section.endpoint_a.y) <= 0.5 && std::abs(section.endpoint_b.y) <= 0.5 &&
           section.endpoint_a.z >= 5.5 && section.endpoint_a.z <= 6.5 &&
           section.endpoint_b.z >= 5.5 && section.endpoint_b.z <= 6.5;
  });
}

bool C649_population_span_identity_does_not_change_placement() {
  const auto first = city::wire::generation::backbone::populate_cable_sections(population_input(101));
  const auto second = city::wire::generation::backbone::populate_cable_sections(population_input(102));
  if (!first.ok || !second.ok || first.value.sections.empty() || second.value.sections.empty()) {
    return false;
  }
  return dist2(first.value.sections.front().endpoint_a, second.value.sections.front().endpoint_a) <= 1e-18 &&
         dist2(first.value.sections.front().endpoint_b, second.value.sections.front().endpoint_b) <= 1e-18;
}

bool C650_population_reserve_blocks_candidates() {
  auto input = population_input();
  city::wire::PlacementReserve reserve{};
  reserve.reserve_id = 1;
  reserve.pole_type_id = input.endpoint_a.pole_type_id;
  reserve.band_id = input.endpoint_a.band_id;
  reserve.lateral_min_m = -1.0;
  reserve.lateral_max_m = 1.0;
  reserve.height_min_m = 5.0;
  reserve.height_max_m = 7.0;
  input.rule.reserves.push_back(reserve);
  const auto result = city::wire::generation::backbone::populate_cable_sections(input);
  return result.ok && result.value.sections.empty() && result.value.diagnostic.extra_count_requested > 0 &&
         result.value.diagnostic.omitted_count == result.value.diagnostic.extra_count_requested;
}

bool C651_population_spacing_rejects_overlap() {
  auto input = population_input();
  input.rule.min_extra_count = 1;
  input.rule.max_extra_count = 1;
  input.rule.min_spacing_m = 2.0;
  input.endpoint_a.lateral_min_m = -0.1;
  input.endpoint_a.lateral_max_m = 0.1;
  input.endpoint_a.height_min_m = 5.9;
  input.endpoint_a.height_max_m = 6.1;
  input.endpoint_b.lateral_min_m = -0.1;
  input.endpoint_b.lateral_max_m = 0.1;
  input.endpoint_b.height_min_m = 5.9;
  input.endpoint_b.height_max_m = 6.1;
  const auto result = city::wire::generation::backbone::populate_cable_sections(input);
  return result.ok && result.value.sections.empty() && result.value.diagnostic.omitted_count == 1;
}

bool C652_population_endpoint_failure_omits_pair() {
  auto input = population_input();
  input.endpoint_b.valid = false;
  input.endpoint_b.failure_reason = "test endpoint unavailable";
  const auto result = city::wire::generation::backbone::populate_cable_sections(input);
  return result.ok && result.value.sections.empty() &&
         result.value.diagnostic.omitted_count == result.value.diagnostic.extra_count_requested &&
         result.value.diagnostic.reason == "test endpoint unavailable";
}

bool C653_population_rejects_duplicate_band_identity() {
  city::wire::CoreState state;
  if (state.view().pole_types().empty()) {
    return false;
  }
  const auto type_it = state.view().pole_types().begin();
  if (type_it->second.port_bands.empty()) {
    return false;
  }
  city::wire::PoleTypeDefinition duplicate_type = type_it->second;
  duplicate_type.port_bands.push_back(duplicate_type.port_bands.front());
  if (!city::wire::generation::backbone::has_duplicate_band_ids(duplicate_type)) {
    return false;
  }
  const auto updated = state.UpdatePoleTypeDefinition(duplicate_type);
  city::wire::BundleTemplate lv_template = state.view().bundle_templates().at(city::wire::DefaultBundleTemplateId(city::wire::BundleKind::kLowVoltage));
  lv_template.population_rules.push_back(lv_population_rule());
  const auto configured = state.UpdateBundleTemplate(lv_template);
  city::wire::BackboneSpec request = line_req(state);
  request.pole_type_id = duplicate_type.id;
  const auto generated = state.GenerateFromBackboneSpec(request);
  if (!updated.ok || !configured.ok || !generated.ok) {
    return false;
  }
  const bool has_extra_visual = std::any_of(
      state.view().visual_curve_parts().parts.begin(), state.view().visual_curve_parts().parts.end(),
      [](const city::wire::VisualCurvePart& part) {
        return part.kind == city::wire::VisualCurvePartKind::kEdgeBody &&
               part.has_section_key && !part.section_key.is_base();
      });
  const bool diagnosed = std::any_of(
      state.view().visual_curve_parts().population_diagnostics.begin(),
      state.view().visual_curve_parts().population_diagnostics.end(),
      [](const city::wire::CablePopulationDiagnostic& diagnostic) {
        return diagnostic.reason == "duplicate band_id in endpoint pole type" &&
               diagnostic.omitted_count == diagnostic.extra_count_requested;
      });
  return !has_extra_visual && diagnosed;
}

bool C654_population_does_not_mutate_logical_topology() {
  city::wire::CoreState control;
  city::wire::CoreState populated;
  city::wire::BundleTemplate lv_template = populated.view().bundle_templates().at(city::wire::DefaultBundleTemplateId(city::wire::BundleKind::kLowVoltage));
  lv_template.population_rules.push_back(lv_population_rule());
  const auto configured = populated.UpdateBundleTemplate(lv_template);
  if (!configured.ok) {
    return false;
  }
  const auto control_result = control.GenerateFromBackboneSpec(line_req(control));
  const auto populated_result = populated.GenerateFromBackboneSpec(line_req(populated));
  if (!control_result.ok || !populated_result.ok) {
    return false;
  }
  const auto& control_graph = control.view().backbone();
  const auto& populated_graph = populated.view().backbone();
  const bool control_has_extra_visual = std::any_of(
      control.view().visual_curve_parts().parts.begin(), control.view().visual_curve_parts().parts.end(),
      [](const city::wire::VisualCurvePart& part) {
        return part.kind == city::wire::VisualCurvePartKind::kEdgeBody &&
               part.has_section_key && !part.section_key.is_base();
      });
  const bool topology_equal =
      control_result.value.generated_pole_ids == populated_result.value.generated_pole_ids &&
      control_result.value.generated_span_ids == populated_result.value.generated_span_ids &&
      control.view().poles().size() == populated.view().poles().size() &&
      control.view().ports().size() == populated.view().ports().size() &&
      control.view().bundles().size() == populated.view().bundles().size() &&
      control.view().spans().size() == populated.view().spans().size() &&
      control_graph.nodes.size() == populated_graph.nodes.size() &&
      control_graph.edges.size() == populated_graph.edges.size() &&
      control_graph.edge_bundles.size() == populated_graph.edge_bundles.size() &&
      control_graph.port_bindings.size() == populated_graph.port_bindings.size() &&
      control_graph.span_bindings.size() == populated_graph.span_bindings.size();
  bool graph_identity_equal = topology_equal;
  for (std::size_t i = 0; graph_identity_equal && i < control_graph.nodes.size(); ++i) {
    graph_identity_equal =
        control_graph.nodes[i].node_id == populated_graph.nodes[i].node_id &&
        control_graph.nodes[i].pole_id == populated_graph.nodes[i].pole_id;
  }
  for (std::size_t i = 0; graph_identity_equal && i < control_graph.edges.size(); ++i) {
    graph_identity_equal =
        control_graph.edges[i].edge_id == populated_graph.edges[i].edge_id &&
        control_graph.edges[i].node_a == populated_graph.edges[i].node_a &&
        control_graph.edges[i].node_b == populated_graph.edges[i].node_b;
  }
  for (std::size_t i = 0; graph_identity_equal && i < control_graph.edge_bundles.size(); ++i) {
    graph_identity_equal =
        control_graph.edge_bundles[i].edge_bundle_id == populated_graph.edge_bundles[i].edge_bundle_id &&
        control_graph.edge_bundles[i].edge_id == populated_graph.edge_bundles[i].edge_id &&
        control_graph.edge_bundles[i].bundle_id == populated_graph.edge_bundles[i].bundle_id &&
        control_graph.edge_bundles[i].span_ids == populated_graph.edge_bundles[i].span_ids;
  }
  const bool has_extra_visual = std::any_of(
      populated.view().visual_curve_parts().parts.begin(), populated.view().visual_curve_parts().parts.end(),
      [](const city::wire::VisualCurvePart& part) {
        return part.kind == city::wire::VisualCurvePartKind::kEdgeBody &&
               part.has_section_key && !part.section_key.is_base();
      });
  return !control_has_extra_visual && graph_identity_equal && has_extra_visual;
}

bool C686_population_rule_on_bundle_template_adds_visual_only_sections() {
  city::wire::CoreState control;
  city::wire::CoreState populated;
  city::wire::BundleTemplate lv_template =
      populated.view().bundle_templates().at(city::wire::DefaultBundleTemplateId(city::wire::BundleKind::kLowVoltage));
  lv_template.population_rules.push_back(lv_population_rule());
  if (!populated.UpdateBundleTemplate(lv_template).ok) {
    return false;
  }

  const auto control_result = control.GenerateFromBackboneSpec(line_req(control));
  const auto populated_result = populated.GenerateFromBackboneSpec(line_req(populated));
  if (!control_result.ok || !populated_result.ok) {
    return false;
  }

  return !has_extra_visual_curve(control) && has_extra_visual_curve(populated) &&
         control.view().poles().size() == populated.view().poles().size() &&
         control.view().ports().size() == populated.view().ports().size() &&
         control.view().bundles().size() == populated.view().bundles().size() &&
         control.view().spans().size() == populated.view().spans().size() &&
         control.view().backbone().nodes.size() == populated.view().backbone().nodes.size() &&
         control.view().backbone().edges.size() == populated.view().backbone().edges.size() &&
         control.view().backbone().edge_bundles.size() == populated.view().backbone().edge_bundles.size() &&
         control.view().backbone().span_bindings.size() == populated.view().backbone().span_bindings.size();
}

bool C687_population_rule_update_is_reshape_not_regenerate() {
  city::wire::CoreState state;
  city::wire::BundleTemplate lv_template =
      state.view().bundle_templates().at(city::wire::DefaultBundleTemplateId(city::wire::BundleKind::kLowVoltage));
  lv_template.population_rules.push_back(lv_population_rule(11));
  if (!state.UpdateBundleTemplate(lv_template).ok) {
    return false;
  }
  const auto generated = state.GenerateFromBackboneSpec(line_req(state));
  if (!generated.ok || !has_extra_visual_curve(state)) {
    return false;
  }

  city::wire::BundleTemplate edited =
      state.view().bundle_templates().at(city::wire::DefaultBundleTemplateId(city::wire::BundleKind::kLowVoltage));
  edited.population_rules.front().explicit_seed = 12;
  const auto updated = state.UpdateBundleTemplate(edited);
  return updated.ok && state.view().last_update_timing().kind == city::wire::UpdateKind::kReshape &&
         has_extra_visual_curve(state);
}

bool C730_same_kind_bundle_templates_can_coexist() {
  city::wire::CoreState state;
  const DuplicateLvTemplates ids = install_duplicate_low_voltage_templates(state);
  const auto& templates = state.view().bundle_templates();
  const auto a = templates.find(ids.a);
  const auto b = templates.find(ids.b);
  return a != templates.end() && b != templates.end() &&
         a->second.id != b->second.id &&
         a->second.kind == city::wire::BundleKind::kLowVoltage &&
         b->second.kind == city::wire::BundleKind::kLowVoltage;
}

bool C731_backbone_spec_references_duplicate_kind_templates() {
  city::wire::CoreState state;
  const DuplicateLvTemplates ids = install_duplicate_low_voltage_templates(state);
  const auto generated = state.GenerateFromBackboneSpec(duplicate_template_line_req(state, ids));
  const city::wire::Bundle* a = find_bundle_by_template(state, generated, ids.a);
  const city::wire::Bundle* b = find_bundle_by_template(state, generated, ids.b);
  return generated.ok && a != nullptr && b != nullptr && a->id != b->id &&
         a->bundle_template_id == ids.a && b->bundle_template_id == ids.b;
}

bool C732_population_rule_owner_is_bundle_template_id() {
  city::wire::CoreState state;
  const DuplicateLvTemplates ids = install_duplicate_low_voltage_templates(state);
  city::wire::BundleTemplate a = state.view().bundle_templates().at(ids.a);
  a.population_rules.push_back(lv_population_rule(31));
  if (!state.UpdateBundleTemplate(a).ok) {
    return false;
  }
  const auto generated = state.GenerateFromBackboneSpec(duplicate_template_line_req(state, ids));
  if (!generated.ok) {
    return false;
  }
  bool saw_a = false;
  bool saw_b = false;
  for (const city::wire::VisualCurvePart& part : state.view().visual_curve_parts().parts) {
    if (part.kind != city::wire::VisualCurvePartKind::kEdgeBody || !part.has_section_key ||
        part.section_key.is_base()) {
      continue;
    }
    saw_a = saw_a || part.section_key.rule_owner_id == ids.a;
    saw_b = saw_b || part.section_key.rule_owner_id == ids.b;
  }
  return saw_a && !saw_b;
}

bool C733_regenerate_scope_uses_bundle_template_id() {
  city::wire::CoreState state;
  const DuplicateLvTemplates ids = install_duplicate_low_voltage_templates(state);
  const auto generated = state.GenerateFromBackboneSpec(duplicate_template_line_req(state, ids));
  const city::wire::Bundle* a_before = find_bundle_by_template(state, generated, ids.a);
  const city::wire::Bundle* b_before = find_bundle_by_template(state, generated, ids.b);
  if (!generated.ok || a_before == nullptr || b_before == nullptr ||
      a_before->conductor_count != 1 || b_before->conductor_count != 2) {
    return false;
  }
  const city::wire::ObjectId b_bundle_id = b_before->id;
  city::wire::BundleTemplate a = state.view().bundle_templates().at(ids.a);
  a.fixed_count = 3;
  const auto updated = state.UpdateBundleTemplate(a);
  const city::wire::Bundle* a_after = find_current_bundle_by_template(state, ids.a);
  const city::wire::Bundle* b_after = state.view().bundles().find(b_bundle_id);
  return updated.ok && a_after != nullptr && b_after != nullptr &&
         a_after->conductor_count == 3 && b_after->conductor_count == 2;
}

bool C734_cable_template_lookup_is_not_kind_based() {
  city::wire::CoreState state;
  const DuplicateLvTemplates ids = install_duplicate_low_voltage_templates(state);
  city::wire::CableTemplate cable_b =
      state.view().cable_templates().at(state.view().bundle_templates().at(ids.a).cable_template_id);
  cable_b.id = 9101;
  cable_b.color_rgba = 0x11223344u;
  city::wire::CoreStateTestHook::cable_templates(state)[cable_b.id] = cable_b;
  city::wire::BundleTemplate b = state.view().bundle_templates().at(ids.b);
  b.cable_template_id = cable_b.id;
  if (!state.UpdateBundleTemplate(b).ok) {
    return false;
  }
  const auto generated = state.GenerateFromBackboneSpec(duplicate_template_line_req(state, ids));
  if (!generated.ok) {
    return false;
  }
  bool saw_b_color = false;
  bool saw_a_other_color = false;
  for (const city::wire::VisualCurvePart& part : state.view().visual_curve_parts().parts) {
    if (part.kind != city::wire::VisualCurvePartKind::kEdgeBody ||
        (part.has_section_key && !part.section_key.is_base())) {
      continue;
    }
    if (part.bundle_template_id == ids.b) {
      saw_b_color = saw_b_color || part.color_rgba == cable_b.color_rgba;
    }
    if (part.bundle_template_id == ids.a) {
      saw_a_other_color = saw_a_other_color || part.color_rgba != cable_b.color_rgba;
    }
  }
  return saw_b_color && saw_a_other_color;
}

bool C735_bundle_template_id_source_guard() {
  const std::vector<std::filesystem::path> paths = {
      "domains/wire/include/city/wire/core_state_storage_types.hpp",
      "domains/wire/include/city/wire/entities.hpp",
      "domains/wire/include/city/wire/workflow_types.hpp",
      "domains/wire/include/city/wire/core_authoritative_types.hpp",
      "domains/wire/src/generation/backbone/regenerate.cpp",
      "domains/wire/src/generation/backbone/population.cpp",
      "domains/wire/src/state/template/update.cpp"};
  const std::vector<std::string> banned = {
      "std::unordered_map<BundleKind, BundleTemplate>",
      "std::map<BundleKind, BundleTemplate>",
      "BundleKind bundle_template_id",
      "rule_owner_id = static_cast<std::uint64_t>(BundleKind",
      "bundle_template_id = BundleKind::",
      "bundle->bundle_template_id == BundleKind::"};
  return std::none_of(paths.begin(), paths.end(), [&](const std::filesystem::path& path) {
    return source_contains_any(repo_root() / path, banned);
  });
}

} // namespace backbone_tests
