#include "fixtures.hpp"
#include "cases.hpp"

#include "city/wire/core_test_hook.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace backbone_tests {
namespace {

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
  a.name = "LV duplicate A";
  a.fixed_count = 1;
  a.default_count = 1;
  b.id = ids.b;
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

const city::wire::Bundle* find_bundle_by_template(
    const city::wire::CoreState& state,
    const city::wire::EditResult<city::wire::GenerateBundleFromPathResult>& result,
    city::wire::BundleTemplateId template_id) {
  if (!result.ok) return nullptr;
  for (city::wire::ObjectId bundle_id : result.value.bundle_ids) {
    const city::wire::Bundle* bundle = state.view().bundles().find(bundle_id);
    if (bundle != nullptr && bundle->bundle_template_id == template_id) return bundle;
  }
  return nullptr;
}

const city::wire::Bundle* find_current_bundle_by_template(
    const city::wire::CoreState& state, city::wire::BundleTemplateId template_id) {
  const auto found = std::ranges::find_if(state.view().bundles().items(), [template_id](const auto& bundle) {
    return bundle.bundle_template_id == template_id;
  });
  return found == state.view().bundles().items().end() ? nullptr : &*found;
}

bool source_contains_any(const std::filesystem::path& path,
                         const std::vector<std::string>& banned) {
  std::ifstream in(path);
  if (!in) return true;
  std::ostringstream buffer;
  buffer << in.rdbuf();
  const std::string text = buffer.str();
  return std::ranges::any_of(banned, [&](const std::string& pattern) {
    return text.find(pattern) != std::string::npos;
  });
}

} // namespace

bool C730_same_kind_bundle_templates_can_coexist() {
  city::wire::CoreState state;
  const DuplicateLvTemplates ids = install_duplicate_low_voltage_templates(state);
  const auto& templates = state.view().bundle_templates();
  const auto a = templates.find(ids.a);
  const auto b = templates.find(ids.b);
  return a != templates.end() && b != templates.end() && a->second.id != b->second.id &&
         a->second.kind == city::wire::BundleKind::kLowVoltage &&
         b->second.kind == city::wire::BundleKind::kLowVoltage;
}

bool C731_backbone_spec_references_duplicate_kind_templates() {
  city::wire::CoreState state;
  const DuplicateLvTemplates ids = install_duplicate_low_voltage_templates(state);
  const auto generated = state.GenerateFromBackboneSpec(duplicate_template_line_req(state, ids));
  const city::wire::Bundle* a = find_bundle_by_template(state, generated, ids.a);
  const city::wire::Bundle* b = find_bundle_by_template(state, generated, ids.b);
  return generated.ok && a != nullptr && b != nullptr && a->id != b->id;
}

bool C732_route_bundle_rule_owner_is_bundle_template_id() {
  city::wire::CoreState state;
  const DuplicateLvTemplates ids = install_duplicate_low_voltage_templates(state);
  city::wire::RouteBundleVariationInput input{};
  input.route_seed = 31;
  input.preferred_side_sign = -1;
  input.pole_type_id = 2;
  input.rules = {{ids.a, 1, 1, 1, 7.0, 7.4, 0.2, 0.4}};
  const auto resolved = state.ResolveRouteBundleVariation(input);
  return resolved.ok && resolved.value.size() == 1 &&
         resolved.value.front().bundle_template_id == ids.a;
}

bool C733_regenerate_scope_uses_bundle_template_id() {
  city::wire::CoreState state;
  const DuplicateLvTemplates ids = install_duplicate_low_voltage_templates(state);
  const auto generated = state.GenerateFromBackboneSpec(duplicate_template_line_req(state, ids));
  const city::wire::Bundle* a_before = find_bundle_by_template(state, generated, ids.a);
  const city::wire::Bundle* b_before = find_bundle_by_template(state, generated, ids.b);
  if (!generated.ok || a_before == nullptr || b_before == nullptr ||
      a_before->conductor_count != 1 || b_before->conductor_count != 2) return false;
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
  if (!state.UpdateBundleTemplate(b).ok) return false;
  const auto generated = state.GenerateFromBackboneSpec(duplicate_template_line_req(state, ids));
  if (!generated.ok) return false;
  bool saw_b_color = false;
  bool saw_a_other_color = false;
  for (const city::wire::VisualCurvePart& part : state.view().visual_curve_parts().parts) {
    if (part.kind != city::wire::VisualCurvePartKind::kEdgeBody) continue;
    if (part.bundle_template_id == ids.b) saw_b_color = saw_b_color || part.color_rgba == cable_b.color_rgba;
    if (part.bundle_template_id == ids.a) saw_a_other_color = saw_a_other_color || part.color_rgba != cable_b.color_rgba;
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
      "domains/wire/src/state/route_bundle_variation.cpp",
      "domains/wire/src/state/template/update.cpp"};
  const std::vector<std::string> banned = {
      "std::unordered_map<BundleKind, BundleTemplate>", "std::map<BundleKind, BundleTemplate>",
      "BundleKind bundle_template_id", "bundle_template_id = BundleKind::",
      "bundle->bundle_template_id == BundleKind::"};
  return std::ranges::none_of(paths, [&](const std::filesystem::path& path) {
    return source_contains_any(repo_root() / path, banned);
  });
}

} // namespace backbone_tests
