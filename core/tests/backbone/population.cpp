#include "fixtures.hpp"
#include "cases.hpp"

#include "../../src/generation/backbone/population.hpp"

#include <algorithm>
#include <cmath>

namespace backbone_tests {
namespace {

wire::core::PoleFrame identity_frame(const wire::core::Vec3d& origin) {
  wire::core::PoleFrame frame{};
  frame.origin = origin;
  frame.forward = {1.0, 0.0, 0.0};
  frame.lateral = {0.0, 1.0, 0.0};
  frame.up = {0.0, 0.0, 1.0};
  return frame;
}

wire::core::generation::backbone::CablePopulationInput population_input(
    wire::core::ObjectId span_id = 101, std::uint64_t seed = 77) {
  using namespace wire::core;
  using namespace wire::core::generation::backbone;
  CablePopulationInput input{};
  input.key.logical_span_id = span_id;
  input.key.edge_bundle_id = 202;
  input.key.rule_owner_id = static_cast<std::uint64_t>(BundleKind::kCommunication);
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

bool same_instances(const std::vector<wire::core::CableSectionLayout>& lhs,
                    const std::vector<wire::core::CableSectionLayout>& rhs) {
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

wire::core::CablePopulationRule lv_population_rule(std::uint64_t seed = 1234) {
  wire::core::CablePopulationRule rule{};
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

bool has_extra_visual_curve(const wire::core::CoreState& state) {
  return std::any_of(
      state.view().visual_curve_parts().parts.begin(), state.view().visual_curve_parts().parts.end(),
      [](const wire::core::VisualCurvePart& part) {
        return part.kind == wire::core::VisualCurvePartKind::kEdgeBody &&
               part.has_cable_instance_key && !part.cable_instance_key.is_base();
      });
}

} // namespace

bool C648_population_same_seed_is_stable() {
  const auto input = population_input();
  const auto first = wire::core::generation::backbone::populate_cable_sections(input);
  const auto second = wire::core::generation::backbone::populate_cable_sections(input);
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

bool C649_population_span_identity_changes_placement() {
  const auto first = wire::core::generation::backbone::populate_cable_sections(population_input(101));
  const auto second = wire::core::generation::backbone::populate_cable_sections(population_input(102));
  if (!first.ok || !second.ok || first.value.sections.empty() || second.value.sections.empty()) {
    return false;
  }
  return dist2(first.value.sections.front().endpoint_a, second.value.sections.front().endpoint_a) > 1e-12;
}

bool C650_population_reserve_blocks_candidates() {
  auto input = population_input();
  wire::core::PlacementReserve reserve{};
  reserve.reserve_id = 1;
  reserve.pole_type_id = input.endpoint_a.pole_type_id;
  reserve.band_id = input.endpoint_a.band_id;
  reserve.lateral_min_m = -1.0;
  reserve.lateral_max_m = 1.0;
  reserve.height_min_m = 5.0;
  reserve.height_max_m = 7.0;
  input.rule.reserves.push_back(reserve);
  const auto result = wire::core::generation::backbone::populate_cable_sections(input);
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
  const auto result = wire::core::generation::backbone::populate_cable_sections(input);
  return result.ok && result.value.sections.empty() && result.value.diagnostic.omitted_count == 1;
}

bool C652_population_endpoint_failure_omits_pair() {
  auto input = population_input();
  input.endpoint_b.valid = false;
  input.endpoint_b.failure_reason = "test endpoint unavailable";
  const auto result = wire::core::generation::backbone::populate_cable_sections(input);
  return result.ok && result.value.sections.empty() &&
         result.value.diagnostic.omitted_count == result.value.diagnostic.extra_count_requested &&
         result.value.diagnostic.reason == "test endpoint unavailable";
}

bool C653_population_rejects_duplicate_band_identity() {
  wire::core::CoreState state;
  if (state.view().pole_types().empty()) {
    return false;
  }
  const auto type_it = state.view().pole_types().begin();
  if (type_it->second.port_bands.empty()) {
    return false;
  }
  wire::core::PoleTypeDefinition duplicate_type = type_it->second;
  duplicate_type.port_bands.push_back(duplicate_type.port_bands.front());
  if (!wire::core::generation::backbone::has_duplicate_band_ids(duplicate_type)) {
    return false;
  }
  const auto updated = state.UpdatePoleTypeDefinition(duplicate_type);
  wire::core::BundleTemplate lv_template = state.view().bundle_templates().at(wire::core::BundleKind::kLowVoltage);
  lv_template.population_rules.push_back(lv_population_rule());
  const auto configured = state.UpdateBundleTemplate(lv_template);
  wire::core::BackboneSpec request = line_req(state);
  request.pole_type_id = duplicate_type.id;
  const auto generated = state.GenerateFromBackboneSpec(request);
  if (!updated.ok || !configured.ok || !generated.ok) {
    return false;
  }
  const bool has_extra_visual = std::any_of(
      state.view().visual_curve_parts().parts.begin(), state.view().visual_curve_parts().parts.end(),
      [](const wire::core::VisualCurvePart& part) {
        return part.kind == wire::core::VisualCurvePartKind::kEdgeBody &&
               part.has_cable_instance_key && !part.cable_instance_key.is_base();
      });
  const bool diagnosed = std::any_of(
      state.view().visual_curve_parts().population_diagnostics.begin(),
      state.view().visual_curve_parts().population_diagnostics.end(),
      [](const wire::core::CablePopulationDiagnostic& diagnostic) {
        return diagnostic.reason == "duplicate band_id in endpoint pole type" &&
               diagnostic.omitted_count == diagnostic.extra_count_requested;
      });
  return !has_extra_visual && diagnosed;
}

bool C654_population_does_not_mutate_logical_topology() {
  wire::core::CoreState control;
  wire::core::CoreState populated;
  wire::core::BundleTemplate lv_template = populated.view().bundle_templates().at(wire::core::BundleKind::kLowVoltage);
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
      [](const wire::core::VisualCurvePart& part) {
        return part.kind == wire::core::VisualCurvePartKind::kEdgeBody &&
               part.has_cable_instance_key && !part.cable_instance_key.is_base();
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
      [](const wire::core::VisualCurvePart& part) {
        return part.kind == wire::core::VisualCurvePartKind::kEdgeBody &&
               part.has_cable_instance_key && !part.cable_instance_key.is_base();
      });
  return !control_has_extra_visual && graph_identity_equal && has_extra_visual;
}

bool C686_population_rule_on_bundle_template_adds_visual_only_sections() {
  wire::core::CoreState control;
  wire::core::CoreState populated;
  wire::core::BundleTemplate lv_template =
      populated.view().bundle_templates().at(wire::core::BundleKind::kLowVoltage);
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
  wire::core::CoreState state;
  wire::core::BundleTemplate lv_template =
      state.view().bundle_templates().at(wire::core::BundleKind::kLowVoltage);
  lv_template.population_rules.push_back(lv_population_rule(11));
  if (!state.UpdateBundleTemplate(lv_template).ok) {
    return false;
  }
  const auto generated = state.GenerateFromBackboneSpec(line_req(state));
  if (!generated.ok || !has_extra_visual_curve(state)) {
    return false;
  }

  wire::core::BundleTemplate edited =
      state.view().bundle_templates().at(wire::core::BundleKind::kLowVoltage);
  edited.population_rules.front().explicit_seed = 12;
  const auto updated = state.UpdateBundleTemplate(edited);
  return updated.ok && state.view().last_update_timing().kind == wire::core::UpdateKind::kReshape &&
         has_extra_visual_curve(state);
}

} // namespace backbone_tests
