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

wire::core::generation::backbone::SpanMemberPopulationInput population_input(
    wire::core::ObjectId span_id = 101, std::uint64_t seed = 77) {
  using namespace wire::core;
  using namespace wire::core::generation::backbone;
  SpanMemberPopulationInput input{};
  input.key.logical_span_id = span_id;
  input.key.edge_bundle_id = 202;
  input.key.rule_owner_id = static_cast<std::uint64_t>(BundleKind::kCommunication);
  input.key.rule_id = 303;
  input.explicit_seed = seed;
  input.rule.rule_id = 303;
  input.rule.bundle_template_id = BundleKind::kCommunication;
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

bool same_instances(const std::vector<wire::core::SpanMemberLayout>& lhs,
                    const std::vector<wire::core::SpanMemberLayout>& rhs) {
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

wire::core::ExperimentalSpanMemberPopulationConfig lv_population_config() {
  wire::core::ExperimentalSpanMemberPopulationConfig config{};
  config.enabled = true;
  config.explicit_seed = 1234;
  wire::core::ExperimentalSpanMemberRule rule{};
  rule.rule_id = 1;
  rule.bundle_template_id = wire::core::BundleKind::kLowVoltage;
  rule.priority = 10;
  rule.min_extra_count = 1;
  rule.max_extra_count = 1;
  rule.min_spacing_m = 0.04;
  rule.lateral_min_m = -2.0;
  rule.lateral_max_m = 2.0;
  rule.height_min_m = 0.0;
  rule.height_max_m = 20.0;
  rule.randomness = 0.4;
  config.rules.push_back(rule);
  return config;
}

} // namespace

bool C648_experimental_population_same_seed_is_stable() {
  const auto input = population_input();
  const auto first = wire::core::generation::backbone::populate_span_members(input);
  const auto second = wire::core::generation::backbone::populate_span_members(input);
  if (!first.ok || !second.ok ||
      first.value.diagnostic.extra_count_requested != second.value.diagnostic.extra_count_requested ||
      !same_instances(first.value.members, second.value.members)) {
    return false;
  }
  return std::all_of(first.value.members.begin(), first.value.members.end(), [](const auto& member) {
    return std::abs(member.endpoint_a.y) <= 1e-12 && std::abs(member.endpoint_b.y) <= 1e-12 &&
           std::abs(member.endpoint_a.z - 6.0) > 1e-12 && std::abs(member.endpoint_b.z - 6.0) > 1e-12;
  });
}

bool C649_experimental_population_span_identity_changes_placement() {
  const auto first = wire::core::generation::backbone::populate_span_members(population_input(101));
  const auto second = wire::core::generation::backbone::populate_span_members(population_input(102));
  if (!first.ok || !second.ok || first.value.members.empty() || second.value.members.empty()) {
    return false;
  }
  return dist2(first.value.members.front().endpoint_a, second.value.members.front().endpoint_a) > 1e-12;
}

bool C650_experimental_population_reserve_blocks_candidates() {
  auto input = population_input();
  wire::core::ExperimentalPlacementReserve reserve{};
  reserve.reserve_id = 1;
  reserve.pole_type_id = input.endpoint_a.pole_type_id;
  reserve.band_id = input.endpoint_a.band_id;
  reserve.lateral_min_m = -1.0;
  reserve.lateral_max_m = 1.0;
  reserve.height_min_m = 5.0;
  reserve.height_max_m = 7.0;
  input.reserves.push_back(reserve);
  const auto result = wire::core::generation::backbone::populate_span_members(input);
  return result.ok && result.value.members.empty() && result.value.diagnostic.extra_count_requested > 0 &&
         result.value.diagnostic.omitted_count == result.value.diagnostic.extra_count_requested;
}

bool C651_experimental_population_spacing_rejects_overlap() {
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
  const auto result = wire::core::generation::backbone::populate_span_members(input);
  return result.ok && result.value.members.empty() && result.value.diagnostic.omitted_count == 1;
}

bool C652_experimental_population_endpoint_failure_omits_pair() {
  auto input = population_input();
  input.endpoint_b.valid = false;
  input.endpoint_b.failure_reason = "test endpoint unavailable";
  const auto result = wire::core::generation::backbone::populate_span_members(input);
  return result.ok && result.value.members.empty() &&
         result.value.diagnostic.omitted_count == result.value.diagnostic.extra_count_requested &&
         result.value.diagnostic.reason == "test endpoint unavailable";
}

bool C653_experimental_population_rejects_duplicate_band_identity() {
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
  const auto configured = state.UpdateExperimentalSpanMemberPopulationConfig(lv_population_config());
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
               part.has_span_member_key && !part.span_member_key.is_base();
      });
  const bool diagnosed = std::any_of(
      state.view().visual_curve_parts().experimental_population_diagnostics.begin(),
      state.view().visual_curve_parts().experimental_population_diagnostics.end(),
      [](const wire::core::SpanMemberPopulationDiagnostic& diagnostic) {
        return diagnostic.reason == "duplicate band_id in endpoint pole type" &&
               diagnostic.omitted_count == diagnostic.extra_count_requested;
      });
  return !has_extra_visual && diagnosed;
}

bool C654_experimental_population_does_not_mutate_logical_topology() {
  wire::core::CoreState control;
  wire::core::CoreState experimental;
  const auto configured = experimental.UpdateExperimentalSpanMemberPopulationConfig(lv_population_config());
  if (!configured.ok) {
    return false;
  }
  const auto control_result = control.GenerateFromBackboneSpec(line_req(control));
  const auto experimental_result = experimental.GenerateFromBackboneSpec(line_req(experimental));
  if (!control_result.ok || !experimental_result.ok) {
    return false;
  }
  const auto& control_graph = control.view().backbone();
  const auto& experimental_graph = experimental.view().backbone();
  const bool control_has_extra_visual = std::any_of(
      control.view().visual_curve_parts().parts.begin(), control.view().visual_curve_parts().parts.end(),
      [](const wire::core::VisualCurvePart& part) {
        return part.kind == wire::core::VisualCurvePartKind::kEdgeBody &&
               part.has_span_member_key && !part.span_member_key.is_base();
      });
  const bool topology_equal =
      control_result.value.generated_pole_ids == experimental_result.value.generated_pole_ids &&
      control_result.value.generated_span_ids == experimental_result.value.generated_span_ids &&
      control.view().poles().size() == experimental.view().poles().size() &&
      control.view().ports().size() == experimental.view().ports().size() &&
      control.view().bundles().size() == experimental.view().bundles().size() &&
      control.view().spans().size() == experimental.view().spans().size() &&
      control_graph.nodes.size() == experimental_graph.nodes.size() &&
      control_graph.edges.size() == experimental_graph.edges.size() &&
      control_graph.edge_bundles.size() == experimental_graph.edge_bundles.size() &&
      control_graph.port_bindings.size() == experimental_graph.port_bindings.size() &&
      control_graph.span_bindings.size() == experimental_graph.span_bindings.size();
  bool graph_identity_equal = topology_equal;
  for (std::size_t i = 0; graph_identity_equal && i < control_graph.nodes.size(); ++i) {
    graph_identity_equal =
        control_graph.nodes[i].node_id == experimental_graph.nodes[i].node_id &&
        control_graph.nodes[i].pole_id == experimental_graph.nodes[i].pole_id;
  }
  for (std::size_t i = 0; graph_identity_equal && i < control_graph.edges.size(); ++i) {
    graph_identity_equal =
        control_graph.edges[i].edge_id == experimental_graph.edges[i].edge_id &&
        control_graph.edges[i].node_a == experimental_graph.edges[i].node_a &&
        control_graph.edges[i].node_b == experimental_graph.edges[i].node_b;
  }
  for (std::size_t i = 0; graph_identity_equal && i < control_graph.edge_bundles.size(); ++i) {
    graph_identity_equal =
        control_graph.edge_bundles[i].edge_bundle_id == experimental_graph.edge_bundles[i].edge_bundle_id &&
        control_graph.edge_bundles[i].edge_id == experimental_graph.edge_bundles[i].edge_id &&
        control_graph.edge_bundles[i].bundle_id == experimental_graph.edge_bundles[i].bundle_id &&
        control_graph.edge_bundles[i].span_ids == experimental_graph.edge_bundles[i].span_ids;
  }
  const bool has_extra_visual = std::any_of(
      experimental.view().visual_curve_parts().parts.begin(), experimental.view().visual_curve_parts().parts.end(),
      [](const wire::core::VisualCurvePart& part) {
        return part.kind == wire::core::VisualCurvePartKind::kEdgeBody &&
               part.has_span_member_key && !part.span_member_key.is_base();
      });
  return !control_has_extra_visual && graph_identity_equal && has_extra_visual;
}

} // namespace backbone_tests
