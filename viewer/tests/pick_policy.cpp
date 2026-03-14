#include <algorithm>
#include <vector>

#include "path_pick_policy.hpp"
#include "registry.hpp"
#include "wire/core/core_state.hpp"

namespace {

wire::core::PoleTypeId FirstPoleTypeId(const wire::core::CoreState& state) {
  wire::core::PoleTypeId best = wire::core::kInvalidPoleTypeId;
  for (const auto& [id, _] : state.view().pole_types()) {
    if (best == wire::core::kInvalidPoleTypeId || id < best) {
      best = id;
    }
  }
  return best;
}

wire::core::ObjectId CreatePole(wire::core::CoreState& state, const wire::core::Vec3d& pos, wire::core::PoleTypeId type_id,
                                const char* name) {
  wire::core::Transformd tf{};
  tf.position = pos;
  const auto pole = state.AddPole(tf, 10.0, name);
  if (!pole.ok) {
    return wire::core::kInvalidObjectId;
  }
  if (!state.ApplyPoleType(pole.value, type_id).ok) {
    return wire::core::kInvalidObjectId;
  }
  return pole.value;
}

wire::core::ObjectId CreateTemplateSpan(wire::core::CoreState& state, wire::core::PoleTypeId type_id,
                                        wire::core::BundleKind kind) {
  const wire::core::ObjectId pole_a = CreatePole(state, {0.0, 0.0, 0.0}, type_id, "A");
  const wire::core::ObjectId pole_b = CreatePole(state, {10.0, 0.0, 0.0}, type_id, "B");
  if (pole_a == wire::core::kInvalidObjectId || pole_b == wire::core::kInvalidObjectId) {
    return wire::core::kInvalidObjectId;
  }
  wire::core::AddConnectionByPoleOptions options{};
  options.use_bundle_template = true;
  options.bundle_template_id = kind;
  const auto result =
      state.AddConnectionByPole(pole_a, pole_b, wire::core::ConnectionCategory::kLowVoltage, options);
  return result.ok ? result.value.span_id : wire::core::kInvalidObjectId;
}

std::uint32_t TemplateMask(wire::core::BundleKind kind) {
  return (1u << static_cast<unsigned>(kind));
}

bool test_selected_template_wins_over_hit_span_template() {
  wire::core::CoreState state;
  const wire::core::PoleTypeId type_id = FirstPoleTypeId(state);
  if (type_id == wire::core::kInvalidPoleTypeId) {
    return false;
  }
  const wire::core::ObjectId span_id = CreateTemplateSpan(state, type_id, wire::core::BundleKind::kHighVoltage);
  if (span_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::PickResult pick{};
  pick.hit_kind = wire::core::PickHitKind::kSegment;
  pick.hit_id = span_id;
  const wire::core::BundleKind resolved =
      ResolveBundleTemplateForPathPick(state, TemplateMask(wire::core::BundleKind::kLowVoltage), pick);
  return resolved == wire::core::BundleKind::kLowVoltage;
}

bool test_hit_span_template_is_used_when_nothing_selected() {
  wire::core::CoreState state;
  const wire::core::PoleTypeId type_id = FirstPoleTypeId(state);
  if (type_id == wire::core::kInvalidPoleTypeId) {
    return false;
  }
  const wire::core::ObjectId span_id = CreateTemplateSpan(state, type_id, wire::core::BundleKind::kHighVoltage);
  if (span_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::PickResult pick{};
  pick.hit_kind = wire::core::PickHitKind::kSegment;
  pick.hit_id = span_id;
  const wire::core::BundleKind resolved = ResolveBundleTemplateForPathPick(state, 0u, pick);
  return resolved == wire::core::BundleKind::kHighVoltage;
}

bool test_midair_branch_block_checks_selected_templates() {
  wire::core::CoreState state;
  const std::vector<wire::core::BundleKind> selected =
      ResolveTemplateKindsForPathPick(state, TemplateMask(wire::core::BundleKind::kLowVoltage), {});
  const std::string blocked = FindMidairBranchBlockedTemplateName(state, selected);
  return selected.size() == 1 && selected.front() == wire::core::BundleKind::kLowVoltage && blocked.empty();
}

bool test_midair_branch_block_finds_disallowed_template() {
  wire::core::CoreState state;
  const std::uint32_t mask =
      TemplateMask(wire::core::BundleKind::kLowVoltage) | TemplateMask(wire::core::BundleKind::kHighVoltage);
  const std::vector<wire::core::BundleKind> selected = ResolveTemplateKindsForPathPick(state, mask, {});
  const std::string blocked = FindMidairBranchBlockedTemplateName(state, selected);
  return std::find(selected.begin(), selected.end(), wire::core::BundleKind::kHighVoltage) != selected.end() &&
         blocked == "HV_3PH";
}

bool test_draw_path_segment_pick_near_endpoint_snaps_to_endpoint_position() {
  wire::core::CoreState state;
  const wire::core::PoleTypeId type_id = FirstPoleTypeId(state);
  if (type_id == wire::core::kInvalidPoleTypeId) {
    return false;
  }
  const wire::core::ObjectId pole_a = CreatePole(state, {0.0, 0.0, 0.0}, type_id, "A");
  const wire::core::ObjectId pole_b = CreatePole(state, {10.0, 0.0, 0.0}, type_id, "B");
  if (pole_a == wire::core::kInvalidObjectId || pole_b == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::PickResult pick{};
  pick.hit_kind = wire::core::PickHitKind::kSegment;
  pick.has_segment_endpoints = true;
  pick.segment_node_a_id = pole_a;
  pick.segment_node_b_id = pole_b;
  pick.segment_endpoint_a_world = {0.6, 0.0, 8.7};
  pick.segment_endpoint_b_world = {9.4, 0.0, 8.7};
  pick.hit_pos_world = {0.6, 0.0, 0.0};

  const wire::core::PickResult normalized = NormalizeDrawPathPick(state, pick, 1.25);
  return normalized.hit_kind == wire::core::PickHitKind::kNode &&
         normalized.hit_id == pole_a &&
         normalized.hit_pos_world.x == 0.0 &&
         normalized.hit_pos_world.y == 0.0 &&
         normalized.hit_pos_world.z == 0.0;
}

bool test_draw_path_segment_pick_midspan_does_not_snap_to_endpoint_position() {
  wire::core::CoreState state;
  wire::core::PickResult pick{};
  pick.hit_kind = wire::core::PickHitKind::kSegment;
  pick.has_segment_endpoints = true;
  pick.segment_node_a_id = 10;
  pick.segment_node_b_id = 11;
  pick.segment_endpoint_a_world = {0.0, 0.0, 0.0};
  pick.segment_endpoint_b_world = {10.0, 0.0, 0.0};
  pick.hit_pos_world = {5.0, 0.0, 0.0};

  const wire::core::PickResult normalized = NormalizeDrawPathPick(state, pick, 1.25);
  return normalized.hit_pos_world.x == 5.0 &&
         normalized.hit_pos_world.y == 0.0 &&
         normalized.hit_pos_world.z == 0.0;
}

void register_pick_policy_tests(viewer_test_registry::TestRegistry& tests) {
  viewer_test_registry::AddTest(tests, "V01", "Selected DrawPath template wins over hit span template",
                                test_selected_template_wins_over_hit_span_template);
  viewer_test_registry::AddTest(tests, "V02", "Hit span template is used when no DrawPath template is selected",
                                test_hit_span_template_is_used_when_nothing_selected);
  viewer_test_registry::AddTest(tests, "V03", "Midair branch block uses selected templates, not hit span fallback",
                                test_midair_branch_block_checks_selected_templates);
  viewer_test_registry::AddTest(tests, "V04", "Midair branch block finds any disallowed selected template",
                                test_midair_branch_block_finds_disallowed_template);
  viewer_test_registry::AddTest(tests, "V09", "DrawPath segment pick near endpoint snaps to endpoint position",
                                test_draw_path_segment_pick_near_endpoint_snaps_to_endpoint_position);
  viewer_test_registry::AddTest(tests, "V10", "DrawPath segment pick at midspan does not snap to endpoint position",
                                test_draw_path_segment_pick_midspan_does_not_snap_to_endpoint_position);
}

WIRE_REGISTER_VIEWER_TEST_SUITE(register_pick_policy_tests);

} // namespace
