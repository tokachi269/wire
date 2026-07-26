#include <algorithm>
#include <vector>

#include "path_pick_policy.hpp"
#include "registry.hpp"
#include "city/wire/core_state.hpp"

namespace {

city::wire::PoleTypeId FirstPoleTypeId(const city::wire::CoreState& state) {
  city::wire::PoleTypeId best = city::wire::kInvalidPoleTypeId;
  for (const auto& [id, _] : state.view().pole_types()) {
    if (best == city::wire::kInvalidPoleTypeId || id < best) {
      best = id;
    }
  }
  return best;
}

city::wire::ObjectId CreatePole(city::wire::CoreState& state, const city::wire::Vec3d& pos, city::wire::PoleTypeId type_id,
                                const char* name) {
  city::wire::Transformd tf{};
  tf.position = pos;
  const auto pole = state.AddPole(tf, 10.0, name);
  if (!pole.ok) {
    return city::wire::kInvalidObjectId;
  }
  if (!state.ApplyPoleType(pole.value, type_id).ok) {
    return city::wire::kInvalidObjectId;
  }
  return pole.value;
}

city::wire::ObjectId CreateTemplateSpan(city::wire::CoreState& state, city::wire::PoleTypeId type_id,
                                        city::wire::BundleKind kind) {
  city::wire::BackboneSpec request{};
  request.path.polyline = {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}};
  request.interval_m = 1000.0;
  request.pole_type_id = type_id;
  city::wire::BackboneBundleSpec bundle{};
  bundle.bundle_template_id = kind;
  request.bundles.push_back(bundle);
  const auto result = state.GenerateFromBackboneSpec(request);
  return result.ok && !result.value.generated_span_ids.empty()
             ? result.value.generated_span_ids.front()
             : city::wire::kInvalidObjectId;
}

std::uint32_t TemplateMask(city::wire::BundleKind kind) {
  return (1u << static_cast<unsigned>(kind));
}

bool test_selected_templates_are_not_collapsed_to_first_selected_kind() {
  city::wire::CoreState state;
  const city::wire::CoreView& view = state.view();
  const city::wire::PoleTypeId type_id = FirstPoleTypeId(state);
  if (type_id == city::wire::kInvalidPoleTypeId) {
    return false;
  }
  const city::wire::ObjectId span_id = CreateTemplateSpan(state, type_id, city::wire::BundleKind::kHighVoltage);
  if (span_id == city::wire::kInvalidObjectId) {
    return false;
  }

  city::wire::PickResult pick{};
  pick.hit_kind = city::wire::PickHitKind::kSegment;
  pick.hit_id = span_id;
  const std::uint32_t mask =
      TemplateMask(city::wire::BundleKind::kLowVoltage) | TemplateMask(city::wire::BundleKind::kHighVoltage);
  const std::vector<city::wire::BundleKind> resolved = ResolveTemplateKindsForPathPick(view, mask, pick);
  return resolved.size() == 2 &&
         std::find(resolved.begin(), resolved.end(), city::wire::BundleKind::kLowVoltage) != resolved.end() &&
         std::find(resolved.begin(), resolved.end(), city::wire::BundleKind::kHighVoltage) != resolved.end();
}

bool test_hit_span_template_is_used_when_nothing_selected() {
  city::wire::CoreState state;
  const city::wire::CoreView& view = state.view();
  const city::wire::PoleTypeId type_id = FirstPoleTypeId(state);
  if (type_id == city::wire::kInvalidPoleTypeId) {
    return false;
  }
  const city::wire::ObjectId span_id = CreateTemplateSpan(state, type_id, city::wire::BundleKind::kHighVoltage);
  if (span_id == city::wire::kInvalidObjectId) {
    return false;
  }

  city::wire::PickResult pick{};
  pick.hit_kind = city::wire::PickHitKind::kSegment;
  pick.hit_id = span_id;
  const std::vector<city::wire::BundleKind> resolved = ResolveTemplateKindsForPathPick(view, 0u, pick);
  return resolved.size() == 1 && resolved.front() == city::wire::BundleKind::kHighVoltage;
}

bool test_midair_branch_block_checks_selected_templates() {
  city::wire::CoreState state;
  const city::wire::CoreView& view = state.view();
  const std::vector<city::wire::BundleKind> selected =
      ResolveTemplateKindsForPathPick(view, TemplateMask(city::wire::BundleKind::kLowVoltage), {});
  const std::string blocked = FindMidairBranchBlockedTemplateName(view, selected);
  return selected.size() == 1 && selected.front() == city::wire::BundleKind::kLowVoltage && blocked.empty();
}

bool test_midair_branch_block_finds_disallowed_template() {
  city::wire::CoreState state;
  const city::wire::CoreView& view = state.view();
  const std::uint32_t mask =
      TemplateMask(city::wire::BundleKind::kLowVoltage) | TemplateMask(city::wire::BundleKind::kHighVoltage);
  const std::vector<city::wire::BundleKind> selected = ResolveTemplateKindsForPathPick(view, mask, {});
  const std::string blocked = FindMidairBranchBlockedTemplateName(view, selected);
  return std::find(selected.begin(), selected.end(), city::wire::BundleKind::kHighVoltage) != selected.end() &&
         blocked == "HV_3PH";
}

bool test_draw_path_segment_pick_near_endpoint_snaps_to_endpoint_position() {
  city::wire::CoreState state;
  const city::wire::CoreView& view = state.view();
  const city::wire::PoleTypeId type_id = FirstPoleTypeId(state);
  if (type_id == city::wire::kInvalidPoleTypeId) {
    return false;
  }
  const city::wire::ObjectId pole_a = CreatePole(state, {0.0, 0.0, 0.0}, type_id, "A");
  const city::wire::ObjectId pole_b = CreatePole(state, {10.0, 0.0, 0.0}, type_id, "B");
  if (pole_a == city::wire::kInvalidObjectId || pole_b == city::wire::kInvalidObjectId) {
    return false;
  }

  city::wire::PickResult pick{};
  pick.hit_kind = city::wire::PickHitKind::kSegment;
  pick.has_segment_endpoints = true;
  pick.segment_node_a_id = pole_a;
  pick.segment_node_b_id = pole_b;
  pick.segment_endpoint_a_world = {0.6, 0.0, 8.7};
  pick.segment_endpoint_b_world = {9.4, 0.0, 8.7};
  pick.hit_pos_world = {0.6, 0.0, 0.0};

  const city::wire::PickResult normalized = NormalizeDrawPathPick(view, pick, 1.25);
  return normalized.hit_kind == city::wire::PickHitKind::kNode &&
         normalized.hit_id == pole_a &&
         normalized.hit_pos_world.x == 0.0 &&
         normalized.hit_pos_world.y == 0.0 &&
         normalized.hit_pos_world.z == 0.0;
}

bool test_draw_path_segment_pick_midspan_does_not_snap_to_endpoint_position() {
  city::wire::CoreState state;
  const city::wire::CoreView& view = state.view();
  city::wire::PickResult pick{};
  pick.hit_kind = city::wire::PickHitKind::kSegment;
  pick.has_segment_endpoints = true;
  pick.segment_node_a_id = 10;
  pick.segment_node_b_id = 11;
  pick.segment_endpoint_a_world = {0.0, 0.0, 0.0};
  pick.segment_endpoint_b_world = {10.0, 0.0, 0.0};
  pick.hit_pos_world = {5.0, 0.0, 0.0};

  const city::wire::PickResult normalized = NormalizeDrawPathPick(view, pick, 1.25);
  return normalized.hit_pos_world.x == 5.0 &&
         normalized.hit_pos_world.y == 0.0 &&
         normalized.hit_pos_world.z == 0.0;
}

bool test_ground_hover_near_pole_promotes_to_node_pick() {
  city::wire::CoreState state;
  const city::wire::CoreView& view = state.view();
  const city::wire::PoleTypeId type_id = FirstPoleTypeId(state);
  if (type_id == city::wire::kInvalidPoleTypeId) {
    return false;
  }
  const city::wire::ObjectId pole_id = CreatePole(state, {2.0, -1.0, 0.0}, type_id, "SnapPole");
  if (pole_id == city::wire::kInvalidObjectId) {
    return false;
  }

  const city::wire::PickResult promoted =
      PromoteGroundHoverToNearbyPolePick(view, {2.35, -0.8, 0.0}, 1.0);
  return promoted.hit_kind == city::wire::PickHitKind::kNode &&
         promoted.hit_id == pole_id &&
         promoted.hit_pos_world.x == 2.0 &&
         promoted.hit_pos_world.y == -1.0 &&
         promoted.hit_pos_world.z == 0.0;
}

bool test_canonical_pick_ground_near_pole_promotes_to_node() {
  city::wire::CoreState state;
  const city::wire::CoreView& view = state.view();
  const city::wire::PoleTypeId type_id = FirstPoleTypeId(state);
  if (type_id == city::wire::kInvalidPoleTypeId) {
    return false;
  }
  const city::wire::ObjectId pole_id = CreatePole(state, {2.0, -1.0, 0.0}, type_id, "GroundSnapPole");
  if (pole_id == city::wire::kInvalidObjectId) {
    return false;
  }

  city::wire::PickResult pick{};
  pick.hit_kind = city::wire::PickHitKind::kGround;
  pick.hit_pos_world = {2.35, -0.8, 0.0};
  const city::wire::PickResult canonical =
      CanonicalizeDrawPathPick(view, pick, pick.hit_pos_world, true, 1.0);
  return canonical.hit_kind == city::wire::PickHitKind::kNode &&
         canonical.hit_id == pole_id &&
         canonical.hit_pos_world.x == 2.0 &&
         canonical.hit_pos_world.y == -1.0;
}

bool test_canonical_pick_unresolved_segment_near_pole_promotes_to_node() {
  city::wire::CoreState state;
  const city::wire::CoreView& view = state.view();
  const city::wire::PoleTypeId type_id = FirstPoleTypeId(state);
  if (type_id == city::wire::kInvalidPoleTypeId) {
    return false;
  }
  const city::wire::ObjectId pole_id = CreatePole(state, {4.0, 3.0, 0.0}, type_id, "SegmentSnapPole");
  if (pole_id == city::wire::kInvalidObjectId) {
    return false;
  }

  city::wire::PickResult pick{};
  pick.hit_kind = city::wire::PickHitKind::kSegment;
  pick.hit_pos_world = {4.2, 3.1, 0.0};
  pick.has_segment_endpoints = false;
  const city::wire::PickResult canonical =
      CanonicalizeDrawPathPick(view, pick, {0.0, 0.0, 0.0}, false, 0.5);
  return canonical.hit_kind == city::wire::PickHitKind::kNode &&
         canonical.hit_id == pole_id &&
         canonical.hit_pos_world.x == 4.0 &&
         canonical.hit_pos_world.y == 3.0;
}

bool test_ground_hover_far_from_pole_stays_empty() {
  city::wire::CoreState state;
  const city::wire::CoreView& view = state.view();
  const city::wire::PoleTypeId type_id = FirstPoleTypeId(state);
  if (type_id == city::wire::kInvalidPoleTypeId) {
    return false;
  }
  if (CreatePole(state, {0.0, 0.0, 0.0}, type_id, "FarPole") == city::wire::kInvalidObjectId) {
    return false;
  }

  const city::wire::PickResult promoted =
      PromoteGroundHoverToNearbyPolePick(view, {4.0, 4.0, 0.0}, 0.75);
  return promoted.hit_kind == city::wire::PickHitKind::kEmpty &&
         promoted.hit_id == city::wire::kInvalidObjectId;
}

bool test_canonical_pick_empty_ground_near_pole_promotes_to_node() {
  city::wire::CoreState state;
  const city::wire::CoreView& view = state.view();
  const city::wire::PoleTypeId type_id = FirstPoleTypeId(state);
  if (type_id == city::wire::kInvalidPoleTypeId) {
    return false;
  }
  const city::wire::ObjectId pole_id = CreatePole(state, {1.0, 1.0, 0.0}, type_id, "FallbackPole");
  if (pole_id == city::wire::kInvalidObjectId) {
    return false;
  }

  city::wire::PickResult pick{};
  const city::wire::Vec3d hover{1.2, 1.1, 0.0};
  const city::wire::PickResult canonical =
      CanonicalizeDrawPathPick(view, pick, hover, true, 0.5);
  return canonical.hit_kind == city::wire::PickHitKind::kNode &&
         canonical.hit_id == pole_id;
}

void register_pick_policy_tests(viewer_test_registry::TestRegistry& tests) {
  viewer_test_registry::AddTest(tests, "V01", "Selected DrawPath templates are preserved instead of collapsing to the first kind",
                                test_selected_templates_are_not_collapsed_to_first_selected_kind);
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
  viewer_test_registry::AddTest(tests, "V11", "Ground hover near a pole promotes DrawPath pick to that pole",
                                test_ground_hover_near_pole_promotes_to_node_pick);
  viewer_test_registry::AddTest(tests, "V12", "Ground hover far from poles remains unresolved",
                                test_ground_hover_far_from_pole_stays_empty);
  viewer_test_registry::AddTest(tests, "V13", "Canonical DrawPath ground pick near a pole becomes a node pick",
                                test_canonical_pick_ground_near_pole_promotes_to_node);
  viewer_test_registry::AddTest(tests, "V14", "Canonical DrawPath unresolved segment near a pole becomes a node pick",
                                test_canonical_pick_unresolved_segment_near_pole_promotes_to_node);
  viewer_test_registry::AddTest(tests, "V15", "Canonical DrawPath empty pick with nearby ground pole avoids invalid-node fallback",
                                test_canonical_pick_empty_ground_near_pole_promotes_to_node);
}

WIRE_REGISTER_VIEWER_TEST_SUITE(register_pick_policy_tests);

} // namespace
