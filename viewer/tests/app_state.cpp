#include "app_state.hpp"
#include "registry.hpp"

namespace {

bool test_selection_helpers_keep_primary_and_deduplicate() {
  ViewerUiState ui_state{};
  ReplaceSelection(ui_state,
                   {
                       {SelectedType::kSpan, 42},
                       {SelectedType::kPole, 7},
                       {SelectedType::kPole, 7},
                       {SelectedType::kNone, wire::core::kInvalidObjectId},
                   });
  return ui_state.selection_items.size() == 2 && ui_state.selected_type == SelectedType::kPole && ui_state.selected_id == 7 &&
         SelectionContains(ui_state, SelectedType::kSpan, 42) && SelectionCountByType(ui_state, SelectedType::kPole) == 1;
}

bool test_set_primary_and_clear_selection_reset_state() {
  ViewerUiState ui_state{};
  SetPrimarySelection(ui_state, SelectedType::kSupportNode, 99);
  if (ui_state.selection_items.size() != 1 || ui_state.selected_type != SelectedType::kSupportNode ||
      ui_state.selected_id != 99) {
    return false;
  }
  ClearSelection(ui_state);
  return ui_state.selection_items.empty() && ui_state.selected_type == SelectedType::kNone &&
         ui_state.selected_id == wire::core::kInvalidObjectId;
}

bool test_auto_recalc_defaults_to_legacy_off() {
  ViewerUiState ui_state{};
  return !ui_state.auto_recalc;
}

void register_app_state_tests(viewer_test_registry::TestRegistry& tests) {
  viewer_test_registry::AddTest(tests, "V07", "Selection helpers keep a deduplicated primary selection",
                                test_selection_helpers_keep_primary_and_deduplicate);
  viewer_test_registry::AddTest(tests, "V08", "Primary selection helper and clear reset viewer selection state",
                                test_set_primary_and_clear_selection_reset_state);
  viewer_test_registry::AddTest(tests, "V23", "Legacy auto recalc is off by default",
                                test_auto_recalc_defaults_to_legacy_off);
}

WIRE_REGISTER_VIEWER_TEST_SUITE(register_app_state_tests);

} // namespace
