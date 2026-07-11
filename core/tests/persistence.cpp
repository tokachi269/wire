#include "backbone/fixtures.hpp"
#include "registry.hpp"

#include "wire/core/core_state.hpp"

#include <string>

namespace persistence_tests {
namespace {

bool C750_authoritative_save_is_deterministic_and_changes_after_edit() {
  wire::core::CoreState state;
  wire::core::BackboneSpec first = backbone_tests::line_req(state);
  first.path.polyline = {{0.0, 0.0, 0.0}, {650.0, 0.0, 0.0}};
  first.interval_m = 10.0;
  const auto populated = state.GenerateFromBackboneSpec(first);
  if (!populated.ok || populated.value.generated_pole_ids.size() < 66) {
    return false;
  }

  std::string first_save{};
  std::string repeated_save{};
  const auto saved = state.SerializeAuthoritative(&first_save);
  const auto repeated = state.SerializeAuthoritative(&repeated_save);
  if (!saved.ok || !repeated.ok || first_save.empty() || first_save.rfind("wire_state_v1\n", 0) != 0 ||
      first_save != repeated_save) {
    return false;
  }

  wire::core::BackboneSpec second = backbone_tests::line_req(state);
  second.path.polyline = {{0.0, 100.0, 0.0}, {650.0, 100.0, 0.0}};
  second.interval_m = 10.0;
  const auto edited = state.GenerateFromBackboneSpec(second);
  if (!edited.ok || edited.value.generated_pole_ids.size() < 66) {
    return false;
  }
  std::string edited_save{};
  const auto saved_after_edit = state.SerializeAuthoritative(&edited_save);
  return saved_after_edit.ok && edited_save != first_save;
}

void register_tests(test_registry::TestRegistry& tests) {
  test_registry::AddTest(tests, "C750_authoritative_save_is_deterministic_and_changes_after_edit",
                         "authoritative save is versioned, deterministic, and changes after state edit",
                         "Invariant", false, C750_authoritative_save_is_deterministic_and_changes_after_edit);
}

WIRE_REGISTER_TEST_SUITE(register_tests);

} // namespace
} // namespace persistence_tests
