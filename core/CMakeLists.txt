add_library(
  wire_core
  STATIC
  src/state/state.cpp
  src/state/templates.cpp
  src/pole_orientation_utils.cpp
  src/generation/service.cpp
  src/generation/grouped_spans.cpp
  src/generation/from_backbone.cpp
  src/generation/backbone_prepare.cpp
  src/generation/regenerate.cpp
  src/state/backbone.cpp
  src/validation/validator.cpp
  src/recalc/recalc_pipeline.cpp
)

target_include_directories(
  wire_core
  PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

if(BUILD_TESTING)
  add_executable(
    wire_core_tests
    tests/registry.cpp
    tests/runner.cpp
    tests/helpers.cpp
    tests/editing.cpp
    tests/generation.cpp
    tests/geometry.cpp
    tests/junction.cpp
    tests/bundle_visuals.cpp
    tests/regeneration.cpp
    tests/template_policy.cpp
    tests/pick.cpp
    tests/workflow.cpp
  )

  target_link_libraries(
    wire_core_tests
    PRIVATE
    wire_core
  )

  target_compile_definitions(
    wire_core_tests
    PRIVATE
    WIRE_INTERNAL=1
    WIRE_TEST_SPEC_LEDGER_PATH="${CMAKE_CURRENT_SOURCE_DIR}/tests/spec_ledger.md"
  )

  add_test(NAME wire_core_tests COMMAND wire_core_tests)
endif()


