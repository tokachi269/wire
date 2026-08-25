add_library(
  wire_core
  STATIC
  src/state/state.cpp
  src/state/template/registry.cpp
  src/state/template/ports.cpp
  src/state/core_view.cpp
  src/state/inspection.cpp
  src/state/endpoint_refresh_service.cpp
  src/state/port_placement.cpp
  src/state/default_bundle_placement.cpp
  src/state/route_bundle_variation.cpp
  src/state/override_resolution.cpp
  src/state/template/update.cpp
  src/generation/backbone/curve_parts.cpp
  src/generation/backbone/derive.cpp
  src/generation/backbone/derive_span_layout.cpp
  src/generation/backbone/emit_shared.cpp
  src/generation/backbone/out.cpp
  src/generation/backbone/regenerate.cpp
  src/generation/backbone/span_visual_assembly.cpp
  src/generation/backbone/mount_graph.cpp
  src/generation/backbone/model_placement_rules.cpp
  src/generation/backbone/model_assembly.cpp
  src/generation/backbone/row_representation.cpp
  src/generation/backbone/pipeline.cpp
  src/generation/generate_from_backbone.cpp
  src/state/backbone.cpp
  src/validation/validator.cpp
  src/geometry/detail_curve.cpp
  src/geometry/curve/curve.cpp
  src/geometry/detail_curve_input_resolution.cpp
  src/geometry/detail_curve_postprocess.cpp
  src/geometry/style_context.cpp
  src/geometry/variation.cpp
  src/geometry/curve_support.cpp
  src/support/instrumentation.cpp
  src/state/span_runtime.cpp
  src/state/persistence.cpp
)

option(WIRE_ENABLE_PCH "Enable precompiled headers for core targets" ON)

target_include_directories(
  wire_core
  PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

wire_enable_msvc_target_defaults(wire_core)

if(WIRE_ENABLE_PCH)
  target_precompile_headers(
    wire_core
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/pch.hpp
  )
endif()

if(BUILD_TESTING)
  target_compile_definitions(
    wire_core
    PRIVATE
    WIRE_TEST_INSTRUMENTATION=1
  )

  find_package(Python3 COMPONENTS Interpreter REQUIRED)

  set(
    WIRE_BACKBONE_SEMANTICS_CELLS
    "${CMAKE_CURRENT_BINARY_DIR}/backbone_semantics_required_cells.txt"
  )
  add_custom_command(
    OUTPUT ${WIRE_BACKBONE_SEMANTICS_CELLS}
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_SOURCE_DIR}/tools/backbone_semantics.py
      --input ${CMAKE_SOURCE_DIR}/docs/wire/backbone_operation_semantics.md
      --output ${WIRE_BACKBONE_SEMANTICS_CELLS}
    DEPENDS
      ${CMAKE_SOURCE_DIR}/tools/backbone_semantics.py
      ${CMAKE_SOURCE_DIR}/docs/wire/backbone_operation_semantics.md
    VERBATIM
  )
  add_custom_target(
    wire_backbone_semantics_cells
    DEPENDS ${WIRE_BACKBONE_SEMANTICS_CELLS}
  )

  add_custom_target(
    wire_arch_lint
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tools/harness/test_architecture_lint.py
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tools/test_arch_lint.py
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tools/arch_lint.py --root ${CMAKE_SOURCE_DIR}
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    VERBATIM
  )

  add_custom_target(
    wire_test_family_lint
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tools/test_family_lint.py --root ${CMAKE_SOURCE_DIR}
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    VERBATIM
  )

  add_executable(
    wire_core_tests
    tests/registry.cpp
    tests/runner.cpp
    tests/helpers.cpp
    tests/editing.cpp
    tests/geometry.cpp
    tests/bundle_visuals.cpp
    tests/regeneration.cpp
    tests/template_policy.cpp
    tests/state_services.cpp
    tests/pick.cpp
    tests/workflow.cpp
    tests/api_surface.cpp
    tests/persistence.cpp
    tests/backbone/fixtures.cpp
    tests/backbone/generation.cpp
    tests/backbone/graph.cpp
    tests/backbone/guard.cpp
    tests/backbone/input.cpp
    tests/backbone/output.cpp
    tests/backbone/bundle_template_identity.cpp
    tests/backbone/route_bundle_variation.cpp
    tests/backbone/semantics_coverage.cpp
    tests/backbone/semantics_matrix.cpp
    tests/backbone/update.cpp
    tests/backbone/registry.cpp
  )

  if(WIRE_ENABLE_UNITY_BUILD)
    # Measured useful mainly for test-heavy iteration, not for the default clean-core build.
    set_target_properties(
      wire_core_tests
      PROPERTIES
      UNITY_BUILD ON
    )
  endif()

  target_link_libraries(
    wire_core_tests
    PRIVATE
    wire_core
  )

  add_dependencies(
    wire_core_tests
    wire_arch_lint
    wire_backbone_semantics_cells
    wire_test_family_lint
  )

  wire_enable_msvc_target_defaults(wire_core_tests)

  if(WIRE_ENABLE_PCH)
    target_precompile_headers(
      wire_core_tests
      PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src/pch.hpp
    )
  endif()

  target_compile_definitions(
    wire_core_tests
    PRIVATE
    WIRE_INTERNAL=1
    WIRE_TEST_SPEC_LEDGER_PATH="${CMAKE_CURRENT_SOURCE_DIR}/tests/spec_ledger.md"
    WIRE_TEST_BACKBONE_SEMANTICS_CELLS_PATH="${WIRE_BACKBONE_SEMANTICS_CELLS}"
  )

  wire_enable_builtin_vcpkg_applocal(wire_core_tests)

  add_test(NAME wire_core_tests COMMAND wire_core_tests)
endif()
