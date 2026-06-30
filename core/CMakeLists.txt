add_library(
  wire_core
  STATIC
  src/state/state.cpp
  src/state/legacy/topology.cpp
  src/state/template/registry.cpp
  src/state/template/ports.cpp
  src/state/core_view.cpp
  src/state/inspection.cpp
  src/state/endpoint_refresh_service.cpp
  src/state/override_resolution.cpp
  src/state/template/update.cpp
  src/pole_orientation_utils.cpp
  src/generation/pole_facing/pole_facing_rules.cpp
  src/generation/backbone/derive.cpp
  src/generation/backbone/out.cpp
  src/generation/backbone/pipeline.cpp
  src/generation/generate_from_backbone.cpp
  src/state/backbone.cpp
  src/validation/validator.cpp
  src/geometry/detail_curve.cpp
  src/geometry/detail_curve_input_resolution.cpp
  src/geometry/detail_curve_postprocess.cpp
  src/geometry/style_context.cpp
  src/geometry/variation.cpp
  src/geometry/curve_support.cpp
  src/state/span_runtime.cpp
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
  find_package(Python3 COMPONENTS Interpreter REQUIRED)

  add_custom_target(
    wire_arch_lint
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
    tests/backbone/fixtures.cpp
    tests/backbone/generation.cpp
    tests/backbone/graph.cpp
    tests/backbone/guard.cpp
    tests/backbone/input.cpp
    tests/backbone/output.cpp
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

  add_dependencies(wire_core_tests wire_arch_lint wire_test_family_lint)

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
  )

  wire_enable_builtin_vcpkg_applocal(wire_core_tests)

  add_test(NAME wire_core_tests COMMAND wire_core_tests)
endif()
