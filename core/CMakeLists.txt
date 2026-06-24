add_library(
  wire_core
  STATIC
  src/support_orientation_utils.cpp
  src/state/state.cpp
  src/state/templates.cpp
  src/state/core_view.cpp
  src/state/inspection.cpp
  src/state/endpoint_refresh_service.cpp
  src/state/override_resolution.cpp
  src/state/template_mutation_service.cpp
  src/pole_orientation_utils.cpp
  src/generation/build_backbone_service.cpp
  src/generation/pole_facing/pole_facing_rules.cpp
  src/generation/bb2/pipeline.cpp
  src/generation/generate_from_backbone.cpp
  src/generation/rebuild_from_backbone.cpp
  src/state/backbone.cpp
  src/validation/validator.cpp
  src/recalc/detail_curve.cpp
  src/recalc/detail_curve_input_resolution.cpp
  src/recalc/detail_curve_postprocess.cpp
  src/recalc/style_context.cpp
  src/recalc/variation.cpp
  src/recalc/support_layout_projection.cpp
  src/recalc/support_layout_materialization.cpp
  src/recalc/support_layout_world_geometry.cpp
  src/recalc/recalc_pipeline.cpp
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
  add_executable(
    wire_capture_replay
    tools/capture_replay.cpp
  )

  target_link_libraries(
    wire_capture_replay
    PRIVATE
    wire_core
  )

  wire_enable_msvc_target_defaults(wire_capture_replay)

  if(WIRE_ENABLE_PCH)
    target_precompile_headers(
      wire_capture_replay
      PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src/pch.hpp
    )
  endif()

  wire_enable_builtin_vcpkg_applocal(wire_capture_replay)

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
    tests/state_services.cpp
    tests/pick.cpp
    tests/workflow.cpp
    tests/api_surface.cpp
    tests/bb2.cpp
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
