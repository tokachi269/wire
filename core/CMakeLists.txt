add_library(
  wire_core
  STATIC
  src/state/state.cpp
  src/state/templates.cpp
  src/state/inspection.cpp
  src/state/endpoint_refresh_service.cpp
  src/state/override_resolution.cpp
  src/pole_orientation_utils.cpp
  src/generation/service.cpp
  src/generation/grouped_spans.cpp
  src/generation/from_backbone.cpp
  src/generation/backbone_prepare.cpp
  src/generation/regenerate.cpp
  src/state/backbone.cpp
  src/validation/validator.cpp
  src/recalc/detail_curve.cpp
  src/recalc/variation.cpp
  src/recalc/recalc_pipeline.cpp
)

option(WIRE_ENABLE_PCH "Enable precompiled headers for core targets" ON)

target_include_directories(
  wire_core
  PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

if(MSVC)
  target_compile_options(
    wire_core
    PRIVATE
    /MP
  )
endif()

if(WIRE_ENABLE_PCH)
  target_precompile_headers(
    wire_core
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/pch.hpp
  )
endif()

add_library(
  wire_core_public_headers_smoke
  OBJECT
  tests/public_headers_smoke.cpp
)

target_include_directories(
  wire_core_public_headers_smoke
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(
  wire_core_public_headers_smoke
  PRIVATE
  cxx_std_20
)

if(BUILD_TESTING)
  add_executable(
    wire_core_tests
    EXCLUDE_FROM_ALL
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
  )

  target_link_libraries(
    wire_core_tests
    PRIVATE
    wire_core
  )

  if(MSVC)
    target_compile_options(
      wire_core_tests
      PRIVATE
      /MP
    )
  endif()

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

  add_test(NAME wire_core_tests COMMAND wire_core_tests)
endif()


