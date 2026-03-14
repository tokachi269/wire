add_library(
  wire_core
  STATIC
  src/state/state.cpp
  src/state/templates.cpp
  src/state/inspection.cpp
  src/state/endpoint_refresh_service.cpp
  src/state/override_resolution.cpp
  src/state/template_mutation_service.cpp
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
    /FS
  )
  set_property(
    TARGET wire_core
    PROPERTY MSVC_DEBUG_INFORMATION_FORMAT
             "$<$<CONFIG:Debug,RelWithDebInfo>:Embedded>"
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

function(wire_enable_builtin_vcpkg_applocal target_name)
  if(MSVC)
    # Avoid the noisy pwsh.exe probe in vcpkg's MSBuild applocal path while
    # keeping DLL deployment enabled via vcpkg's built-in helper.
    set_target_properties(
      ${target_name}
      PROPERTIES
      VS_GLOBAL_VcpkgXUseBuiltInApplocalDeps "true"
    )
  endif()
endfunction()

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
    tests/state_services.cpp
    tests/pick.cpp
    tests/workflow.cpp
    tests/api_surface.cpp
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

  if(MSVC)
    target_compile_options(
      wire_core_tests
      PRIVATE
      /MP
      /FS
    )
    set_property(
      TARGET wire_core_tests
      PROPERTY MSVC_DEBUG_INFORMATION_FORMAT
               "$<$<CONFIG:Debug,RelWithDebInfo>:Embedded>"
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

  wire_enable_builtin_vcpkg_applocal(wire_core_tests)

  add_test(NAME wire_core_tests COMMAND wire_core_tests)
endif()
