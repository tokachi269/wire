add_library(
  road
  STATIC
  src/road.cpp
  src/lookup.cpp
  src/generation/regenerate.cpp
  src/generation/segments.cpp
  src/generation/connections.cpp
  src/generation/markings.cpp
  src/generation/emit.cpp
  src/geometry/geometry.cpp
  src/geometry/section.cpp
  src/geometry/junction.cpp
  src/operations/operation_plan.cpp
  src/persistence/road_archive.cpp
)

target_include_directories(
  road
  PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

wire_enable_msvc_target_defaults(road)

if(BUILD_TESTING)
  add_executable(
    road_tests
    tests/road_tests.cpp
  )

  target_link_libraries(
    road_tests
    PRIVATE
    road
  )

  wire_enable_msvc_target_defaults(road_tests)

  add_test(NAME road_tests COMMAND road_tests)

  add_executable(
    road_architecture_contract_tests
    tests/architecture_contract_tests.cpp
  )

  target_link_libraries(
    road_architecture_contract_tests
    PRIVATE
    road
  )

  wire_enable_msvc_target_defaults(road_architecture_contract_tests)

  add_test(NAME road_architecture_contract_tests COMMAND road_architecture_contract_tests)
endif()
