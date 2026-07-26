add_library(
  road
  STATIC
  src/road.cpp
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
endif()
