add_library(
  wire_core
  STATIC
  src/core_state.cpp
)

target_include_directories(
  wire_core
  PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

if(BUILD_TESTING)
  add_executable(
    wire_core_tests
    tests/core_tests.cpp
  )

  target_link_libraries(
    wire_core_tests
    PRIVATE
    wire_core
  )

  add_test(NAME wire_core_tests COMMAND wire_core_tests)
endif()
