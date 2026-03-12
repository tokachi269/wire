set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_GAMES OFF CACHE BOOL "" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)

set(_wire_has_any_local_viewer_dep OFF)
if(WIRE_RAYLIB_SOURCE_DIR OR WIRE_IMGUI_SOURCE_DIR OR WIRE_RLIMGUI_SOURCE_DIR)
  set(_wire_has_any_local_viewer_dep ON)
endif()

if(_wire_has_any_local_viewer_dep)
  if(NOT WIRE_RAYLIB_SOURCE_DIR OR NOT WIRE_IMGUI_SOURCE_DIR OR NOT WIRE_RLIMGUI_SOURCE_DIR)
    message(FATAL_ERROR
      "Offline viewer builds require all local dependency source dirs to be set: "
      "WIRE_RAYLIB_SOURCE_DIR, WIRE_IMGUI_SOURCE_DIR, and WIRE_RLIMGUI_SOURCE_DIR.")
  endif()
  if(NOT EXISTS "${WIRE_RAYLIB_SOURCE_DIR}/CMakeLists.txt")
    message(FATAL_ERROR "WIRE_RAYLIB_SOURCE_DIR does not point to a raylib source tree: ${WIRE_RAYLIB_SOURCE_DIR}")
  endif()
  if(NOT EXISTS "${WIRE_IMGUI_SOURCE_DIR}/imgui.cpp")
    message(FATAL_ERROR "WIRE_IMGUI_SOURCE_DIR does not point to a Dear ImGui source tree: ${WIRE_IMGUI_SOURCE_DIR}")
  endif()
  if(NOT EXISTS "${WIRE_RLIMGUI_SOURCE_DIR}/rlImGui.cpp")
    message(FATAL_ERROR "WIRE_RLIMGUI_SOURCE_DIR does not point to an rlImGui source tree: ${WIRE_RLIMGUI_SOURCE_DIR}")
  endif()

  add_subdirectory("${WIRE_RAYLIB_SOURCE_DIR}" "${CMAKE_BINARY_DIR}/_deps/raylib-local-build" EXCLUDE_FROM_ALL)
  set(imgui_SOURCE_DIR "${WIRE_IMGUI_SOURCE_DIR}")
  set(rlimgui_SOURCE_DIR "${WIRE_RLIMGUI_SOURCE_DIR}")
elseif(WIRE_VIEWER_FETCH_DEPS)
  include(FetchContent)

  FetchContent_Declare(
    raylib
    GIT_REPOSITORY https://github.com/raysan5/raylib.git
    GIT_TAG 5.5
  )
  FetchContent_MakeAvailable(raylib)

  FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG v1.92.1
  )
  FetchContent_MakeAvailable(imgui)

  FetchContent_Declare(
    rlimgui
    GIT_REPOSITORY https://github.com/raylib-extras/rlImGui.git
    GIT_TAG main
  )
  FetchContent_MakeAvailable(rlimgui)
else()
  message(FATAL_ERROR
    "WIRE_BUILD_VIEWER=ON requires viewer dependencies. "
    "Either provide local source directories via WIRE_RAYLIB_SOURCE_DIR, "
    "WIRE_IMGUI_SOURCE_DIR, WIRE_RLIMGUI_SOURCE_DIR, or enable WIRE_VIEWER_FETCH_DEPS=ON.")
endif()

add_library(
  imgui_static
  STATIC
  ${imgui_SOURCE_DIR}/imgui.cpp
  ${imgui_SOURCE_DIR}/imgui_draw.cpp
  ${imgui_SOURCE_DIR}/imgui_tables.cpp
  ${imgui_SOURCE_DIR}/imgui_widgets.cpp
)

target_include_directories(
  imgui_static
  PUBLIC
  ${imgui_SOURCE_DIR}
)

add_library(
  rlimgui_static
  STATIC
  ${rlimgui_SOURCE_DIR}/rlImGui.cpp
)

target_include_directories(
  rlimgui_static
  PUBLIC
  ${rlimgui_SOURCE_DIR}
)

target_link_libraries(
  rlimgui_static
  PUBLIC
  raylib
  imgui_static
)

add_executable(
  wire_viewer
  src/main.cpp
  src/backbone_plane.cpp
  src/draw_path.cpp
  src/host_coords.cpp
  src/panels.cpp
  src/path_pick_policy.cpp
  src/render_overlay.cpp
  src/scene_query.cpp
  src/ui_common.cpp
)

target_link_libraries(
  wire_viewer
  PRIVATE
  wire_core
  raylib
  imgui_static
  rlimgui_static
)

if(MSVC)
  # Avoid the noisy pwsh.exe probe in vcpkg's MSBuild applocal path while
  # keeping DLL deployment enabled via vcpkg's built-in helper.
  set_target_properties(
    wire_viewer
    PROPERTIES
    VS_GLOBAL_VcpkgXUseBuiltInApplocalDeps "true"
  )
endif()

add_executable(
  wire_viewer_tests
  src/backbone_plane.cpp
  src/host_coords.cpp
  src/path_pick_policy.cpp
  tests/app_state.cpp
  tests/backbone_plane.cpp
  tests/host_coords.cpp
  tests/pick_policy.cpp
  tests/registry.cpp
  tests/runner.cpp
)

target_include_directories(
  wire_viewer_tests
  PRIVATE
  src
  tests
)

target_link_libraries(
  wire_viewer_tests
  PRIVATE
  wire_core
  raylib
)

if(MSVC)
  set_target_properties(
    wire_viewer_tests
    PROPERTIES
    VS_GLOBAL_VcpkgXUseBuiltInApplocalDeps "true"
  )
endif()

add_test(NAME wire_viewer_tests COMMAND wire_viewer_tests)
