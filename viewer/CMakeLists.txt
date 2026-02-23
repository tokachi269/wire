include(FetchContent)

set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_GAMES OFF CACHE BOOL "" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)

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
)

target_link_libraries(
  wire_viewer
  PRIVATE
  wire_core
  raylib
  imgui_static
  rlimgui_static
)
