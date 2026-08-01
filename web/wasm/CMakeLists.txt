cmake_minimum_required(VERSION 3.24)

project(wire_web_wasm LANGUAGES CXX)

if(NOT EMSCRIPTEN)
  message(FATAL_ERROR "Configure this project through emcmake.")
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(WIRE_ENABLE_PCH "Enable core precompiled headers" OFF)

# domains/wire/CMakeLists.txt shares these target hooks with the root MSVC build.
function(wire_enable_msvc_target_defaults target_name)
endfunction()

function(wire_enable_builtin_vcpkg_applocal target_name)
endfunction()

add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../../domains/wire" wire)
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../../domains/road" road)

add_executable(wire_web_core bindings.cpp)
target_link_libraries(wire_web_core PRIVATE wire_core road)
set(WIRE_BUILD_COMMIT "unknown" CACHE STRING "Git commit embedded in the WebAssembly module")
set(WIRE_BUILD_VERSION "unknown" CACHE STRING "Package version embedded in the WebAssembly module")
target_compile_definitions(
  wire_web_core
  PRIVATE
  WIRE_BUILD_COMMIT="${WIRE_BUILD_COMMIT}"
  WIRE_BUILD_VERSION="${WIRE_BUILD_VERSION}"
)

target_link_options(
  wire_web_core
  PRIVATE
  --bind
  "SHELL:-sMODULARIZE=1"
  "SHELL:-sEXPORT_ES6=1"
  "SHELL:-sEXPORT_NAME=createWireModule"
  "SHELL:-sENVIRONMENT=web,node"
  "SHELL:-sALLOW_MEMORY_GROWTH=1"
  "SHELL:-sFILESYSTEM=0"
)

set_target_properties(
  wire_web_core
  PROPERTIES
  OUTPUT_NAME "wire_web_core"
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../src/wasm-generated"
  SUFFIX ".js"
)
