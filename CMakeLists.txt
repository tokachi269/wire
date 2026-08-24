cmake_minimum_required(VERSION 3.24)

project(city LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(WIRE_EXPORT_COMPILE_COMMANDS "Export compile_commands.json for clang tooling" ON)
if(WIRE_EXPORT_COMPILE_COMMANDS)
  set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
endif()

include(CTest)

option(WIRE_BUILD_VIEWER "Build lightweight viewer app" ON)
option(WIRE_VIEWER_FETCH_DEPS "Allow viewer to fetch third-party dependencies when local sources are not provided" ON)
option(WIRE_ENABLE_FORMAT_TARGETS "Enable clang-format helper targets" ON)
option(WIRE_ENABLE_CLANG_TIDY_TARGETS "Enable clang-tidy helper targets" ON)
option(WIRE_ENABLE_UML_TARGETS "Enable clang-uml helper targets" ON)
option(WIRE_ENABLE_COMPILER_CACHE "Enable sccache/ccache when available" ON)
option(WIRE_ENABLE_UNITY_BUILD "Enable CMake unity builds primarily for core test iteration targets" OFF)

set(WIRE_RAYLIB_SOURCE_DIR "" CACHE PATH "Optional local raylib source directory for offline viewer builds")
set(WIRE_IMGUI_SOURCE_DIR "" CACHE PATH "Optional local Dear ImGui source directory for offline viewer builds")
set(WIRE_RLIMGUI_SOURCE_DIR "" CACHE PATH "Optional local rlImGui source directory for offline viewer builds")
set(WIRE_COMPILER_CACHE_PROGRAM "" CACHE FILEPATH "Optional compiler launcher executable (for example sccache or ccache)")
set(WIRE_CLANG_UML_PROGRAM "" CACHE FILEPATH "Optional clang-uml executable path")

function(wire_configure_compiler_launcher)
  if(NOT WIRE_ENABLE_COMPILER_CACHE)
    return()
  endif()

  if(CMAKE_C_COMPILER_LAUNCHER OR CMAKE_CXX_COMPILER_LAUNCHER)
    message(STATUS "Compiler launcher already configured externally.")
    return()
  endif()

  set(_wire_compiler_launcher "${WIRE_COMPILER_CACHE_PROGRAM}")
  if(NOT _wire_compiler_launcher)
    find_program(_wire_compiler_launcher NAMES sccache ccache)
  endif()

  if(_wire_compiler_launcher)
    set(CMAKE_C_COMPILER_LAUNCHER "${_wire_compiler_launcher}" CACHE STRING "Compiler launcher" FORCE)
    set(CMAKE_CXX_COMPILER_LAUNCHER "${_wire_compiler_launcher}" CACHE STRING "Compiler launcher" FORCE)
    message(STATUS "Using compiler launcher: ${_wire_compiler_launcher}")
  else()
    message(STATUS "WIRE_ENABLE_COMPILER_CACHE=ON but no sccache/ccache executable was found.")
  endif()
endfunction()

function(wire_find_clang_uml out_var)
  set(_wire_clang_uml_program "${WIRE_CLANG_UML_PROGRAM}")
  if(NOT _wire_clang_uml_program AND EXISTS "C:/Program Files/clang-uml/bin/clang-uml.exe")
    set(_wire_clang_uml_program "C:/Program Files/clang-uml/bin/clang-uml.exe")
  endif()
  if(NOT _wire_clang_uml_program)
    find_program(
      _wire_clang_uml_program
      NAMES clang-uml clang-uml.exe
      HINTS
        "$ENV{LOCALAPPDATA}/Microsoft/WinGet/Links"
        "C:/Program Files/clang-uml"
        "C:/Program Files/clang-uml/bin"
    )
  endif()

  if(NOT _wire_clang_uml_program AND WIN32 AND DEFINED ENV{LOCALAPPDATA})
    file(
      GLOB_RECURSE _wire_clang_uml_candidates
      LIST_DIRECTORIES FALSE
      "$ENV{LOCALAPPDATA}/Microsoft/WinGet/Packages/*clang-uml*/*.exe"
      "$ENV{LOCALAPPDATA}/Programs/*clang-uml*/*.exe"
    )
    list(FILTER _wire_clang_uml_candidates INCLUDE REGEX "([/\\\\])clang-uml\\.exe$")
    list(LENGTH _wire_clang_uml_candidates _wire_clang_uml_candidate_count)
    if(_wire_clang_uml_candidate_count GREATER 0)
      list(GET _wire_clang_uml_candidates 0 _wire_clang_uml_program)
    endif()
  endif()

  set(${out_var} "${_wire_clang_uml_program}" PARENT_SCOPE)
endfunction()

function(wire_enable_msvc_target_defaults target_name)
  if(MSVC)
    target_compile_options(
      ${target_name}
      PRIVATE
      /EHsc
      /MP
      /FS
    )
    set_property(
      TARGET ${target_name}
      PROPERTY MSVC_DEBUG_INFORMATION_FORMAT
               "$<$<CONFIG:Debug,RelWithDebInfo>:Embedded>"
    )
  endif()
endfunction()

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

wire_configure_compiler_launcher()

add_subdirectory(domains/wire)
add_subdirectory(domains/road)

if(WIRE_BUILD_VIEWER)
  add_subdirectory(viewer)
endif()

if(WIRE_ENABLE_FORMAT_TARGETS)
  find_program(
    CLANG_FORMAT_EXE
    NAMES clang-format
    HINTS
      "C:/Program Files/Microsoft Visual Studio/2022/Community/VC/Tools/Llvm/bin"
      "C:/Program Files/Microsoft Visual Studio/2022/Community/VC/Tools/Llvm/x64/bin"
      "C:/Program Files/Microsoft Visual Studio/2022/Community/VC/Tools/Llvm/ARM64/bin"
      "C:/Program Files (x86)/Microsoft Visual Studio/2019/Community/VC/Tools/Llvm/bin"
  )

  if(CLANG_FORMAT_EXE)
    file(GLOB_RECURSE WIRE_FORMAT_FILES
      "${CMAKE_CURRENT_SOURCE_DIR}/domains/wire/include/*.hpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/domains/wire/src/*.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/domains/wire/tests/*.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/domains/road/include/*.hpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/domains/road/src/*.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/domains/road/tests/*.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/viewer/src/*.cpp"
    )

    add_custom_target(
      format-check
      COMMAND ${CLANG_FORMAT_EXE} --dry-run -Werror ${WIRE_FORMAT_FILES}
      COMMENT "Checking C/C++ formatting with clang-format"
      VERBATIM
    )

    add_custom_target(
      format
      COMMAND ${CLANG_FORMAT_EXE} -i ${WIRE_FORMAT_FILES}
      COMMENT "Formatting C/C++ sources with clang-format"
      VERBATIM
    )
  else()
    message(WARNING "clang-format was not found; 'format' and 'format-check' targets are disabled.")
  endif()
endif()

if(WIRE_ENABLE_CLANG_TIDY_TARGETS)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  find_program(
    CLANG_TIDY_EXE
    NAMES clang-tidy clang-tidy.exe
    HINTS
      "C:/Program Files/Microsoft Visual Studio/18/Community/VC/Tools/Llvm/bin"
      "C:/Program Files/Microsoft Visual Studio/18/Community/VC/Tools/Llvm/x64/bin"
      "C:/Program Files/Microsoft Visual Studio/18/Community/VC/Tools/Llvm/ARM64/bin"
      "C:/Program Files/Microsoft Visual Studio/2022/Community/VC/Tools/Llvm/bin"
      "C:/Program Files/Microsoft Visual Studio/2022/Community/VC/Tools/Llvm/x64/bin"
      "C:/Program Files/Microsoft Visual Studio/2022/Community/VC/Tools/Llvm/ARM64/bin"
      "C:/Program Files (x86)/Microsoft Visual Studio/2019/Community/VC/Tools/Llvm/bin"
  )

  if(CLANG_TIDY_EXE)
    add_custom_target(
      tidy-check
      COMMAND
        ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tools/clang_tidy_check.py
        --build-dir ${CMAKE_BINARY_DIR}
        --clang-tidy ${CLANG_TIDY_EXE}
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
      COMMENT "Checking C/C++ sources with clang-tidy"
      USES_TERMINAL
      VERBATIM
    )
  else()
    message(WARNING "clang-tidy was not found; 'tidy-check' target is disabled.")
  endif()
endif()

if(WIRE_ENABLE_UML_TARGETS)
  wire_find_clang_uml(CLANG_UML_EXE)

  if(CLANG_UML_EXE)
    find_program(WIRE_POWERSHELL_EXE NAMES pwsh powershell)
    if(WIRE_POWERSHELL_EXE)
      set(WIRE_CLANG_UML_DB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/build/clang-uml-db")

      add_custom_target(
        uml-db
        COMMAND
          ${WIRE_POWERSHELL_EXE}
          -NoProfile
          -ExecutionPolicy Bypass
          -File ${CMAKE_CURRENT_SOURCE_DIR}/tools/prepare_clang_uml_compile_db.ps1
          -WorkspaceRoot ${CMAKE_CURRENT_SOURCE_DIR}
          -OutputDir ${WIRE_CLANG_UML_DB_DIR}
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        COMMENT "Preparing clang-uml compilation database"
        VERBATIM
      )

      add_custom_target(
        uml-list
        COMMAND
          ${WIRE_POWERSHELL_EXE}
          -NoProfile
          -ExecutionPolicy Bypass
          -File ${CMAKE_CURRENT_SOURCE_DIR}/tools/prepare_clang_uml_compile_db.ps1
          -WorkspaceRoot ${CMAKE_CURRENT_SOURCE_DIR}
          -OutputDir ${WIRE_CLANG_UML_DB_DIR}
        COMMAND ${CLANG_UML_EXE} -c ${CMAKE_CURRENT_SOURCE_DIR}/.clang-uml -l
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        COMMENT "Listing clang-uml diagrams"
        VERBATIM
      )

      add_custom_target(
        uml
        COMMAND
          ${WIRE_POWERSHELL_EXE}
          -NoProfile
          -ExecutionPolicy Bypass
          -File ${CMAKE_CURRENT_SOURCE_DIR}/tools/prepare_clang_uml_compile_db.ps1
          -WorkspaceRoot ${CMAKE_CURRENT_SOURCE_DIR}
          -OutputDir ${WIRE_CLANG_UML_DB_DIR}
        COMMAND ${CLANG_UML_EXE} -c ${CMAKE_CURRENT_SOURCE_DIR}/.clang-uml -p
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        COMMENT "Generating UML diagrams with clang-uml"
        VERBATIM
      )
    else()
      message(STATUS "PowerShell was not found; 'uml', 'uml-list', and 'uml-db' targets are disabled.")
    endif()
  else()
    message(STATUS "clang-uml was not found; 'uml' and 'uml-list' targets are disabled. Set WIRE_CLANG_UML_PROGRAM to override the executable path.")
  endif()
endif()
