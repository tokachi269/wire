cmake_minimum_required(VERSION 3.24)

project(wire LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(CTest)

option(WIRE_BUILD_VIEWER "Build lightweight viewer app" ON)
option(WIRE_ENABLE_FORMAT_TARGETS "Enable clang-format helper targets" ON)

add_subdirectory(core)

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
      "${CMAKE_CURRENT_SOURCE_DIR}/core/include/*.hpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/core/src/*.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/core/tests/*.cpp"
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
