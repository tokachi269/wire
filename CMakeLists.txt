cmake_minimum_required(VERSION 3.24)

project(wire LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(CTest)

option(WIRE_BUILD_VIEWER "Build lightweight viewer app" ON)

add_subdirectory(core)

if(WIRE_BUILD_VIEWER)
  add_subdirectory(viewer)
endif()
