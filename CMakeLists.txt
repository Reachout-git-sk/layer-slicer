cmake_minimum_required(VERSION 3.16)
project(LayerSlicer VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(LAYER_SLICER_BUILD_VIEWER "Build the Qt viewer" ON)
option(LAYER_SLICER_BUILD_TESTS "Build the GoogleTest suite" ON)

find_package(Threads REQUIRED)

add_library(layerslicer_core STATIC
  src/mesh.cpp
  src/stl_io.cpp
  src/intersect.cpp
  src/geometry.cpp
  src/stitch.cpp
  src/paths.cpp
  src/slicer.cpp
)
target_include_directories(layerslicer_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(layerslicer_core PUBLIC Threads::Threads)
if(MSVC)
  target_compile_options(layerslicer_core PRIVATE /W4)
else()
  target_compile_options(layerslicer_core PRIVATE -Wall -Wextra -Wpedantic)
endif()
