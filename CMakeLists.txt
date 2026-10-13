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
  src/mesh_gen.cpp
)
target_include_directories(layerslicer_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(layerslicer_core PUBLIC Threads::Threads)
if(MSVC)
  target_compile_options(layerslicer_core PRIVATE /W4)
else()
  target_compile_options(layerslicer_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

function(layer_slicer_warnings target)
  if(MSVC)
    target_compile_options(${target} PRIVATE /W4)
  else()
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
  endif()
endfunction()

add_executable(gen_models apps/gen_models/main.cpp)
target_link_libraries(gen_models PRIVATE layerslicer_core)
layer_slicer_warnings(gen_models)

add_executable(slicer_cli apps/cli/main.cpp)
target_link_libraries(slicer_cli PRIVATE layerslicer_core)
layer_slicer_warnings(slicer_cli)

if(LAYER_SLICER_BUILD_VIEWER)
  find_package(Qt6 REQUIRED COMPONENTS Widgets)
  set(CMAKE_AUTOMOC ON)
  qt_add_executable(layer_slicer_viewer
    apps/viewer/main.cpp
    apps/viewer/main_window.cpp
    apps/viewer/main_window.h
    apps/viewer/slice_view.cpp
    apps/viewer/slice_view.h
  )
  target_link_libraries(layer_slicer_viewer PRIVATE layerslicer_core Qt6::Widgets)
  set_target_properties(layer_slicer_viewer PROPERTIES WIN32_EXECUTABLE ON)
  layer_slicer_warnings(layer_slicer_viewer)
endif()

if(LAYER_SLICER_BUILD_TESTS)
  include(FetchContent)
  FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.tar.gz
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  )
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
  set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)

  enable_testing()
  add_executable(layer_slicer_tests
    tests/test_stl.cpp
    tests/test_slicing.cpp
    tests/test_paths.cpp
    tests/test_parallel.cpp
  )
  target_link_libraries(layer_slicer_tests PRIVATE layerslicer_core GTest::gtest_main)
  layer_slicer_warnings(layer_slicer_tests)
  include(GoogleTest)
  gtest_discover_tests(layer_slicer_tests DISCOVERY_TIMEOUT 60)
endif()

# Smoke tests for the executables.
if(LAYER_SLICER_BUILD_TESTS)
  add_test(NAME cli_slices_cube COMMAND slicer_cli ${CMAKE_CURRENT_SOURCE_DIR}/models/cube.stl --quiet)
  if(LAYER_SLICER_BUILD_VIEWER)
    add_test(NAME viewer_screenshot
             COMMAND layer_slicer_viewer ${CMAKE_CURRENT_SOURCE_DIR}/models/torus.stl --layer 20
                     --screenshot ${CMAKE_CURRENT_BINARY_DIR}/viewer_smoke.png)
    set_tests_properties(viewer_screenshot PROPERTIES ENVIRONMENT "QT_QPA_PLATFORM=offscreen")
    if(WIN32 AND NOT CMAKE_VERSION VERSION_LESS 3.22)
      get_filename_component(_qt_bin "${Qt6_DIR}/../../../bin" ABSOLUTE)
      set_tests_properties(viewer_screenshot PROPERTIES ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:${_qt_bin}")
    endif()
  endif()
endif()
