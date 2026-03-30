cmake_minimum_required(VERSION 3.16)
project(VGPathPlanning LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Qt5 COMPONENTS Widgets Svg Concurrent REQUIRED)

add_executable(route_planner
    src/main.cpp
    src/planner_cli.cpp
    src/planner_geometry.cpp
    src/planner_io.cpp
    src/planner_tss.cpp
)
target_compile_definitions(route_planner PRIVATE VGPP_PROJECT_ROOT=\"${CMAKE_SOURCE_DIR}\")

add_executable(world_layers_viewer
    src/world_layers_viewer.cpp
    src/world_layers_support.cpp
    src/planner_geometry.cpp
    src/planner_io.cpp
)
target_compile_definitions(world_layers_viewer PRIVATE VGPP_PROJECT_ROOT=\"${CMAKE_SOURCE_DIR}\")

add_executable(world_layers_desktop
    src/world_layers_desktop.cpp
    src/world_layers_support.cpp
    src/planner_geometry.cpp
    src/planner_io.cpp
    src/planner_tss.cpp
)
target_compile_definitions(world_layers_desktop PRIVATE VGPP_PROJECT_ROOT=\"${CMAKE_SOURCE_DIR}\")
target_link_libraries(world_layers_desktop PRIVATE Qt5::Widgets Qt5::Svg Qt5::Concurrent)

add_executable(buffered_land_dataset_builder
    src/buffered_land_dataset_builder.cpp
    src/planner_geometry.cpp
    src/planner_io.cpp
)
target_compile_definitions(buffered_land_dataset_builder PRIVATE VGPP_PROJECT_ROOT=\"${CMAKE_SOURCE_DIR}\")

add_executable(adaptive_grid_debug
    src/adaptive_grid_debug.cpp
)
