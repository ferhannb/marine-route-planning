cmake_minimum_required(VERSION 3.16)
project(BosphorusVGPathMPC LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CASADI_ROOT "" CACHE PATH "Root directory of CasADi C++ distribution (contains include/, lib/, bin/)")

set(_casadi_include_hints)
set(_casadi_library_hints)

if(CASADI_ROOT)
  list(APPEND _casadi_include_hints "${CASADI_ROOT}" "${CASADI_ROOT}/include")
  list(APPEND _casadi_library_hints "${CASADI_ROOT}" "${CASADI_ROOT}/lib" "${CASADI_ROOT}/lib64" "${CASADI_ROOT}/bin")
endif()

if(DEFINED ENV{CASADI_ROOT} AND NOT CASADI_ROOT)
  list(APPEND _casadi_include_hints "$ENV{CASADI_ROOT}" "$ENV{CASADI_ROOT}/include")
  list(APPEND _casadi_library_hints "$ENV{CASADI_ROOT}/lib" "$ENV{CASADI_ROOT}/lib64" "$ENV{CASADI_ROOT}/bin")
endif()

if(WIN32)
  list(APPEND _casadi_include_hints "C:/casadi/include")
  list(APPEND _casadi_library_hints "C:/casadi/lib" "C:/casadi/bin")
elseif(APPLE)
  list(APPEND _casadi_include_hints /usr/local/include /opt/homebrew/include)
  list(APPEND _casadi_library_hints /usr/local/lib /opt/homebrew/lib)
else()
  list(APPEND _casadi_include_hints /usr/local/include /usr/include)
  list(APPEND _casadi_library_hints /usr/local/lib /usr/lib /usr/lib/x86_64-linux-gnu)
endif()

find_path(CASADI_INCLUDE_DIR NAMES casadi/casadi.hpp HINTS ${_casadi_include_hints} PATH_SUFFIXES include)
find_library(CASADI_LIBRARY NAMES casadi libcasadi HINTS ${_casadi_library_hints} PATH_SUFFIXES lib lib64 bin)

if(NOT CASADI_INCLUDE_DIR OR NOT CASADI_LIBRARY)
  message(FATAL_ERROR "CasADi C++ library not found. Set CASADI_ROOT or CASADI_INCLUDE_DIR/CASADI_LIBRARY.")
endif()

message(STATUS "CASADI_INCLUDE_DIR=${CASADI_INCLUDE_DIR}")
message(STATUS "CASADI_LIBRARY=${CASADI_LIBRARY}")

set(ROUTE_CORE_SOURCES
    src/route_planner_core.cpp
    src/planner_cli.cpp
    src/planner_geometry.cpp
    src/planner_io.cpp
    src/planner_tss.cpp
)

add_executable(route_planner
    src/main.cpp
    ${ROUTE_CORE_SOURCES}
)
target_include_directories(route_planner PRIVATE src)
target_compile_definitions(route_planner PRIVATE VGPP_PROJECT_ROOT=\"${CMAKE_SOURCE_DIR}\")

add_executable(buffered_land_dataset_builder
    src/buffered_land_dataset_builder.cpp
    src/planner_geometry.cpp
    src/planner_io.cpp
)
target_include_directories(buffered_land_dataset_builder PRIVATE src)
target_compile_definitions(buffered_land_dataset_builder PRIVATE VGPP_PROJECT_ROOT=\"${CMAKE_SOURCE_DIR}\")

set(MPC_SOURCES
    src/rota_optimal_ds.cpp
    src/obstacle_avoidance.cpp
    src/scenario_parser.cpp
)

add_executable(rota_optimal_ds
    src/rota_optimal_ds_main.cpp
    ${MPC_SOURCES}
)
target_include_directories(rota_optimal_ds PRIVATE src ${CASADI_INCLUDE_DIR})
target_compile_definitions(rota_optimal_ds PRIVATE _GLIBCXX_USE_CXX11_ABI=0)
target_link_libraries(rota_optimal_ds PRIVATE ${CASADI_LIBRARY})

add_executable(bosphorus_vg_mpc
    src/bosphorus_vg_mpc_main.cpp
    src/reference_path.cpp
    ${ROUTE_CORE_SOURCES}
    ${MPC_SOURCES}
)
target_include_directories(bosphorus_vg_mpc PRIVATE src ${CASADI_INCLUDE_DIR})
target_compile_definitions(bosphorus_vg_mpc PRIVATE VGPP_PROJECT_ROOT=\"${CMAKE_SOURCE_DIR}\" _GLIBCXX_USE_CXX11_ABI=0)
target_link_libraries(bosphorus_vg_mpc PRIVATE ${CASADI_LIBRARY})

if(UNIX AND NOT APPLE)
  get_filename_component(_casadi_lib_dir "${CASADI_LIBRARY}" DIRECTORY)
  foreach(_target rota_optimal_ds bosphorus_vg_mpc)
    target_link_options(${_target} PRIVATE "-Wl,--disable-new-dtags")
    set_target_properties(${_target} PROPERTIES
      BUILD_RPATH "${_casadi_lib_dir}"
      INSTALL_RPATH "${_casadi_lib_dir}"
    )
  endforeach()
endif()
