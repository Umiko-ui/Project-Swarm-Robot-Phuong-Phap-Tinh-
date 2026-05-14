cmake_minimum_required(VERSION 3.14)
project(formation_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

execute_process(
    COMMAND python3 -m pybind11 --cmakedir
    OUTPUT_VARIABLE pybind11_DIR
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

find_package(pybind11 REQUIRED)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/..)

add_library(formation_core
    formation_kernel.cpp
    Spatial_grid.cpp
    ellipse_kernel.cpp
)

target_include_directories(formation_core PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(formation_core PRIVATE pybind11::module)

pybind11_add_module(formation_cpp bindings.cpp)

target_link_libraries(formation_cpp PRIVATE formation_core)