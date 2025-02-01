#cmake -G "MinGW Makefiles" - for Window
cmake_minimum_required (VERSION 3.22)

project ("RoadGraphLIB")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


add_library(road_graph_lib STATIC "RoadGraph.cpp" "RoadGraph.h")

target_include_directories(road_graph_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})




