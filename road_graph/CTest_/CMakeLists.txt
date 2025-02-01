cmake_minimum_required(VERSION 3.22)
project(RoadNetworkTests)

set(CMAKE_CXX_STANDARD 20)

include(CTest)

include_directories("${CMAKE_CURRENT_SOURCE_DIR}/../RoadGraph")

add_library(RoadGraph STATIC  "${CMAKE_CURRENT_SOURCE_DIR}/../RoadGraph/RoadGraph.cpp")
add_executable(ctest_road_gr "${CMAKE_CURRENT_SOURCE_DIR}/ctest_road_gr.cpp")

target_link_libraries(ctest_road_gr RoadGraph)

add_test(NAME TEST_reachability_of_point COMMAND ctest_road_gr "1" "4" "Test_reachability_of_poin")
add_test(NAME TEST_Short_Fast COMMAND ctest_road_gr "1" "4" "Test_Short_Fast")







