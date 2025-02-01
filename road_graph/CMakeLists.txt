#cmake -G "MinGW Makefiles" - for Window
cmake_minimum_required (VERSION 3.22)

project ("RoadGraphUtilite")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(WIN32)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -static")
endif()

add_subdirectory(RoadGraph)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20")

file (GLOB CPPM "*.cpp")

add_executable (${PROJECT_NAME} ${CPPM})

target_link_libraries(${PROJECT_NAME} PRIVATE road_graph_lib)


