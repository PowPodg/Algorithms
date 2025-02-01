cmake_minimum_required(VERSION 3.22)
project(RoadNetworkTests)

set(CMAKE_CXX_STANDARD 20)

# Включаем поддержку тестирования CTest
include(CTest)

#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}")

# Включаем директорию с заголовочными файлами
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/../RoadGraph")

# Добавляем библиотеки и исполнимые файлы

#add_executable(${PROJECT_NAME} ctest_road_gr.cpp "../ShuntingYard.h" "../ShuntingYard.cpp")
add_library(RoadGraph STATIC  "${CMAKE_CURRENT_SOURCE_DIR}/../RoadGraph/RoadGraph.cpp")
add_executable(ctest_road_gr "${CMAKE_CURRENT_SOURCE_DIR}/ctest_road_gr.cpp")

# Линкуем с библиотекой
target_link_libraries(ctest_road_gr RoadGraph)

# Включаем флаг для отображения вывода в процессе выполнения тестов
#set(CMAKE_TEST_OUTPUT_VERBOSE TRUE)

# Добавляем тест в CTest
#add_test(NAME RoadNetworkTest COMMAND ctest_road_gr)
add_test(NAME TEST_reachability_of_point COMMAND ctest_road_gr "1" "4" "Test_reachability_of_poin")
#add_test(NAME Test_reachability_of_point2 COMMAND ctest_road_gr "2" "3" "Test_reachability_of_poin")

add_test(NAME TEST_Short_Fast COMMAND ctest_road_gr "1" "4" "Test_Short_Fast")
#add_test(NAME Test_findPathIfReachable2 COMMAND ctest_road_gr "2" "4" "Test_findPathIfReachable")
#set_tests_properties(RoadNetworkTest PROPERTIES WILL_FAIL TRUE)
#add_test(Test_reachability_of_poin COMMAND ctest_road_gr, "${CMAKE_CURRENT_SOURCE_DIR}" "1" "Test_reachability_of_poin")








