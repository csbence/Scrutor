cmake_minimum_required(VERSION 3.2)
project(RealTimeSearch)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pedantic -std=c++14")

set(SOURCE_FILES
        src/main.cpp
        src/algorithm/AStar.hpp
        src/algorithm/RTAStar.hpp
        src/domain/tile/SlidingTiles.hpp
        )

set(TEST_FILES
        test/test.cpp
        test/algorithm/AStarTest.hpp
        test/domain/tile/SlidingTilesTest.hpp
        )

add_executable(RealTimeSearch ${SOURCE_FILES})
add_executable(RealTimeSearchTest ${TEST_FILES})

find_package(Boost 1.59.0 COMPONENTS thread system REQUIRED)

include_directories(
        ${Boost_INCLUDE_DIRS}
        ${Boost_LIBRARY_DIRS}
        dependencies/
)

target_link_libraries(RealTimeSearch ${Boost_LIBRARIES})
