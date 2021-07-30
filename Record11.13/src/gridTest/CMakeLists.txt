cmake_minimum_required(VERSION 3.10.0)
project(gridtest)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/../bin)
##set(CMAKE_BUILD_TYPE "release")
set(CMAKE_BUILD_TYPE "debug")

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

##find_package( PCL REQUIRED COMPONENT common io visualization filters)
find_package( PCL REQUIRED)
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

set(SOURCE_FILES  headTeat.cpp ../grid.h )
## src/0_tartget.cpp RSDecoder/input.cpp src/GetFrame_RS32.cpp
##        src/grid_manage.cpp src/PointCloudManage.cpp src/grid.h src/Auxiliary.cpp
add_executable(gridtest ${SOURCE_FILES})
target_link_libraries(gridtest
        pcap
        ${PCL_LIBRARIES}
        ${OpenCV_LIBS})
