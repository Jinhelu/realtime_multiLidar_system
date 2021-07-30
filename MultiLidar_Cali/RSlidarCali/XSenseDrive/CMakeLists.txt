cmake_minimum_required(VERSION 3.5.0)
project(xsens_mti_driver)

## Compile as C++11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/../bin)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
#find_package( )

## Specify additional locations of header files
## Your package locations should be listed before other locations
#头文件
include_directories(
# include
  lib/xspublic
  include/receive_imu_data
  include/common
)
#库文件
link_directories(
	lib/xspublic/xscontroller 		 
	lib/xspublic/xscommon
 	lib/xspublic/xstypes
)
add_executable(receive_data src/main.cpp)

target_link_libraries(receive_data
	xscontroller
	xscommon
	xstypes
	rt
	dl
)