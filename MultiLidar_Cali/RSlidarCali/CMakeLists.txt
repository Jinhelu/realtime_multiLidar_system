cmake_minimum_required(VERSION 3.5.0)
project(MutiLidarCali )

set(CMAKE_EXE_LINKER_FLAGS " -no-pie")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread -O3")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/../bin)
set(CMAKE_BUILD_TYPE "release")
##set(CMAKE_BUILD_TYPE "debug")

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

##find_package( PCL REQUIRED COMPONENT common io visualization filters)
find_package( PCL REQUIRED)
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

#[[
add_library(RSDecoder SHARED RSDecoder/rslidar_decoder.hpp RSDecoder/rslidar_packet.h)
target_link_libraries(RSDecoder
        ${PCL_LIBRARIES})
        ]]

##target_include_directories(RSDecoder PUBLIC RSDecoder)

######################################  PointCloudManage库  #############################################
add_library(PointCloudManage SHARED src/PointCloudManage.cpp)
target_link_libraries(PointCloudManage
        )


#####################################  IMU库  ##############################################
#头文件
include_directories(
        XSenseDrive/lib/xspublic
)
#库文件
link_directories(
        XSenseDrive/lib/xspublic/xscontroller
        XSenseDrive/lib/xspublic/xscommon
        XSenseDrive/lib/xspublic/xstypes
)
add_library(imu_get_data src/imu_get_data.cpp)

target_link_libraries(imu_get_data
        xscontroller
        xscommon
        xstypes
        rt
        dl
        )
###################################################################################

set(SOURCE_FILES src/MutLidarCali.cpp RSDecoder/input.cpp src/GetFrame_RS.cpp
        src/Auxiliary.cpp src/Auxiliary_IMU.cpp src/Auxiliary_IMU.h src/cali.h src/cali.cpp)
add_executable(MutiLidarCali ${SOURCE_FILES})
target_link_libraries(MutiLidarCali
        imu_get_data
        PointCloudManage
        pcap
        ${PCL_LIBRARIES}
        )

