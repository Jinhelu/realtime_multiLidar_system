cmake_minimum_required(VERSION 3.5.0)
project(MutiLidarTest )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread -O3")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/../bin)
set(CMAKE_BUILD_TYPE "release")
##set(CMAKE_BUILD_TYPE "debug")

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

##find_package( PCL REQUIRED COMPONENT common io visualization filters)
find_package( PCL REQUIRED)
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

#[[
# 添加cmake模块以使用ceres库
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
# 寻找Ceres库并添加它的头文件
find_package( Ceres REQUIRED )
include_directories( ${CERES_INCLUDE_DIRS} )
]]

#[[
add_library(RSDecoder SHARED RSDecoder/rslidar_decoder.hpp RSDecoder/rslidar_packet.h)
target_link_libraries(RSDecoder
        ${PCL_LIBRARIES})
        ]]

##target_include_directories(RSDecoder PUBLIC RSDecoder)

######################################  PointCloudManage库  #############################################
add_library(PointCloudManage SHARED src/PointCloudManage.cpp)
install(TARGETS PointCloudManage
        LIBRARY DESTINATION ${PROJECT_BINARY_DIR}/../lib)
#[[
target_link_libraries(PointCloudManage
        ${CERES_LIBRARIES}
        )
        ]]

######################################  grid_manage库  #############################################
add_library(grid_manage SHARED src/grid_manage.cpp)
install(TARGETS grid_manage
        LIBRARY DESTINATION ${PROJECT_BINARY_DIR}/../lib)




#####################################  IMU库  ##############################################
#[[
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
        ]]
###################################################################################


add_library(LidarMap SHARED src/MutiLidar.cpp src/MutiLidar.h RSDecoder/input.cpp src/GetFrame_RS.cpp
        src/grid.h src/Auxiliary.cpp
        )
install(TARGETS LidarMap
        LIBRARY DESTINATION ${PROJECT_BINARY_DIR}/../lib)


include_directories(${PROJECT_BINARY_DIR}/../src)
link_directories(${PROJECT_BINARY_DIR}/../lib)


set(SOURCE_FILES src/main.cpp src/LidarMap_t.h)
add_executable(main ${SOURCE_FILES})
target_link_libraries(main
        LidarMap
        PointCloudManage
        grid_manage
        pcap
        ${PCL_LIBRARIES}
        ${OpenCV_LIBS}
        )


#[[
set(SOURCE_FILES RSDecoder/input.cpp src/GetFrame_RS.cpp  src/GetFrame_Velo16.cpp
        src/grid.h src/Auxiliary.cpp src/GetFrame_Velo16.h)
add_executable(Velo16Show ${SOURCE_FILES})
target_link_libraries(Velo16Show
        PointCloudManage
        grid_manage
        PacketDriver
        PacketDecoder
        pcap
        ${PCL_LIBRARIES}
        ${OpenCV_LIBS})
]]


#[[
set(SOURCE_FILES src/0_grid.cpp RSDecoder/input.cpp src/PointCloudManage.cpp
        src/GetFrame_RS.cpp src/grid_manage.cpp)
add_executable(grid ${SOURCE_FILES})
target_link_libraries(grid
        pcap
        ${PCL_LIBRARIES}
        ${OpenCV_LIBS})
]]
