cmake_minimum_required(VERSION 3.6)
project(modi_ImuRecord)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/../bin)
set(CMAKE_BUILD_TYPE "release")

set(SOURCE_FILES main.cpp)
add_executable(modi_ImuRecord ${SOURCE_FILES})