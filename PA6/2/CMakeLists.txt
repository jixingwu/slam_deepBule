cmake_minimum_required(VERSION 2.8)
project(directMethod)

set(${CMAKE_BUILD_TYPE} Release)
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

#opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

#sophus
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

#eigen
include_directories("/usr/include/eigen3")

#pangolin
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})



add_executable(directMethod_node direct_method.cpp)
target_link_libraries(directMethod_node
        ${OpenCV_LIBRARIES}
        ${Pangolin_LIBRARIES}
        ${Sophus_LIBRARIES} )