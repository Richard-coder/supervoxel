cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(supervoxel_clustering)

SET(CMAKE_CXX_COMPILER "g++")#设定编译器
SET( CMAKE_BUILD_TYPE Release )#设置build类型
SET( CMAKE_CXX_FLAGS "-O3 -std=c++11" )#使用C++11标准

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/. )#设定头文件目录

ADD_LIBRARY( feature feature.cpp )
target_link_libraries(feature 
${PCL_LIBRARIES})

add_executable (supervoxel_clustering supervoxel_clustering.cpp)
target_link_libraries (supervoxel_clustering 
feature
${PCL_LIBRARIES}) 
