cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(supervoxel_clustering)

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
