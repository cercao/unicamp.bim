cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(rg_segmentation)

find_package(PCL 1.5 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (rg_segmentation rg_segmentation.cpp)
target_link_libraries (rg_segmentation ${PCL_LIBRARIES})
