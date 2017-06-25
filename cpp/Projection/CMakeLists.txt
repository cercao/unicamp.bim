cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(learning_1)

find_package(PCL 1.5 REQUIRED)
find_package( OpenCV REQUIRED )

include_directories(${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} ${OpenCV_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (project_inliers project_inliers.cpp)
target_link_libraries (project_inliers ${PCL_LIBRARIES} ${OpenCV_LIBS})

