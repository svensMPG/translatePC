cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(TransformResults)

find_package(PCL 1.7 REQUIRED)


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (TransformResults main.cpp file_io.hpp)
target_link_libraries (TransformResults ${PCL_LIBRARIES})
