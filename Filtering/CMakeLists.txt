cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
 
project(filtering)
 
find_package(PCL 1.2 REQUIRED)
 
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
 
set(PCL_BUILD_TYPE Release)
 
file(GLOB filtering_SRC
    "src/*.h"
    "src/*.cpp"
)
add_executable(filtering ${filtering_SRC})
 
target_link_libraries (filtering ${PCL_LIBRARIES})
