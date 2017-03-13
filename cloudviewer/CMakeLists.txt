cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
 
project(cloudViewer)
 
find_package(PCL 1.2 REQUIRED)
 
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
 
set(PCL_BUILD_TYPE Release)
 
file(GLOB cloudViewer_SRC
    "src/*.h"
    "src/*.cpp"
)
add_executable(cloudViewer ${cloudViewer_SRC})
 
target_link_libraries (cloudViewer ${PCL_LIBRARIES})
