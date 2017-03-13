cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
 
project(removeNoise)
 
find_package(PCL 1.2 REQUIRED)
 
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
 
set(PCL_BUILD_TYPE Release)
 
file(GLOB removeNoise_SRC
    "src/*.h"
    "src/*.cpp"
)
add_executable(removeNoise ${removeNoise_SRC})
 
target_link_libraries (removeNoise ${PCL_LIBRARIES})
