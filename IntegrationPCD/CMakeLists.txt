cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(tuto-pairwise)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_definitions(-Wno-deprecated -DEIGEN_DONT_VECTORIZE -DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT)

set(PCL_BUILD_TYPE Release)
 
file(GLOB integrationPCD_SRC
    "src/*.h"
    "src/*.cpp"
)
add_executable(integrationPCD ${integrationPCD_SRC})
target_link_libraries (integrationPCD ${PCL_LIBRARIES})
