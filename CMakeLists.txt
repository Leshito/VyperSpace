cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(PCL_demo)

# set build type to release
set(CMAKE_BUILD_TYPE "Release")

# explicitly set c++11
set(CMAKE_CXX_STANDARD 11)

# configure PCL
find_package(PCL 1.8.0 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
include_directories(include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#file(GLOB SOURCES src/*.cpp)

add_executable (visualizer src/visual.cpp src/rewtMain.cpp src/CloudVisualizer.cpp src/random_sample_consensus.cpp src/voxel_grid.cpp src/load_pcd.cpp src/plane_seg.cpp src/cluster_extraction.cpp src/passthrough.cpp src/shape_detection.cpp src/correspondence_grouping.cpp src/outputPoints.cpp)
target_link_libraries (visualizer ${PCL_LIBRARIES})

#add_executable (openni2_snapper src/openni2_snapper.cpp)
#target_link_libraries (openni2_snapper ${PCL_LIBRARIES})
