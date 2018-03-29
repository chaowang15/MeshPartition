cmake_minimum_required (VERSION 3.1)

project (MeshPartition)
set (CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "-fpermissive")

include_directories(/usr/include/eigen3 include/heap)

set(SOURCES include/heap/MxHeap.cpp covariance.cpp mesh_partition.cpp main.cpp)

add_executable(MeshPartition ${SOURCES})