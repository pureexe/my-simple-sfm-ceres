cmake_minimum_required(VERSION 2.8)

project(sfm)

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

# helloworld
add_executable(sfm sfm.cc)
target_link_libraries(sfm ${CERES_LIBRARIES})