cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(bspline_fitting)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


set(BUILD_surface_on_nurbs 1 CACHE BOOL "Fitting NURBS to point-clouds using openNURBS")


add_executable (bspline_fitting bspline_fitting.cpp)
target_link_libraries (bspline_fitting ${PCL_LIBRARIES})
