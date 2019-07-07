# in PCL/cmake

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(pcd_examples)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

ADD_EXECUTABLE(the_code src/the_code.cpp)
target_link_libraries(the_code ${PCL_LIBRARIES})

# add_subdirectory(source_dir [binary_dir])
# adds subdirectory PCL/src and writes this in PCL/build/examples
ADD_SUBDIRECTORY(src/examples examples)


