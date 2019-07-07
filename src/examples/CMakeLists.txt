# find the source file in src/examples and write the executable in build/examples:
ADD_EXECUTABLE(pcd_write pcd_write.cpp)
target_link_libraries(pcd_write ${PCL_LIBRARIES})

ADD_EXECUTABLE(concave_hull_2d concave_hull_2d.cpp)
target_link_libraries(concave_hull_2d ${PCL_LIBRARIES})

ADD_EXECUTABLE(cylinder_segmentation cylinder_segmentation.cpp)
target_link_libraries(cylinder_segmentation ${PCL_LIBRARIES})

ADD_EXECUTABLE(random_sample_consensus random_sample_consensus.cpp)
target_link_libraries(random_sample_consensus ${PCL_LIBRARIES})

ADD_EXECUTABLE(moving_least_squares moving_least_squares.cpp)
target_link_libraries(moving_least_squares ${PCL_LIBRARIES})

ADD_EXECUTABLE(pcl_visualizer_demo pcl_visualizer_demo.cpp)
target_link_libraries(pcl_visualizer_demo ${PCL_LIBRARIES})

ADD_EXECUTABLE(difference_of_normals_based_seg difference_of_normals_based_seg.cpp)
target_link_libraries(difference_of_normals_based_seg ${PCL_LIBRARIES})

#fatal error: pcl/surface/on_nurbs/fitting_surface_tdm.h: No such file or directory --> wahrscheinlich weil bei "sudo make install" die entsprechende Option nicht gesetzt wurde
#ADD_EXECUTABLE(bspline_fitting bspline_fitting.cpp)
#target_link_libraries(bspline_fitting ${PCL_LIBRARIES})

ADD_EXECUTABLE(greedy_projection greedy_projection.cpp)
target_link_libraries(greedy_projection ${PCL_LIBRARIES})

ADD_EXECUTABLE(pass_through_filter pass_through_filter.cpp)
target_link_libraries(pass_through_filter ${PCL_LIBRARIES})
