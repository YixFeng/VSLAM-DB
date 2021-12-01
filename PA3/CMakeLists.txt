cmake_minimum_required( VERSION 2.8 )

project( Draw_Trajectory )

set( CMKAE_BUILD_TYPE Release )
set( CMAKE_CXX_FLAGS "-std=c++14" )

include_directories( "/usr/include/eigen3" )
find_package( Sophus REQUIRED )
find_package( Pangolin REQUIRED )

add_executable( draw_trajectory draw_trajectory.cpp )
target_link_libraries( draw_trajectory ${Sophus_LIBRARIES} ${Pangolin_LIBRARIES} )

add_executable( calculate_error calculate_error.cpp )
target_link_libraries( calculate_error ${Sophus_LIBRARIES} ${Pangolin_LIBRARIES} )