project( Stereo_Vision )
cmake_minimum_required( VERSION 2.8 )

set( CMAKE_CXX_FLAGS "-std=c++11" )

find_package( OpenCV 3.4 REQUIRED )
find_package( Pangolin REQUIRED )
include_directories( "/usr/include/eigen3" ${OpenCV_INCLUDE_DIRS} )

add_subdirectory( src )