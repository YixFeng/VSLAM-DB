project( Image_Undistorted )
cmake_minimum_required( VERSION 2.8 )

set( CMAKE_CXX_FLAGS "-std=c++14" )

find_package( OpenCV 3.4 REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_subdirectory( src )