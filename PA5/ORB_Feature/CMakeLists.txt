CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )
PROJECT( ORB_EXTRACT )

set( CMAKE_CXX_FLAGS "-std=c++11" )

find_package( OpenCV 3.4 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_subdirectory( src )
