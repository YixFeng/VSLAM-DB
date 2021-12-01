CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )
project( Gauss_Newton )

find_package( OpenCV 3.4 REQUIRED )
include_directories( "/usr/include/eigen3" ${OpenCV_INCLUDE_DIRS} )

add_subdirectory( src )