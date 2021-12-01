PROJECT( OPTICAL_FLOW )
CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )

set( CMAKE_CXX_FLAGS "-std=c++11" )

find_package( OpenCV 4 REQUIRED )
include_directories( "/usr/include/eigen3"
                    ${OpenCV_INCLUDE_DIRS} )

add_subdirectory( src )