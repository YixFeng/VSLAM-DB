CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )
PROJECT( E2Rt )

set( CMAKE_CXX_FLAGS "-std=c++11" )

find_package( Sophus REQUIRED )
include_directories( "/usr/include/eigen3"
                    ${Sophus_INCLUDE_DIRS} )

add_subdirectory( src )