CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )
PROJECT( GN_BA )

set( CMAKE_CXX_FLAGS "-std=c++11" )

find_package( Sophus REQUIRED )
include_directories( "/usr/include/eigen3"
                    ${SOPHUS_INCLUDE_DIRS})

add_subdirectory( src )