project( Direct_Method )
CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )

set( CMAKE_BUILD_TYPE Release )
set( CMAKE_CXX_FLAGS "-std=c++14" )

find_package( OpenCV 4.3 REQUIRED )
find_package( Sophus REQUIRED )
find_package( Pangolin REQUIRED )
include_directories( "/usr/include/eigen3"
                    ${OpenCV_INCLUDE_DIRS}  ${Pangolin_INCLUDE_DIRS} ${Sophus_INCLUDE_DIRS} )

add_subdirectory( src )