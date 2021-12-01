project( Image_Undistorted )

add_executable( image_undistorted image_undistorted.cpp )
target_link_libraries( image_undistorted ${OpenCV_LIBS} )