PROJECT( ORB_EXTRACT )

add_executable( orb_extract ORB_Extract.cpp )
target_link_libraries( orb_extract ${OpenCV_LIBS} )