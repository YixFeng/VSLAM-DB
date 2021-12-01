PROJECT( OPTICAL_FLOW )

add_executable( optical_flow optical_flow.cpp )
target_link_libraries( optical_flow ${OpenCV_LIBS} )