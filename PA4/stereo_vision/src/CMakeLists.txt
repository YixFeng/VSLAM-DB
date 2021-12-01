project( Stereo_Vision )

add_executable( stereo_vision stereo_vision.cpp )
target_link_libraries( stereo_vision ${Pangolin_LIBRARIES} ${OpenCV_LIBS} )