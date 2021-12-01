project( Direct_Method )

add_executable( ${PROJECT_NAME} direct_method.cpp )
target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} ${Sophus_LIBRARIES} ${Pangolin_LIBRARIES} )