PROJECT( bundle_adjustment )

add_library( bal_common common.cpp )
add_executable( BAL_BA BAL_BA.cpp)
target_link_libraries( BAL_BA bal_common ${G2O_LIBS} )

add_executable( directBA directBA.cpp )
target_link_libraries( directBA ${OpenCV_LIBS} ${Pangolin_LIBRARIES} ${G2O_LIBS} )