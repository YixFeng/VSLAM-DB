project( Gauss_Newton )

add_executable( gauss_newton gaussnewton.cpp )
target_link_libraries( gauss_newton ${OpenCV_LIBS} )