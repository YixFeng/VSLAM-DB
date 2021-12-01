PROJECT( GN_BA )

add_executable( ${PROJECT_NAME} GN-BA.cpp )
target_link_libraries( ${PROJECT_NAME} ${Sophus_LIBRARIES})