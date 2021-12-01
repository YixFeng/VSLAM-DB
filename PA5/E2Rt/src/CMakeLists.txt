PROJECT( E2Rt )

add_executable( ${PROJECT_NAME} E2Rt.cpp )
target_link_libraries( ${PROJECT_NAME} ${SOPHUS_LIBRARIES})