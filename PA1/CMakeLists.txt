cmake_minimum_required(VERSION 2.8)
project(sayhello)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

add_library(hello SHARED src/hello.cpp)

include_directories(${PROJECT_SOURCE_DIR}/include)

INSTALL(TARGETS hello
	LIBRARY DESTINATION /usr/local/lib)

INSTALL(FILES ${PROJECT_SOURCE_DIR}/include/hello.h
	DESTINATION /usr/local/include)

add_executable(${PROJECT_NAME} src/useHello.cpp)
target_link_libraries(${PROJECT_NAME} hello)
