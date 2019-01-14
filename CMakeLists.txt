project(VLPGraber)
cmake_minimum_required(VERSION 2.8)
add_compile_options(-std=c++11)
file(COPY VLP-16CalibrationFile.xml DESTINATION ${CMAKE_BINARY_DIR})
add_executable(${PROJECT_NAME}
main.cpp
VLPGrabber.cpp
VLPGrabber.h)

IF(CMAKE_SYSTEM_NAME MATCHES "Windows")
target_link_libraries(${PROJECT_NAME} ws2_32)
ELSE()
target_link_libraries(${PROJECT_NAME}  pthread)
ENDIF()
