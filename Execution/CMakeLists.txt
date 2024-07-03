#Miinimum CMAKE version required
cmake_minimum_required(VERSION 3.5)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

include_directories(
        ${PROJECT_BINARY_DIR}
        ${EIGEN3_INCLUDE_DIR}
        ${CMAKE_SOURCE_DIR}/Thirdparty/Sophus
        ${SLAM_LIB_INCLUDE_DIR})

#Set executalble directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Execution)

#Set the name of the Program
set(PROGRAM_NAME main)
add_executable(${PROGRAM_NAME} main.cc)
target_link_libraries(${PROGRAM_NAME} ${OpenCV_LIBS} slam)
