# Minimum CMAKE version required
cmake_minimum_required(VERSION 3.5)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

include_directories(
        ${PROJECT_BINARY_DIR}
        ${EIGEN3_INCLUDE_DIR}
        ${CMAKE_SOURCE_DIR}/Thirdparty/Sophus
        ${SLAM_LIB_INCLUDE_DIR})

# Set executable directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Execution)

# Set the name of the main program and add its source file
set(PROGRAM_NAME simulation)
add_executable(${PROGRAM_NAME} simulation.cc)
target_link_libraries(${PROGRAM_NAME} ${OpenCV_LIBS} slam)

set(PROGRAM_NAME drunkard)
add_executable(${PROGRAM_NAME} drunkard.cc)
target_link_libraries(${PROGRAM_NAME} ${OpenCV_LIBS} slam)

set(PROGRAM_NAME syncolon)
add_executable(${PROGRAM_NAME} syncolon.cc)
target_link_libraries(${PROGRAM_NAME} ${OpenCV_LIBS} slam)
