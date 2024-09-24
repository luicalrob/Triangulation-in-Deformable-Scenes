#Miinimum CMAKE version required
cmake_minimum_required(VERSION 3.5)
project(CV_non_rigid_objects_reconstruction)

#Build type:
#   -Set to RELEASE to benchmark your code
#   -Set to DEBUG to debug your code
set(CMAKE_BUILD_TYPE DEBUG)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

#We require the C++14 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#Compilation flags:
#   -Flag for code optimizations: -O3
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/OpenCV/build")
  set(OpenCV_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/OpenCV/build")
endif()

# Set Open3D_DIR
if(EXISTS "$ENV{HOME}/open3d_install")
  set(Open3D_DIR "$ENV{HOME}/open3d_install/lib/cmake/Open3D")
  list(APPEND CMAKE_PREFIX_PATH "${$ENV{HOME}/open3d_install/lib/cmake}")
endif()

#qhull-8.1-alpha1
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/Qhull/build")
  set(Qhull_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/Qhull/build")
  list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/Qhull/build")
endif()

#Nlopt
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/NLopt/build")
  set(NLopt_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/NLopt/build")
  list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/NLopt/build")
endif()

# Add compile definition to use vendored Ceres
add_definitions(-DG2O_USE_VENDORED_CERES)

add_subdirectory(Modules)
add_subdirectory(Execution)