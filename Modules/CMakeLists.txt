# Minimum CMAKE version required
cmake_minimum_required(VERSION 3.5)

# Find required libraries
find_package(Eigen3 3.1.0 REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Open3D REQUIRED)
find_package(Qhull REQUIRED)
find_package(NLopt REQUIRED)

add_definitions(-DCGAL_USE_BASIC_VIEWER -DQT_NO_KEYWORDS)

if(NOT Open3D_FOUND)
    message(FATAL_ERROR "Open3D not found, please use -DCMAKE_INSTALL_PREFIX=open3d_install_dir")
endif()

# Set the name of the Library
set(LIBRARY_NAME slam)

# Current library
set(root ${CMAKE_CURRENT_SOURCE_DIR})

# Define header and source files of the library
set(HEADER_FILES
        ${root}/Calibration/CameraModel.h
        ${root}/Calibration/PinHole.h
        ${root}/Calibration/KannalaBrandt8.h
        ${root}/DatasetLoader/DrunkardLoader.h
        ${root}/DatasetLoader/RealcolonLoader.h
        ${root}/Features/Descriptor.h
        ${root}/Features/Feature.h
        ${root}/Features/FAST.h
        ${root}/Features/ORB.h
        ${root}/Map/KeyFrame.h
        ${root}/Map/Map.h
        ${root}/Map/MapPoint.h
        ${root}/Matching/DescriptorMatching.h
        ${root}/Optimization/nloptOptimization.h
        ${root}/Optimization/EigenOptimization.h
        ${root}/Optimization/g2oBundleAdjustment.h
        ${root}/Optimization/g2oTypes.h
        ${root}/System/Settings.h
        ${root}/System/SLAM.h
        ${root}/Mapping/Frame.h
        ${root}/Mapping/MonocularMapInitializer.h
        ${root}/Mapping/Mapping.h
        ${root}/Visualization/FrameVisualizer.h
        ${root}/Visualization/MapVisualizer.h
        ${root}/Utils/Geometry.h
        ${root}/Utils/CommonTypes.h
        ${root}/Utils/Conversions.h
        ${root}/Utils/Measurements.h
        ${root}/Utils/Utils.h)

set(SOURCE_FILES
        ${root}/Calibration/PinHole.cc
        ${root}/Calibration/KannalaBrandt8.cc
        ${root}/DatasetLoader/DrunkardLoader.cc
        ${root}/DatasetLoader/RealcolonLoader.cc
        ${root}/Features/FAST.cc
        ${root}/Features/ORB.cc
        ${root}/Map/KeyFrame.cc
        ${root}/Map/Map.cc
        ${root}/Map/MapPoint.cc
        ${root}/Matching/DescriptorMatching.cc
        ${root}/Optimization/nloptOptimization.cc
        ${root}/Optimization/g2oBundleAdjustment.cc
        ${root}/Optimization/g2oTypes.cc
        ${root}/System/Settings.cc
        ${root}/System/SLAM.cc
        ${root}/Mapping/Frame.cc
        ${root}/Mapping/MonocularMapInitializer.cc
        ${root}/Mapping/Mapping.cc
        ${root}/Visualization/FrameVisualizer.cc
        ${root}/Visualization/MapVisualizer.cc
        ${root}/Utils/Geometry.cc
        ${root}/Utils/Conversions.cc
        ${root}/Utils/Measurements.cc
        ${root}/Utils/Utils.cc)

set(SLAM_LIB_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR} PARENT_SCOPE)

# Ensure config.h is generated from config.h.in
configure_file(${CMAKE_SOURCE_DIR}/Thirdparty/g2o/config.h.in
               ${CMAKE_CURRENT_BINARY_DIR}/g2o/config.h)

# Include directories
include_directories(${CMAKE_CURRENT_BINARY_DIR}
                    ${CMAKE_CURRENT_SOURCE_DIR}
                    ${EIGEN3_INCLUDE_DIR}
                    ${CMAKE_SOURCE_DIR}/Thirdparty/Sophus
                    ${CMAKE_SOURCE_DIR}/Thirdparty/g2o
                    ${Pangolin_INCLUDE_DIRS}
                    ${Open3D_INCLUDE_DIRS}
                    ${CMAKE_SOURCE_DIR}/Thirdparty/Qhull
                    ${CMAKE_SOURCE_DIR}/Thirdparty/NLopt)

# Add the library
add_library(${LIBRARY_NAME} SHARED
            ${SOURCE_FILES}
            ${HEADER_FILES})

# Define the link libraries used in the project
target_link_libraries(${LIBRARY_NAME}
                      ${OpenCV_LIBS}
                      ${EIGEN3_LIBS}
                      ${Pangolin_LIBRARIES}
                      ${Open3D_LIBRARIES}
                      ${CMAKE_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_core.so
                      ${CMAKE_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_stuff.so
                      ${CMAKE_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_types_sba.so
                      ${CMAKE_SOURCE_DIR}/Thirdparty/Qhull/build/libqhullcpp.a
                      ${CMAKE_SOURCE_DIR}/Thirdparty/Qhull/build/libqhull_r.so
                      ${CMAKE_SOURCE_DIR}/Thirdparty/NLopt/build/libnlopt.so)


