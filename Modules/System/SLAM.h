/**
* This file is part of Mini-SLAM
*
* Copyright (C) 2021 Juan J. Gómez Rodríguez and Juan D. Tardós, University of Zaragoza.
*
* Mini-SLAM is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Mini-SLAM is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with Mini-SLAM.
* If not, see <http://www.gnu.org/licenses/>.
*/

/*
 * Author: Juan J. Gómez Rodríguez (jjgomez@unizar.es)
 *
 * This class is the Mini-SLAM system. It takes care of processing each image and send it to the
 * the tracker and to the mapping
 */

#ifndef SLAM_SLAM_H
#define SLAM_SLAM_H

#include "System/Settings.h"
#include "Mapping/LocalMapping.h"
#include "Tracking/Tracking.h"

#include "Visualization/FrameVisualizer.h"
#include "Visualization/MapVisualizer.h"

#include "Tracking/Frame.h"
#include "Map/KeyFrame.h"
#include "Map/Map.h"

#include "sophus/se3.hpp"
#include <opencv2/opencv.hpp>

#include <fstream>

class SLAM {
public:
    SLAM();

    /*
     * Constructor with the path to the settings file
     */
    SLAM(const std::string& settingsFile);

    /*
     * Process an image. Computes in Tcw the camera pose of the image
     */
    bool processImage(const cv::Mat& im, const cv::Mat &depthIm, Sophus::SE3f& Tcw, int &nKF, int &nMPs, clock_t &timer);

    /*
     * Process simulated images.
     */
    bool processSimulatedImage( int &nMPs, clock_t &timer);

    /*
     * Load simulation 3D points
     */
    void loadPoints(const std::string &originalFile, const std::string &movedFile);

    /*
     * Define simulation poses in the environment
     */
    void setCameraPoses(const Eigen::Vector3f firstCamera, const Eigen::Vector3f secondCamera);

    /*
     * Function to see the expected 3D positions of the points and cameras
     */
    void viusualizeSolution();

    /*
     * Project 3D points with a gaussian error
     */
    void createKeyPoints();

    /*
     * Get solutions depth and create depth measurements with gaussian error
     */
    void getSimulatedDepthMeasurements();

    /*
     * Get real depth measurements from depth images
     */
    void getDepthMeasurements();

    /*
     * Create camera orientation matrix from two points
     */
    Eigen::Matrix3f lookAt(const Eigen::Vector3f& camera_pos, const Eigen::Vector3f& target_pos, const Eigen::Vector3f& up_vector = Eigen::Vector3f::UnitY());

    /*
     * Get camera positions
     */
    Eigen::Vector3f getFirstCameraPos();
    Eigen::Vector3f getSecondCameraPos();

    std::vector<Eigen::Vector3f> getOriginalPoints();

    std::vector<Eigen::Vector3f> getMovedPoints();

    std::vector<int> getInsertedIndexes();

    void stop();


    /*
     * Map of the SLAM system
     */
    std::shared_ptr<Map> pMap_;    
    
    /*
     * Simulation Points
     */
    std::vector<Eigen::Vector3f> originalPoints_;
    std::vector<Eigen::Vector3f> movedPoints_;
    
    std::string filePath_;

    bool drawRaysSelection_;
    bool showSolution_;
    bool stop_;

private:

    /*
     * Converts if needed the image to grayscale
     */
    cv::Mat convertImageToGrayScale(const cv::Mat& im);

    /*
     * Tracker and mapper
     */
    Tracking tracker_;
    LocalMapping mapper_;

    /*
     * Settings of the system. Loaded from a file
     */
    Settings settings_;

    cv::Mat currIm_;
    cv::Mat currDepthIm_;

    Frame prevFrame_, currFrame_;
    std::shared_ptr<KeyFrame> prevKeyFrame_, currKeyFrame_;

    Sophus::SE3f Tc_cref_;
    Sophus::SE3f Tcref_w_;

    std::shared_ptr<CameraModel> prevCalibration_;
    std::shared_ptr<CameraModel> currCalibration_;

    std::vector<int> insertedIndexes_; // 3D points inserted in the map, to compare the positions
    /*
     * Visualizers
     */
    std::shared_ptr<FrameVisualizer> visualizer_;
    std::shared_ptr<MapVisualizer> mapVisualizer_;

    Eigen::Vector3f C1Pose_;
    Eigen::Vector3f C2Pose_;

    float simulatedRepErrorStanDesv_;
    int decimalsRepError_;
    float SimulatedDepthErrorStanDesv_;
    float SimulatedDepthScaleC1_;
    float SimulatedDepthScaleC2_;

    double repBalanceWeight_;
    double arapBalanceWeight_;
    double globalBalanceWeight_;
    double alphaWeight_;
    double betaWeight_;

    std::string OptSelection_;
    std::string OptWeightsSelection_;
    std::string TrianSelection_;
    std::string TrianMethod_;
    std::string TrianLocation_;

    int nOptimizations_;
    int nOptIterations_;

    int NloptnOptimizations_;
    double NloptRelTolerance_;
    double NloptAbsTolerance_;
    double NloptRepLowerBound_;
    double NloptRepUpperBound_;
    double NloptGlobalLowerBound_;
    double NloptGlobalUpperBound_;
    double NloptArapLowerBound_;
    double NloptArapUpperBound_;
    
    bool showScene_;

    std::ofstream outFile_;

    bool bFirstTriang_;         //Flag to check if we have already received the first optimization
    bool firstCall_;
};


#endif //SLAM_SLAM_H