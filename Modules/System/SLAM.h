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

#include "Visualization/FrameVisualizer.h"
#include "Visualization/MapVisualizer.h"

#include "Tracking/Frame.h"
#include "Map/KeyFrame.h"
#include "Map/Map.h"

#include "sophus/se3.hpp"

#include <opencv2/opencv.hpp>

class SLAM {
public:
    SLAM();

    /*
     * Constructor with the path to the settings file
     */
    SLAM(const std::string& settingsFile);

    /*
     * Load simulation 3D points
     */
    void loadPoints(const std::string &originalFile, const std::string &movedFile);

    /*
     * Define simulation poses in the environment
     */
    void setCameraPoses(const Eigen::Vector3f firstCamera, const Eigen::Vector3f secondCamera);

    /*
     * Project 3D points with a gaussian error
     */
    void createKeyPoints(float reprojErrorDesv);

    /*
     * Mapping of the simulation matches
     */
    void mapping();

    /*
     * Measure the pos & or errors of the poses and the 3D error of the mapPoints
     */
    void measureErrors();

    /*
     * Create camera orientation matrix from two points
     */
    Eigen::Matrix3f lookAt(const Eigen::Vector3f& camera_pos, const Eigen::Vector3f& target_pos, const Eigen::Vector3f& up_vector = Eigen::Vector3f::UnitY());


private:

    /*
     * Settings of the system. Loaded from a file
     */
    Settings settings_;

    /*
     * Map of the SLAM system
     */
    std::shared_ptr<Map> pMap_;
    cv::Mat currIm_;

    Frame prevFrame_, currFrame_;
    std::shared_ptr<KeyFrame> prevKeyFrame_, currKeyFrame_;

    Sophus::SE3f Tcw_;

    std::shared_ptr<CameraModel> calibration1_;
    std::shared_ptr<CameraModel> calibration2_;
    /*
     * Visualizers
     */
    std::shared_ptr<FrameVisualizer> visualizer_;
    std::shared_ptr<MapVisualizer> mapVisualizer_;

    /*
     * Simulation Points
     */
    std::vector<Eigen::Vector3f> originalPoints_;
    std::vector<Eigen::Vector3f> movedPoints_;

};


#endif //SLAM_SLAM_H
