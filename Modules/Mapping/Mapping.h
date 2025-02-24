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
 * Implementation of the MaMappingpping functionalities of Mini-SLAM. It takes every new image, extracts features
 * and itrs descriptors and computes the camera pose by matching MapPoints and running an only pose optimization.
 *
 * It also takes the decision of whenever we need to insert a new KeyFrame into the map
 */

#ifndef MINI_SLAM_MAPPING_H
#define MINI_SLAM_MAPPING_H

#include "Features/Feature.h"
#include "Features/Descriptor.h"

#include "System/Settings.h"

#include "Mapping/Frame.h"
#include "Mapping/MonocularMapInitializer.h"

#include <Visualization/FrameVisualizer.h>
#include <Visualization/MapVisualizer.h>

#include <sophus/se3.hpp>

#include <opencv2/opencv.hpp>

#include <memory>

class Mapping {
public:
    Mapping();

    /*
     * Constructor with the visualizers, the settings and the map
     */
    Mapping(Settings& settings, std::shared_ptr<FrameVisualizer>& visualizer,
             std::shared_ptr<MapVisualizer>& mapVisualizer, std::shared_ptr<Map> map);

    /*
     * Performs the mapping for an image. Returns true on success
     */
    bool doMapping(const cv::Mat& im, const cv::Mat &depthIm, Sophus::SE3f& Tcw, int &nKF, int &nMPs, clock_t &timer);
    
    /*
     * Does the simulated mapping operation: triangulation and optimization
     */
    void doSimulatedMapping(std::shared_ptr<KeyFrame> &pCurrKeyFrame, std::shared_ptr<KeyFrame> &pPrevKeyFrame, int &nMPs);

    /*
     * Gets the last KeyFrame inserted into the map
     */
    std::shared_ptr<KeyFrame> getLastKeyFrame();

    //Matches between the reference and the current frame
    std::vector<int> vMatches_;
    
    /*
     * Simulation Points
     */
    std::vector<Eigen::Vector3f> originalPoints_;
    std::vector<Eigen::Vector3f> movedPoints_;

private:
    //Extracts features and descriptors in the current image
    void extractFeatures(const cv::Mat& im);

    //Initializes a new map from monocuar 2 views
    bool monocularMapInitialization();

    /*
     * Triangulates new MapPoints in a simulation
     */
    void triangulateSimulatedMapPoints();

    /*
     * Check positive z and minimum parallax of triangulated points
     */
    bool isValidParallax(const Eigen::Vector3f& xn1, const Eigen::Vector3f& xn2, 
                        const Sophus::SE3f& T1w, const Sophus::SE3f& T2w, 
                        const Eigen::Vector3f& x3D_1, const Eigen::Vector3f& x3D_2);

    //Feature and descriptor extractors
    std::shared_ptr<Feature> featExtractor_;
    std::shared_ptr<Descriptor> descExtractor_;

    //Reference and current frame
    Frame currFrame_, refFrame_;
    std::shared_ptr<KeyFrame> currKeyFrame_, refKeyFrame_;

    //Last location a KeyPoint was seen. Only used in the monocular initialization
    std::vector<cv::Point2f> vPrevMatched_;

    //Mapping status
    enum TrackStatus{
        NOT_INITIALIZED = 0,    //No map is initialized -> perform monocular initialization
        GOOD = 1,               //Map is initialized and track was good -> perform camera mapping
        LOST = 2                //Track was lost -> perform relocalization (to be implemented)
    };

    TrackStatus status_;
    bool bFirstIm_;         //Flag to check if we have already received an image

    //Monocular map initializer
    MonocularMapInitializer monoInitializer_;

    //SLAM map
    std::shared_ptr<Map> pMap_;

    Sophus::SE3f Tcw_;

    //Visualizers
    std::shared_ptr<FrameVisualizer> visualizer_;
    std::shared_ptr<MapVisualizer> mapVisualizer_;
    cv::Mat currIm_;

    //Number of features correctly tracked
    int nFeatTracked_;

    bool bInserted;

    //Settings of the system
    Settings settings_;

    float depthLimit_;

    std::string filePath_;
    std::ofstream outFile_;

    std::string TrianMethod_;
    std::string TrianLocation_;

    std::shared_ptr<CameraModel> prevCalibration_, currCalibration_;
};


#endif //MINI_SLAM_MAPPING_H
