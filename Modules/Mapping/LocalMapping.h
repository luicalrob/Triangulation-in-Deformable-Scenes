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
 * Implementation of the Mapping functionalities of Mini-SLAM. It takes KeyFrames and process them so
 * it triangulates new MapPoints, removes duplicated ones and runs a Local Bundl
 */

#ifndef MINI_SLAM_LOCALMAPPING_H
#define MINI_SLAM_LOCALMAPPING_H

#include "Map/KeyFrame.h"
#include "Map/Map.h"
#include "System/Settings.h"

#include "Visualization/FrameVisualizer.h"

#include <memory>
#include <fstream>

class LocalMapping {
public:
    LocalMapping();

    /*
     * Constructor with the SLAM map and settings
     */
    LocalMapping(Settings& settings, std::shared_ptr<Map> pMap);

    /*
     * Does the mapping operative: triangulation, duplication remove and Local Bundle Adjustment
     */
    void doMapping(std::shared_ptr<KeyFrame>& pCurrKeyFrame, int &nMPs);

private:
    /*
     * Removes from the map redundant or wrongly triangulated points
     */
    void mapPointCulling();

    /*
     * Triangulates new MapPoints with the current KeyFrame
     */
    void triangulateNewMapPoints();

    /*
     * Matches MapPoints from the current KeyFrame with the previous ones and checks for duplicates
     */
    void checkDuplicatedMapPoints();

    /*
     * Deformation optimization of triangulated points
     */
    void optimization();

    std::shared_ptr<Map> pMap_;

    std::shared_ptr<KeyFrame> currKeyFrame_;

    std::list<std::shared_ptr<MapPoint>> mlpRecentAddedMapPoints;

    Settings settings_;

    /*
     * Simulation Points
     */
    std::vector<Eigen::Vector3f> originalPoints_;
    std::vector<Eigen::Vector3f> movedPoints_;

    Eigen::Vector3f C1Pose_;
    Eigen::Vector3f C2Pose_;

    std::vector<float> C1PointsDepth;
    std::vector<float> C2PointsDepth;
    std::vector<float> C1DepthMeasurements;
    std::vector<float> C2DepthMeasurements;

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
    bool drawRaysSelection_;
    bool showSolution_;
    bool stop_;

    std::ofstream outFile_;
    std::string filePath_;
};


#endif //MINI_SLAM_LOCALMAPPING_H
