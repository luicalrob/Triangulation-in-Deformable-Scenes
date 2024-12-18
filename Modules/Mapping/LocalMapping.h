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

    /*
     * Does the simulated mapping operation: triangulation and optimization
     */
    void doSimulatedMapping(std::shared_ptr<KeyFrame> &pCurrKeyFrame, std::shared_ptr<KeyFrame> &pPrevKeyFrame, int &nMPs);


    /*
     * Simulation Points
     */
    std::vector<Eigen::Vector3f> originalPoints_;
    std::vector<Eigen::Vector3f> movedPoints_;

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
     * Triangulates new MapPoints in a simulation
     */
    void triangulateSimulatedMapPoints();

    /*
     * Matches MapPoints from the current KeyFrame with the previous ones and checks for duplicates
     */
    void checkDuplicatedMapPoints();

    /*
     * Check positive z and minimum parallax of triangulated points
     */
    bool isValidParallax(const Eigen::Vector3f& xn1, const Eigen::Vector3f& xn2, 
                        const Sophus::SE3f& T1w, const Sophus::SE3f& T2w, 
                        const Eigen::Vector3f& x3D_1, const Eigen::Vector3f& x3D_2);

    std::shared_ptr<Map> pMap_;

    std::shared_ptr<KeyFrame> currKeyFrame_;
    std::shared_ptr<KeyFrame> prevKeyFrame_;

    std::list<std::shared_ptr<MapPoint>> mlpRecentAddedMapPoints;

    Settings settings_;

    std::string TrianMethod_;
    std::string TrianLocation_;
};


#endif //MINI_SLAM_LOCALMAPPING_H
