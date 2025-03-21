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
 * Implementation of the Map visualizer
 */

#ifndef SLAM_MAPVISUALIZER_H
#define SLAM_MAPVISUALIZER_H

#include "Map/Map.h"
#include "Utils/CommonTypes.h"

#include <pangolin/pangolin.h>

#include <memory>

class MapVisualizer {
public:
    MapVisualizer() = delete;
    MapVisualizer(std::shared_ptr<Map> pMap, const PoseData initialPose = PoseData(), const bool showScene = true);

    /*
     * Updates the visualization of the map
     */
    void update(bool drawRaysSelection = false);

    /*
     * Updates the current pose
     */
    void updateCurrentPose(Sophus::SE3f& currPose);

private:
    std::shared_ptr<Map> pMap_;

    void drawMapPoints();
    void drawKeyFrames();
    void drawRays();
    void drawCurrentPose();
    bool isGray(const cv::Vec4f& color, float th = 0.05f);

    pangolin::View d_cam;
    pangolin::OpenGlRenderState s_cam;
    bool triangulated = false;
    bool createdDisplay = false;

    Sophus::SE3f currPose_;

    bool showScene_;
};


#endif //SLAM_MAPVISUALIZER_H
