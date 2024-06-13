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

#include "Map/Map.h"

#include "System/Settings.h"

#include "Visualization/FrameVisualizer.h"
#include "Visualization/MapVisualizer.h"

#include "sophus/se3.hpp"

#include <opencv2/opencv.hpp>

class SLAM {
public:
    SLAM();

    /*
     * Constructor with the path to the settings file
     */
    SLAM(const std::string& settingsFile);

private:

    /*
     * Settings of the system. Loaded from a file
     */
    Settings settings_;

    /*
     * Map of the SLAM system
     */
    std::shared_ptr<Map> pMap_;

    /*
     * Visualizers
     */
    std::shared_ptr<FrameVisualizer> visualizer_;
    std::shared_ptr<MapVisualizer> mapVisualizer_;

};


#endif //SLAM_SLAM_H
