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
 * This class represents the map of the SLAM system. It is composed by all MapPoints and KeyFrames and
 * a graph relating observations between them
 */

#ifndef SLAM_MAP_H
#define SLAM_MAP_H


#include "Map/MapPoint.h"
#include "Map/KeyFrame.h"

#include <unordered_map>
#include <unordered_set>
#include <memory>

typedef long unsigned int ID;

class Map {
public:
    Map();

    /*
     * Constructor defining the minimum number of common observations between 2 KeyFrames to be
     * considered covisibles
     */
    Map(float minCommonObs);

    /*
     * Inserts a new MapPoint into the map
     */
    void insertMapPoint(std::shared_ptr<MapPoint> pMP);

    /*
     * Inserts a new KeyFrame into the map
     */
    void insertKeyFrame(std::shared_ptr<KeyFrame> pKF);

    /*
     * Removes a MapPoint from the map
     */
    void removeMapPoint(ID id);

    /*
     * Gets the KeyFrame with the given id
     */
    std::shared_ptr<KeyFrame> getKeyFrame(ID id);

    /*
     * Gets the MapPoint with the given id
     */
    std::shared_ptr<MapPoint> getMapPoint(ID id);

    /*
     * Gets all MapPoints of the map
     */
    std::unordered_map<ID,std::shared_ptr<MapPoint>>& getMapPoints();

    /*
     * Gets all KeyFrames of the map
     */
    std::unordered_map<ID,std::shared_ptr<KeyFrame>>& getKeyFrames();

private:

    /*
     * Mapping of the KeyFrame/MapPoint ids and the KeyFrame/MapPoint itself
     */
    std::unordered_map<ID,std::shared_ptr<MapPoint>> mMapPoints_;
    std::unordered_map<ID,std::shared_ptr<KeyFrame>> mKeyFrames_;

    float minCommonObs_;
};


#endif //SLAM_MAP_H
