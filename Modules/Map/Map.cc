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

#include "Map.h"

#include "Matching/DescriptorMatching.h"

using namespace std;

Map::Map(){}

Map::Map(float minCommonObs){
    minCommonObs_ = minCommonObs;
}

void Map::insertMapPoint(std::shared_ptr<MapPoint> pMP) {
    mMapPoints_[pMP->getId()] = pMP;
}

void Map::insertKeyFrame(std::shared_ptr<KeyFrame> pKF) {
    mKeyFrames_[pKF->getId()] = pKF;
}

void Map::removeMapPoint(ID id) {
    mMapPoints_.erase(id);
}

std::shared_ptr<KeyFrame> Map::getKeyFrame(ID id) {
    if(mKeyFrames_.count(id) != 0)
        return mKeyFrames_[id];
    else
        return nullptr;
}

std::shared_ptr<MapPoint> Map::getMapPoint(ID id) {
    if(mMapPoints_.count(id) != 0)
        return mMapPoints_[id];
    else
        return nullptr;
}

std::unordered_map<ID, std::shared_ptr<MapPoint> > & Map::getMapPoints() {
    return mMapPoints_;
}

std::unordered_map<ID, std::shared_ptr<KeyFrame> > & Map::getKeyFrames() {
    return mKeyFrames_;
}