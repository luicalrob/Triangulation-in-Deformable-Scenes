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

void Map::removeMapPoint(ID id) {
    mMapPoints_.erase(id);
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

void Map::fuseMapPoints(ID mp1, ID mp2) {
    //Decide which point we keep alive and which we kill
    int obs1 = mMapPointObs_[mp1].size();
    int obs2 = mMapPointObs_[mp2].size();

    ID mpToKeep = (obs1 > obs2) ? mp1 : mp2;
    ID mpToDelete = (obs1 > obs2) ? mp2 : mp1;

    removeMapPoint(mpToDelete);
}

int Map::getNumberOfObservations(ID mp) {
    return mMapPointObs_[mp].size();
}