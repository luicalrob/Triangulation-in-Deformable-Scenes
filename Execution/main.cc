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
 * A demo showing the Mini-SLAM library processing a sequence of the EuRoC dataset
 */

#include "System/SLAM.h"

#include <opencv2/opencv.hpp>

using namespace std;

int main(){
    Eigen::Vector3f firstCamera(0.0, 0.0, 0.0);
    Eigen::Vector3f secondCamera(0.4, 0.0, 0.0);

    //Create SLAM system
    SLAM SLAM("Data/Test.yaml");

    // maybe first lets try without movement
    SLAM.loadPoints("Data/original_points.csv", "Data/original_points.csv");

    SLAM.setCameraPoses(firstCamera, secondCamera);

    SLAM.createKeyPoints(1.0f); // with a desviation in the reprojection error of 1 pixel 

    SLAM.mapping();

    return 0;
}