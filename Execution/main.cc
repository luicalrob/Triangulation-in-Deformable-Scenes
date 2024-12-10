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
#include "Utils/Measurements.h"

#include <opencv2/opencv.hpp>

using namespace std;

int main(){
    SLAM SLAM("Data/Test.yaml");

    Eigen::Vector3f firstCamera = SLAM.getFirstCameraPos();
    Eigen::Vector3f secondCamera = SLAM.getSecondCameraPos();

    SLAM.loadPoints("Data/original_points.csv", "Data/moved_points.csv");

    SLAM.setCameraPoses(firstCamera, secondCamera);

    SLAM.getSimulatedDepthMeasurements();

    SLAM.createKeyPoints();

    bool showSolution = SLAM.getShowSolution();


    clock_t timer;
    int nMPs = 0;

    // To visualize solution
    if(showSolution) {
        SLAM.viusualizeSolution();
    } else {
        timer = clock();

        SLAM.processSimulatedImage(nMPs, timer);

        timer = clock() - timer;
    }

    measureRelativeMapErrors(SLAM.pMap_, SLAM.filePath_);
    measureAbsoluteMapErrors(SLAM.pMap_, SLAM.originalPoints_, SLAM.movedPoints_, SLAM.filePath_);

    cout << "[END] Seconds: " << fixed << setprecision(4) << ((float)timer)/CLOCKS_PER_SEC << endl;
    cout << "[END] Number of MapPoints: " << nMPs << endl;

    return 0;
}