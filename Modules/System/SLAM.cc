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

#include "SLAM.h"

using namespace std;

SLAM::SLAM() : tracker_() {
}

SLAM::SLAM(const std::string &settingsFile) {
    //Load settings from file
    cout << "Loading system settings from: " << settingsFile << endl;
    settings_ = Settings(settingsFile);
    cout << settings_ << endl;

    //Create map
    pMap_ = shared_ptr<Map>(new Map(settings_.getMinCommonObs()));

    //Create visualizers
    mapVisualizer_ = shared_ptr<MapVisualizer>(new MapVisualizer(pMap_));

    loadPoints("Data/original_points.csv", "Data/moved_points.csv");

    Sophus::SE3f T1w, T2w;
    std::vector<cv::Point2f> points1, points2;
    std::vector<Eigen::Vector3f> bearing_rays1, bearing_rays2;

    for (size_t i = 0; i < points1.size(); ++i) {
        // Convert image coordinates to normalized bearing rays
        // You may need to normalize the image coordinates before passing them to the function
        // Assuming you have a function to normalize points, e.g., normalizePoint()
        Eigen::Vector3f xn1, xn2;
        normalizePoint(points1[i], xn1);
        normalizePoint(points2[i], xn2);
        bearing_rays1.push_back(xn1);
        bearing_rays2.push_back(xn2);
    }
}


void SLAM::loadPoints(const std::string &originalFile, const std::string &movedFile) {
    std::ifstream originalFileStream(originalFile);
    if (!originalFileStream.is_open()) {
        cerr << "Error opening original points file: " << originalFile << endl;
        return;
    }

    std::ifstream movedFileStream(movedFile);
    if (!movedFileStream.is_open()) {
        cerr << "Error opening moved points file: " << movedFile << endl;
        return;
    }

    // Read points from files and create MapPoints
    std::string line;
    while (std::getline(originalFileStream, line)) {
        std::istringstream iss(line);
        float x, y, z;
        if (!(iss >> x >> y >> z)) {
            cerr << "Error reading original points file format" << endl;
            continue;
        }
        Eigen::Vector3f originalPoint(x, y, z);

        // Read corresponding moved point
        std::getline(movedFileStream, line);
        std::istringstream issMoved(line);
        float xMoved, yMoved, zMoved;
        if (!(issMoved >> xMoved >> yMoved >> zMoved)) {
            cerr << "Error reading moved points file format" << endl;
            continue;
        }
        Eigen::Vector3f movedPoint(xMoved, yMoved, zMoved);

        // Create MapPoint and add to the map
        shared_ptr<MapPoint> pMapPoint(new MapPoint(originalPoint));
        pMapPoint->setWorldPosition(movedPoint); // Set the moved position
        pMap_->addPoint(pMapPoint);
    }

    // Close files
    originalFileStream.close();
    movedFileStream.close();

    cout << "Loaded " << pMap_->getNumPoints() << " MapPoints from files." << endl;
}