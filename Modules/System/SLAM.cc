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

SLAM::SLAM(const std::string &settingsFile) {
    //Load settings from file
    cout << "Loading system settings from: " << settingsFile << endl;
    settings_ = Settings(settingsFile);
    cout << settings_ << endl;

    //Create map
    pMap_ = shared_ptr<Map>(new Map(settings_.getMinCommonObs()));

    //Create visualizers
    mapVisualizer_ = shared_ptr<MapVisualizer>(new MapVisualizer(pMap_));

    currFrame_ = Frame(settings.getFeaturesPerImage(),settings.getGridCols(),settings.getGridRows(),
                       settings.getImCols(),settings.getImRows(), settings.getNumberOfScales(), settings.getScaleFactor(),
                       settings.getCalibration(),settings.getDistortionParameters());
    prevFrame_ = Frame(settings.getFeaturesPerImage(),settings.getGridCols(),settings.getGridRows(),
                       settings.getImCols(),settings.getImRows(),settings.getNumberOfScales(), settings.getScaleFactor(),
                       settings.getCalibration(),settings.getDistortionParameters());
}


void SLAM::loadPoints(const std::string &originalFile, const std::string &movedFile) {
    std::ifstream originalFileStream(originalFile);
    if (!originalFileStream.is_open()) {
        std::cerr << "Error opening original points file: " << originalFile << std::endl;
        return;
    }

    std::ifstream movedFileStream(movedFile);
    if (!movedFileStream.is_open()) {
        std::cerr << "Error opening moved points file: " << movedFile << std::endl;
        return;
    }

    originalPoints_.clear();  // Clear previous points if any
    movedPoints_.clear();     // Clear previous points if any

    std::string line;
    while (std::getline(originalFileStream, line)) {
        std::istringstream iss(line);
        float x, y, z;
        if (!(iss >> x >> y >> z)) {
            std::cerr << "Error reading original points file format" << std::endl;
            continue;
        }
        Eigen::Vector3f originalPoint(x, y, z);
        originalPoints_.push_back(originalPoint);

        // Read corresponding moved point
        if (!std::getline(movedFileStream, line)) {
            std::cerr << "Error reading moved points file format" << std::endl;
            continue;
        }
        std::istringstream issMoved(line);
        float xMoved, yMoved, zMoved;
        if (!(issMoved >> xMoved >> yMoved >> zMoved)) {
            std::cerr << "Error reading moved points file format" << std::endl;
            continue;
        }
        Eigen::Vector3f movedPoint(xMoved, yMoved, zMoved);
        movedPoints_.push_back(movedPoint);
    }

    // Close files
    originalFileStream.close();
    movedFileStream.close();

    std::cout << "Loaded " << originalPoints_.size() << " MapPoints from files." << std::endl;
}

void SLAM::setCameraPoses(const Eigen::Vector3f firstCamera, const Eigen::Vector3f secondCamera) {

    // definitions
    shared_ptr<CameraModel> calibration1 = currFrame_->getCalibration();
    shared_ptr<CameraModel> calibration2 = prevFrame_->getCalibration();

    // set camera poses and orientation
    Eigen::Matrix3f R = Eigen::Matrix3f::Identity();
    Sophus::SE3f T1w(R, firstCamera);
    prevFrame_.setPose(T1w);

    R = lookAt(secondCamera, movedPoints_[0]);
    Sophus::SE3f T2w(R, secondCamera);
    currFrame_.setPose(T2w);

    mapVisualizer_->updateCurrentPose(T2w);

    // compute E matrix
    Sophus::SE3f T21 = T2w*T1w.inverse();
    Eigen::Matrix<float,3,3> E = computeEssentialMatrixFromPose(T21);

    createKeyPoints();
    {
    for(size_t i = 0; i < movedPoints_.size(); i++){
    // todos los original points
    Eigen::Vector2f original_p2D;
    Eigen::Vector2f moved_p2D;
    calibration1->project(originalPoints_[i], original_p2D);
    calibration2->project(movedPoints_[i], moved_p2D);

    // añadir ruido gaussiano de 1 pixel de media (solo unidades de pixel)
    prevFrame_.setKeyPoint(original_p2D, i);
    currFrame_.setKeyPoint(moved_p2D, i);

    }

    shared_ptr<KeyFrame> kf0(new KeyFrame(prevFrame_));
    shared_ptr<KeyFrame> kf1(new KeyFrame(currFrame_));

    // promote to keyframes for visualization
    pMap_->insertKeyFrame(kf0);
    pMap_->insertKeyFrame(kf1);


    // auto x1 = currKeyFrame_->getKeyPoint(i).pt;
    // auto x2 = pKF->getKeyPoint(vMatches[i]).pt;

    // Eigen::Vector3f xn1 = calibration1->unproject(x1).normalized();
    // Eigen::Vector3f xn2 = calibration2->unproject(x2).normalized();
    //Eigen::Vector3f x3D;
    //triangulate(xn1, xn2, T1w, T2w, x3D);

    //Eigen::Vector3f ray1 = (E*calibration->unproject(kf1->getKeyPoint(i).pt).transpose()).normalized();
}

Eigen::Matrix3f SLAM::lookAt(const Eigen::Vector3f& camera_pos, const Eigen::Vector3f& target_pos, const Eigen::Vector3f& up_vector = Eigen::Vector3f::UnitY()) {
    Eigen::Vector3f forward = (target_pos - camera_pos).normalized();
    Eigen::Vector3f right = up_vector.cross(forward).normalized();
    Eigen::Vector3f up = forward.cross(right).normalized();
    
    Eigen::Matrix3f rotation;
    rotation.col(0) = right;
    rotation.col(1) = up;
    rotation.col(2) = forward;
    
    return rotation;
}