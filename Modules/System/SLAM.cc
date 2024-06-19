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

#include "Map/KeyFrame.h"
#include "Map/MapPoint.h"
#include "Optimization/g2oBundleAdjustment.h"
#include "Utils/Geometry.h"


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

    currFrame_ = Frame(settings_.getFeaturesPerImage(),settings_.getGridCols(),settings_.getGridRows(),
                       settings_.getImCols(),settings_.getImRows(), settings_.getNumberOfScales(), settings_.getScaleFactor(),
                       settings_.getCalibration(),settings_.getDistortionParameters());
    prevFrame_ = Frame(settings_.getFeaturesPerImage(),settings_.getGridCols(),settings_.getGridRows(),
                       settings_.getImCols(),settings_.getImRows(),settings_.getNumberOfScales(), settings_.getScaleFactor(),
                       settings_.getCalibration(),settings_.getDistortionParameters());

    calibration1_ = currFrame_.getCalibration();
    calibration2_ = prevFrame_.getCalibration();
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
    // set camera poses and orientation
    Eigen::Matrix3f R = Eigen::Matrix3f::Identity();
    Sophus::SE3f T1w(R, firstCamera);
    prevFrame_.setPose(T1w);

    R = lookAt(secondCamera, movedPoints_[0]);
    Sophus::SE3f T2w(R, secondCamera);
    currFrame_.setPose(T2w);
    Tcw_ = T1w;

    mapVisualizer_->updateCurrentPose(T2w);
}

void SLAM::createKeyPoints(float reprojErrorDesv) {
    std::default_random_engine generator;
    std::normal_distribution<float> distribution(0.0f, reprojErrorDesv);
    
    for(size_t i = 0; i < movedPoints_.size(); ++i) {
        cv::Point2f original_p2D;
        cv::Point2f moved_p2D;
        
        original_p2D = calibration1_->project(originalPoints_[i]);
        moved_p2D = calibration2_->project(originalPoints_[i]);

        // Add Gaussian reprojection noise in units of pixels
        original_p2D.x += std::round(distribution(generator));
        original_p2D.y += std::round(distribution(generator));
        moved_p2D.x += std::round(distribution(generator));
        moved_p2D.y += std::round(distribution(generator));

        cv::KeyPoint original_keypoint(original_p2D, 1.0f); // 1.0f is the size of the keypoint
        cv::KeyPoint moved_keypoint(moved_p2D, 1.0f);

        prevFrame_.setKeyPoint(original_keypoint, i);
        currFrame_.setKeyPoint(moved_keypoint, i);
    }

    // Promote to keyframes for visualization
    prevKeyFrame_ = std::make_shared<KeyFrame>(prevFrame_);
    currKeyFrame_ = std::make_shared<KeyFrame>(currFrame_);
    pMap_->insertKeyFrame(prevKeyFrame_);
    pMap_->insertKeyFrame(currKeyFrame_);
}

void SLAM::mapping() {
    int nTriangulated = 0;

    // Each moved point correspond with the point in the same index in the original vector
    vector<int> vMatches(currKeyFrame_->getMapPoints().size()); //movedPoints_.size()?
    int nMatches = movedPoints_.size();

    std::cout << "Number of keypoints: " << nMatches << std::endl;

    Sophus::SE3f T1w = prevFrame_.getPose();
    Sophus::SE3f T2w = currFrame_.getPose();

    //Try to triangulate a new MapPoint with each match
    for(size_t i = 0; i < nMatches; i++){ //vMatches.size()
        // if(vMatches[i] != -1){
        auto x1 = currKeyFrame_->getKeyPoint(i).pt;
        auto x2 = prevKeyFrame_->getKeyPoint(i).pt; // vMatches[i] si las parejas no fuesen ordenadas

        Eigen::Vector3f xn1 = calibration1_->unproject(x1).normalized();
        Eigen::Vector3f xn2 = calibration2_->unproject(x2).normalized();
        Eigen::Vector3f x3D;

        triangulate(xn1, xn2, T1w, T2w, x3D);

        //Check positive depth
        auto x_1 = T1w * x3D;
        auto x_2 = T2w * x3D;
        if(x_1[2] < 0.0 || x_2[2] < 0.0) continue;

        //Check parallax
        auto ray1 = (T1w.inverse().rotationMatrix() * xn1).normalized();
        auto ray2 = (T2w.inverse().rotationMatrix() * xn2).normalized();
        auto parallax = cosRayParallax(ray1, ray2);

        if(parallax > settings_.getMinCos()) continue;

        //Check reprojection error

        Eigen::Vector2f p_p1;
        Eigen::Vector2f p_p2;
        calibration1_->project(T1w*x3D, p_p1);
        calibration2_->project(T2w*x3D, p_p2);

        cv::Point2f cv_p1(p_p1[0], p_p1[1]);
        cv::Point2f cv_p2(p_p2[0], p_p2[1]);

        auto e1 = squaredReprojectionError(x1, cv_p1);
        auto e2 = squaredReprojectionError(x2, cv_p2);

        if(e1 > 5.991 || e2 > 5.991) continue;

        std::shared_ptr<MapPoint> map_point(new MapPoint(x3D));

        pMap_->insertMapPoint(map_point);

        pMap_->addObservation(currKeyFrame_->getId(), map_point->getId(), i);
        pMap_->addObservation(prevKeyFrame_->getId(), map_point->getId(), i); // vMatches[i] si las parejas no fuesen ordenadas

        currKeyFrame_->setMapPoint(i, map_point);
        prevKeyFrame_->setMapPoint(i, map_point); // vMatches[i]?

        nTriangulated++;
        // }
    }

    std::cout << "Triangulated " << nTriangulated << " MapPoints." << std::endl;

    // visualize
    // visualizer_->drawCurrentFrame(currFrame_);
    // visualizer_->drawCurrentFeatures(currFrame_.getKeyPointsDistorted(),currIm_);
    // visualizer_->drawFrameMatches(currFrame_.getKeyPointsDistorted(),currIm_,vMatches_);
    // mapVisualizer_->update();
    // mapVisualizer_->updateCurrentPose(Tcw_);

    // stop
    //Uncomment for step by step execution (pressing esc key)
    cv::namedWindow("Test Window"); 
    std::cout << "Press esc to continue... " << std::endl;
    while((cv::waitKey(10) & 0xEFFFFF) != 27){
        mapVisualizer_->update();
    }

    // correct reporjection error
    localBundleAdjustment(pMap_.get(),currKeyFrame_->getId());

    std::cout << "Bundle adjustment completed " << std::endl;

    // visualize
    // visualizer_->drawCurrentFrame(currFrame_);
    // visualizer_->drawCurrentFeatures(currFrame_.getKeyPointsDistorted(),currIm_);
    // visualizer_->drawFrameMatches(currFrame_.getKeyPointsDistorted(),currIm_,vMatches_);
    mapVisualizer_->update();
    mapVisualizer_->updateCurrentPose(Tcw_);
}

void SLAM::measureErrors() {

    Sophus::SE3f T1w = prevFrame_.getPose();
    Sophus::SE3f T2w = currFrame_.getPose();

    Sophus::SE3f T1w_corrected = pMap_->getKeyFrame(0)->getPose();
    Sophus::SE3f T2w_corrected = pMap_->getKeyFrame(1)->getPose();

    // Show position and orientation errors in pose 1
    Eigen::Vector3f position_error1 = T1w_corrected.translation() - T1w.translation();
    Eigen::Vector3f orientation_error1 = (T1w_corrected.so3().inverse() * T1w.so3()).log();

    std::cout << "\nError in position 1:\n";
    std::cout << "x: " << position_error1.x() << " y: " << position_error1.y() << " z: " << position_error1.z() << std::endl;
    std::cout << "Error in orientation 1:\n";
    std::cout << "x: " << orientation_error1.x() << " y: " << orientation_error1.y() << " z: " << orientation_error1.z() << std::endl;

    // Show position and orientation errors in pose 2
    Eigen::Vector3f position_error2 = T2w_corrected.translation() - T2w.translation();
    Eigen::Vector3f orientation_error2 = (T2w_corrected.so3().inverse() * T2w.so3()).log();

    std::cout << "\nError in position 2:\n";
    std::cout << "x: " << position_error2.x() << " y: " << position_error2.y() << " z: " << position_error2.z() << std::endl;
    std::cout << "Error in orientation 2:\n";
    std::cout << "x: " << orientation_error2.x() << " y: " << orientation_error2.y() << " z: " << orientation_error2.z() << std::endl;

    // 3D Error measurement in map points
    std::unordered_map<ID, std::shared_ptr<MapPoint>> mapPoints_corrected = pMap_->getMapPoints();

    float total_error = 0.0f;
    int point_count = 0;
    for (const auto& [id, mapPoint] : mapPoints_corrected) {
        Eigen::Vector3f corrected_position = mapPoint->getWorldPosition();
        Eigen::Vector3f original_position = originalPoints_[id]; 

        Eigen::Vector3f point_error = corrected_position - original_position;
        float error_magnitude = point_error.norm();
        total_error += error_magnitude;
        point_count++;

        // std::cout << "\nError for point " << id << ":\n";
        // std::cout << "x: " << point_error.x() << " y: " << point_error.y() << " z: " << point_error.z() << std::endl;
    }

    if (point_count > 0) {
        float average_error = total_error / point_count;
        std::cout << "\nTotal error in 3D: " << total_error << std::endl;
        std::cout << "Average error in 3D: " << average_error << std::endl;
    } else {
        std::cout << "No points to compare." << std::endl;
    }

    // stop
    //Uncomment for step by step execution (pressing esc key)
    std::cout << "Press esc to continue... " << std::endl;
    while((cv::waitKey(10) & 0xEFFFFF) != 27){
        mapVisualizer_->update();
    }
}

void SLAM::measureErrors() {

    Sophus::SE3f T1w = prevFrame_.getPose();
    Sophus::SE3f T2w = currFrame_.getPose();

    Sophus::SE3f T1w_corrected = pMap_->getKeyFrame(0)->getPose();
    Sophus::SE3f T2w_corrected = pMap_->getKeyFrame(1)->getPose();

    // Show position and orientation errors in pose 1
    Eigen::Vector3f position_error1 = T1w_corrected.translation() - T1w.translation();
    Eigen::Vector3f orientation_error1 = (T1w_corrected.so3().inverse() * T1w.so3()).log();

    std::cout << "\nError in position 1:\n";
    std::cout << "x: " << position_error1.x() << " y: " << position_error1.y() << " z: " << position_error1.z() << std::endl;
    std::cout << "Error in orientation 1:\n";
    std::cout << "x: " << orientation_error1.x() << " y: " << orientation_error1.y() << " z: " << orientation_error1.z() << std::endl;

    // Show position and orientation errors in pose 2
    Eigen::Vector3f position_error2 = T2w_corrected.translation() - T2w.translation();
    Eigen::Vector3f orientation_error2 = (T2w_corrected.so3().inverse() * T2w.so3()).log();

    std::cout << "\nError in position 2:\n";
    std::cout << "x: " << position_error2.x() << " y: " << position_error2.y() << " z: " << position_error2.z() << std::endl;
    std::cout << "Error in orientation 2:\n";
    std::cout << "x: " << orientation_error2.x() << " y: " << orientation_error2.y() << " z: " << orientation_error2.z() << std::endl;

    // 3D Error measurement in map points
    std::unordered_map<ID, std::shared_ptr<MapPoint>> mapPoints_corrected = pMap_->getMapPoints();

    float total_error = 0.0f;
    int point_count = 0;
    for (const auto& [id, mapPoint] : mapPoints_corrected) {
        Eigen::Vector3f corrected_position = mapPoint->getWorldPosition();
        Eigen::Vector3f original_position = originalPoints_[id]; 

        Eigen::Vector3f point_error = corrected_position - original_position;
        float error_magnitude = point_error.norm();
        total_error += error_magnitude;
        point_count++;

        std::cout << "\nError for point " << id << ":\n";
        std::cout << "x: " << point_error.x() << " y: " << point_error.y() << " z: " << point_error.z() << std::endl;
    }

    if (point_count > 0) {
        float average_error = total_error / point_count;
        std::cout << "\nTotal error in 3D: " << total_error << std::endl;
        std::cout << "Average error in 3D: " << average_error << std::endl;
    } else {
        std::cout << "No points to compare." << std::endl;
    }
}

Eigen::Matrix3f SLAM::lookAt(const Eigen::Vector3f& camera_pos, const Eigen::Vector3f& target_pos, const Eigen::Vector3f& up_vector) {
    Eigen::Vector3f forward = (target_pos - camera_pos).normalized();
    Eigen::Vector3f right = up_vector.cross(forward).normalized();
    Eigen::Vector3f up = forward.cross(right).normalized();
    
    Eigen::Matrix3f rotation;
    rotation.col(0) = right;
    rotation.col(1) = up;
    rotation.col(2) = forward;
    
    return rotation;
}