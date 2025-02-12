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
#include "Utils/Conversions.h"
#include "Utils/Measurements.h"
#include "Utils/Utils.h"
#include "Optimization/nloptOptimization.h"
#include "Optimization/EigenOptimization.h"

#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>

#include <nlopt.hpp>
#include <memory>
#include <iomanip>
#include <locale>

SLAM::SLAM(const std::string& settingsFile, const PoseData& pose) {
    //Load settings from file
    cout << "Loading system settings from: " << settingsFile << endl;
    settings_ = Settings(settingsFile);
    cout << settings_ << endl;

    //Create map
    pMap_ = shared_ptr<Map>(new Map(settings_.getMinCommonObs()));

    //Create visualizers
    bool showScene = settings_.getShowScene();
    mapVisualizer_ = shared_ptr<MapVisualizer>(new MapVisualizer(pMap_, pose, showScene));
    visualizer_ = shared_ptr<FrameVisualizer>(new FrameVisualizer(showScene));

    //Initialize mapper
    mapper_ = Mapping(settings_, visualizer_, mapVisualizer_, pMap_);

    refFrame_ = Frame(settings_.getFeaturesPerImage(),settings_.getGridCols(),settings_.getGridRows(),
                       settings_.getImCols(),settings_.getImRows(),settings_.getNumberOfScales(), settings_.getScaleFactor(),
                       settings_.getCalibration(),settings_.getDistortionParameters(), settings_.getSimulatedDepthError());

    currFrame_ = Frame(settings_.getFeaturesPerImage(),settings_.getGridCols(),settings_.getGridRows(),
                       settings_.getImCols(),settings_.getImRows(), settings_.getNumberOfScales(), settings_.getScaleFactor(),
                       settings_.getCalibration(),settings_.getDistortionParameters(), settings_.getSimulatedDepthError());

    prevCalibration_ = refFrame_.getCalibration();
    currCalibration_ = currFrame_.getCalibration();

    currIm_ = cv::Mat::zeros(settings_.getImRows(), settings_.getImCols(), CV_8UC3);
    currIm_.setTo(cv::Scalar(255, 255, 255));

    C1Pose_ = settings_.getFirstCameraPos();
    C2Pose_ = settings_.getSecondCameraPos();

    simulatedRepErrorStanDesv_ = settings_.getSimulatedRepError();
    decimalsRepError_ = settings_.getDecimalsRepError();
    SimulatedDepthErrorStanDesv_ = settings_.getSimulatedDepthError();
    SimulatedDepthScaleC1_ = settings_.getSimulatedDepthScaleC1();
    SimulatedDepthScaleC2_ = settings_.getSimulatedDepthScaleC2();

    repBalanceWeight_ = settings_.getOptRepWeight();
    arapBalanceWeight_ = settings_.getOptArapWeight();
    globalBalanceWeight_ = settings_.getOptGlobalWeight();
    alphaWeight_ = settings_.getOptAlphaWeight();
    betaWeight_ = settings_.getOptBetaWeight();

    OptSelection_ = settings_.getOptSelection();
    OptWeightsSelection_ = settings_.getOptWeightsSelection();

    nOptimizations_ = settings_.getnOptimizations();
    nOptIterations_ = settings_.getnOptIterations();

    NloptnOptimizations_ = settings_.getNloptnOptimizations();
    NloptRelTolerance_ = settings_.getNloptRelTolerance();
    NloptAbsTolerance_ = settings_.getNloptAbsTolerance();
    NloptRepLowerBound_ = settings_.getNloptRepLowerBound();
    NloptRepUpperBound_ = settings_.getNloptRepUpperBound();
    NloptGlobalLowerBound_ = settings_.getNloptGlobalLowerBound();
    NloptGlobalUpperBound_ = settings_.getNloptGlobalUpperBound();
    NloptArapLowerBound_ = settings_.getNloptArapLowerBound();
    NloptArapUpperBound_ = settings_.getNloptArapUpperBound();

    drawRaysSelection_ = settings_.getDrawRaysSelection();
    showSolution_ = settings_.getShowSolution();
    stop_ = settings_.getStopExecutionOption();

    filePath_ = settings_.getExpFilePath();
    outFile_.imbue(std::locale("es_ES.UTF-8"));
}

bool SLAM::processImage(const cv::Mat &im, const cv::Mat &depthIm, Sophus::SE3f& Tcw, int &nKF, int &nMPs, clock_t &timer) {
    if(stop_) {
        cv::namedWindow("Test Window");
    }

    //Tcw_ = Twc.inverse();
    
    cv::Mat grayIm = convertImageToGrayScale(im);

    std::cerr << "Let's do Mapping! " << std::endl;

    // Do mapping
    bool goodMapped = mapper_.doMapping(grayIm, depthIm, Tcw, nKF, nMPs, timer);
    
    if (goodMapped) {
        startMeasurementsOnFile();
        stop();

        //Run deformation optimization
        deformationOptimization(pMap_, settings_, mapVisualizer_);
    }

    return goodMapped;
}

bool SLAM::processSimulatedImage(int &nMPs, clock_t &timer) {
    if(stop_) {
        cv::namedWindow("Test Window");
    }

    // Do mapping
    mapper_.doSimulatedMapping(currKeyFrame_, refKeyFrame_, nMPs);

    startMeasurementsOnFile();
    stop();

    //Run deformation optimization
    deformationOptimization(pMap_, settings_, mapVisualizer_, originalPoints_ , movedPoints_);

    return true;
}

cv::Mat SLAM::convertImageToGrayScale(const cv::Mat &im) {
    cv::Mat grayScaled;

    if(im.type() == CV_8U)
        grayScaled = im;
    else if(im.channels()==3){
        cvtColor(im,grayScaled,cv::COLOR_RGB2GRAY);
    }
    else if(im.channels()==4){
        cvtColor(im,grayScaled,cv::COLOR_BGRA2GRAY);
    }

    return grayScaled;
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

    originalPoints_.clear();
    movedPoints_.clear();

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

    mapper_.originalPoints_ = originalPoints_;
    mapper_.movedPoints_ = movedPoints_;
    std::cout << "Loaded " << originalPoints_.size() << " MapPoints from files." << std::endl;
}

void SLAM::setCameraPoses(const Eigen::Vector3f firstCamera, const Eigen::Vector3f secondCamera) {
    // set camera poses and orientation
    Eigen::Matrix3f R = Eigen::Matrix3f::Identity();
    Sophus::SE3f T1w(R, firstCamera);
    refFrame_.setPose(T1w);

    R = lookAt(secondCamera, movedPoints_[0]);
    Sophus::SE3f T2w(R, secondCamera);
    currFrame_.setPose(T2w);
    Tcw_ = T2w;

    mapVisualizer_->updateCurrentPose(T2w);
}

void SLAM::viusualizeSolution() {
    size_t nTriangulated = 0;
    size_t nMatches = movedPoints_.size();

    std::cout << "Number of matches: " << nMatches << std::endl;

    Sophus::SE3f T1w = refFrame_.getPose();
    Sophus::SE3f T2w = currFrame_.getPose();

    for(size_t i = 0; i < nMatches; i++){
        Eigen::Vector3f x3D_1 = originalPoints_[i];
        Eigen::Vector3f x3D_2 = movedPoints_[i];

        std::shared_ptr<MapPoint> map_point_1(new MapPoint(x3D_1));
        std::shared_ptr<MapPoint> map_point_2(new MapPoint(x3D_2));

        pMap_->insertMapPoint(map_point_1);
        pMap_->insertMapPoint(map_point_2);

        insertedIndexes_.push_back(i);

        pMap_->addObservation(refKeyFrame_->getId(), map_point_1->getId(), i);  // vMatches[i] si las parejas no fuesen ordenadas
        pMap_->addObservation(currKeyFrame_->getId(), map_point_2->getId(), i);

        refKeyFrame_->setMapPoint(i, map_point_1);
        currKeyFrame_->setMapPoint(i, map_point_2);

        nTriangulated++;
        nTriangulated++;
    }

    std::cout << "Triangulated " << nTriangulated << " MapPoints." << std::endl;

    // stop
    //Uncomment for step by step execution (pressing esc key)
    if(stop_) {
        cv::namedWindow("Test Window");
    }

    // visualize
    mapVisualizer_->update(drawRaysSelection_);
    mapVisualizer_->updateCurrentPose(Tcw_);
}

void SLAM::createKeyPoints() {
    std::default_random_engine generator;
    std::normal_distribution<float> distribution(0.0f, simulatedRepErrorStanDesv_);

    Sophus::SE3f T1w = refFrame_.getPose();
    Sophus::SE3f T2w = currFrame_.getPose();

    for(size_t i = 0; i < movedPoints_.size(); ++i) {
        cv::Point2f original_p2D;
        cv::Point2f moved_p2D;

        Eigen::Vector3f p3Dcam1 = T1w * originalPoints_[i];
        Eigen::Vector3f p3Dcam2 = T2w * movedPoints_[i];

        original_p2D = prevCalibration_->project(p3Dcam1);
        moved_p2D = currCalibration_->project(p3Dcam2);

        // Add Gaussian reprojection noise in units of pixels
        original_p2D.x = roundToDecimals(original_p2D.x + distribution(generator), decimalsRepError_);
        original_p2D.y = roundToDecimals(original_p2D.y + distribution(generator), decimalsRepError_);
        moved_p2D.x = roundToDecimals(moved_p2D.x + distribution(generator), decimalsRepError_);
        moved_p2D.y = roundToDecimals(moved_p2D.y + distribution(generator), decimalsRepError_);

        cv::KeyPoint original_keypoint(original_p2D, 1.0f); // 1.0f is the size of the keypoint
        cv::KeyPoint moved_keypoint(moved_p2D, 1.0f);

        refFrame_.setKeyPoint(original_keypoint, i);
        currFrame_.setKeyPoint(moved_keypoint, i);
    }

    // Promote to keyframes for visualization
    refKeyFrame_ = std::make_shared<KeyFrame>(refFrame_);
    currKeyFrame_ = std::make_shared<KeyFrame>(currFrame_);
    pMap_->insertKeyFrame(refKeyFrame_);
    pMap_->insertKeyFrame(currKeyFrame_);

    visualizer_->drawFeatures(refFrame_.getKeyPoints(), currIm_, "Previous Frame KeyPoints");
    visualizer_->drawFeatures(currFrame_.getKeyPoints(), currIm_, "Current Frame KeyPoints");
}

void SLAM::getSimulatedDepthMeasurements() {
    std::default_random_engine generator;
    std::normal_distribution<float> distribution(0.0f, SimulatedDepthErrorStanDesv_/1000);

    Sophus::SE3f T1w = refFrame_.getPose();
    Sophus::SE3f T2w = currFrame_.getPose();

    for(size_t i = 0; i < movedPoints_.size(); ++i) {
        Eigen::Vector3f p3Dcam1 = T1w * originalPoints_[i];
        Eigen::Vector3f p3Dcam2 = T2w * movedPoints_[i];

        float d1 = p3Dcam1[2] * SimulatedDepthScaleC1_ + distribution(generator);
        float d2 = p3Dcam2[2] * SimulatedDepthScaleC2_ + distribution(generator);

        refFrame_.setDepthMeasure(d1, i);
        currFrame_.setDepthMeasure(d2, i);
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

Eigen::Vector3f SLAM::getFirstCameraPos(){
    return C1Pose_;
}

Eigen::Vector3f SLAM::getSecondCameraPos(){
    return C2Pose_;
}

void SLAM::stop(){
    stopWithMeasurements(pMap_, Tcw_, mapVisualizer_, filePath_ , drawRaysSelection_, 
                            stop_, originalPoints_, movedPoints_);
}

void SLAM::startMeasurementsOnFile(){
    std::cout << "\nINITIAL MEASUREMENTS: \n";
    outFile_.open(filePath_, std::ios::app);
    if (outFile_.is_open()) {
        outFile_ << "INITIAL MEASUREMENTS: \n";

        outFile_.close();
    } else {
        std::cerr << "Unable to open file for writing" << std::endl;
    }
}
