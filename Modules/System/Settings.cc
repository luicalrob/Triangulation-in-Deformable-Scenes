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

#include "Calibration/PinHole.h"

#include "Settings.h"

using namespace std;

Settings::Settings(){}

Settings::Settings(const std::string& configFile) {
    //Open settings file
    cv::FileStorage fSettings(configFile, cv::FileStorage::READ);
    if(!fSettings.isOpened()){
        cerr << "[ERROR]: could not open configuration file at: " << configFile << endl;
        cerr << "Aborting..." << endl;

        exit(-1);
    }

    //Read camera calibration
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];
    vector<float> vCalibration = {fx,fy,cx,cy};

    calibration_ = shared_ptr<CameraModel>(new PinHole(vCalibration));

    //Read (if exists) distortion parameters
    if(!fSettings["Camera.k1"].empty()){
        if(!fSettings["Camera.k3"].empty()){
            vDistortion_.resize(5);
            vDistortion_[4] = fSettings["Camera.k3"];
        }
        else{
            vDistortion_.resize(4);
        }

        vDistortion_[0] = fSettings["Camera.k1"];
        vDistortion_[1] = fSettings["Camera.k2"];
        vDistortion_[2] = fSettings["Camera.p1"];
        vDistortion_[3] = fSettings["Camera.p2"];
    }

    //Read image dimensions
    imCols_ = fSettings["Camera.cols"];
    imRows_ = fSettings["Camera.rows"];

    //Read Feature extractor parameters
    nFeatures_ = fSettings["FeatureExtractor.nFeatures"];
    nScales_ = fSettings["FeatureExtractor.nScales"];
    fScaleFactor_ = fSettings["FeatureExtractor.fScaleFactor"];

    //Read feature grid parameters
    nGridCols_ = fSettings["FeatureGrid.nGridCols"];
    nGridRows_ = fSettings["FeatureGrid.nGridRows"];

    //Read Epipolar threshold
    fEpipolarTh_ = fSettings["Epipolar.th"];

    //Read matching thresholds
    nMatchingInitTh_ = fSettings["Matching.initialization"];
    nMatchingGuidedTh_ = fSettings["Matching.guidedMatching"];
    nMatchingProjectionTh_ = fSettings["Matching.searchByProjection"];
    nMatchingTriangulationTh_ = fSettings["Matching.searchForTriangulation"];
    nMatchingFuseTh_ = fSettings["Matching.fuse"];

    nMatchingInitRadius_ = fSettings["Matching.initialization.radius"];

    nMinCommonObs_ = fSettings["Map.minObs"];
    fMinCos_ = fSettings["Triangulation.minCos"];

    //LUIS
    float C1x = fSettings["Camera.FirstPose.x"];
    float C1y = fSettings["Camera.FirstPose.y"];
    float C1z = fSettings["Camera.FirstPose.z"];
    C1Pose_ << C1x, C1y, C1z;
    float C2x = fSettings["Camera.SecondPose.x"];
    float C2y = fSettings["Camera.SecondPose.y"];
    float C2z = fSettings["Camera.SecondPose.z"];
    C2Pose_ << C2x, C2y, C2z;

    SimulatedRepError_ = fSettings["Keypoints.RepError"];
    DecimalsRepError_ = fSettings["Keypoints.decimalsApproximation"];
    SimulatedDepthError_ = fSettings["Measurements.DepthError"];
    SimulatedDepthScaleC1_ = fSettings["Measurements.DepthScale.C1"];
    SimulatedDepthScaleC2_ = fSettings["Measurements.DepthScale.C2"];
    DepthMeasurementsScale_ = fSettings["Measurements.Depth.Scale"];

    OptRepWeight_ = fSettings["Optimization.rep"];
    OptArapWeight_ = fSettings["Optimization.arap"];
    OptGlobalWeight_ = fSettings["Optimization.global"];
    OptAlphaWeight_ = fSettings["Optimization.alpha"];
    OptBetaWeight_ = fSettings["Optimization.beta"];

    fSettings["Optimization.selection"] >> OptSelection_;
    fSettings["Optimization.weightsSelection"] >> OptWeightsSelection_;
    fSettings["Triangulation.method"] >> TrianMethod_;
    fSettings["Triangulation.seed.location"] >> TrianLocation_;

    nOptimizations_ = fSettings["Optimization.numberOfOptimizations"];
    nOptIterations_ = fSettings["Optimization.numberOfIterations"];

    NloptnOptimizations_ = fSettings["Optimization.nlopt.numberOfIterations"];
    NloptRelTolerance_ = fSettings["Optimization.nlopt.relTolerance"];
    NloptAbsTolerance_ = fSettings["Optimization.nlopt.absTolerance"];
    NloptRepLowerBound_ = fSettings["Optimization.nlopt.rep.lowerBound"];
    NloptRepUpperBound_ = fSettings["Optimization.nlopt.rep.upperBound"];
    NloptGlobalLowerBound_ = fSettings["Optimization.nlopt.global.lowerBound"];
    NloptGlobalUpperBound_ = fSettings["Optimization.nlopt.global.upperBound"];
    NloptArapLowerBound_ = fSettings["Optimization.nlopt.arap.lowerBound"];
    NloptArapUpperBound_ = fSettings["Optimization.nlopt.arap.upperBound"];

    fSettings["Experiment.Filepath"] >> ExpFilePath_;
    
    std::string showSceneString;
    fSettings["MapVisualizer.showScene"] >> showSceneString;

    if (showSceneString == "true") {
        showScene_ = true;
    } else {
        showScene_ = false;
    }
    
    std::string drawRaysString;
    fSettings["MapVisualizer.drawRays"] >> drawRaysString;

    if (drawRaysString == "true") {
        drawRaysSelection_ = true;
    } else {
        drawRaysSelection_ = false;
    }

    std::string showSolutionString;
    fSettings["Visualizer.showSolution"] >> showSolutionString;

    if (showSolutionString == "true") {
        showSolution_ = true;
    } else {
        showSolution_ = false;
    }

    std::string stopString;
    fSettings["Execution.stop"] >> stopString;

    if (stopString == "true") {
        stop_ = true;
    } else {
        stop_ = false;
    }
}

ostream &operator<<(std::ostream& output, const Settings& settings){
    output << "SLAM settings: " << endl;

    output << "\t-Camera parameters: [ ";
    output << settings.calibration_->getParameter(0) << " , " << settings.calibration_->getParameter(1) << " , ";
    output << settings.calibration_->getParameter(2) << " , " << settings.calibration_->getParameter(3) << " ]" << endl;

    if(!settings.vDistortion_.empty()){
        output << "\t-Distortion parameters: [ ";
        output << settings.vDistortion_[0] << " , " << settings.vDistortion_[1] << " , ";
        output << settings.vDistortion_[2] << " , " << settings.vDistortion_[3];

        if(settings.vDistortion_.size() == 5)
            cout << " , " << settings.vDistortion_[4];

        cout << " ]" << endl;
    }

    output << "\t-Image dimensions: [ ";
    output << settings.imCols_ << " , " << settings.imRows_ << " ]" << endl;

    output << "\t-Features to extract per image: " << settings.nFeatures_ << endl;

    output << "\t-Number of image scales: " << settings.nScales_ << endl;

    output << "\t-Scale factor: " << settings.fScaleFactor_ << endl;

    output << "\t-Feature grid dimensions: [ " << settings.nGridCols_ << " , " << settings.nGridRows_ << " ]" << endl;

    return output;
}

std::shared_ptr<CameraModel> Settings::getCalibration() {
    return calibration_;
}

std::vector<float> Settings::getDistortionParameters() {
    return vDistortion_;
}

int Settings::getImCols() {
    return imCols_;
}

int Settings::getImRows() {
    return imRows_;
}

int Settings::getFeaturesPerImage() {
    return nFeatures_;
}

int Settings::getNumberOfScales() {
    return nScales_;
}

float Settings::getScaleFactor() {
    return fScaleFactor_;
}

int Settings::getGridCols() {
    return nGridCols_;
}

int Settings::getGridRows() {
    return nGridRows_;
}

float Settings::getEpipolarTh(){
    return fEpipolarTh_;
}

int Settings::getMatchingInitTh(){
    return nMatchingInitTh_;
}

int Settings::getMatchingGuidedTh(){
    return nMatchingGuidedTh_;
}

int Settings::getMatchingByProjectionTh(){
    return nMatchingProjectionTh_;
}

int Settings::getMatchingForTriangulationTh(){
    return nMatchingTriangulationTh_;
}

int Settings::getMatchingFuseTh(){
    return nMatchingFuseTh_;
}

float Settings::getMatchingInitRadius(){
    return nMatchingInitRadius_;
}

int Settings::getMinCommonObs(){
    return nMinCommonObs_;
}

float Settings::getMinCos(){
    return fMinCos_;
}

Eigen::Vector3f Settings::getFirstCameraPos(){
    return C1Pose_;
}

Eigen::Vector3f Settings::getSecondCameraPos(){
    return C2Pose_;
}

float Settings::getSimulatedRepError(){
    return SimulatedRepError_;
}

int Settings::getDecimalsRepError(){
    return DecimalsRepError_;
}

float Settings::getSimulatedDepthError(){
    return SimulatedDepthError_;
}

float Settings::getSimulatedDepthScaleC1(){
    return SimulatedDepthScaleC1_;
}

float Settings::getSimulatedDepthScaleC2(){
    return SimulatedDepthScaleC2_;
}

double Settings::getDepthMeasurementsScale(){
    return DepthMeasurementsScale_;
}

double Settings::getOptRepWeight(){
    return OptRepWeight_;
}

double Settings::getOptArapWeight(){
    return OptArapWeight_;
}

double Settings::getOptGlobalWeight(){
    return OptGlobalWeight_;
}

double Settings::getOptAlphaWeight(){
    return OptAlphaWeight_;
}

double Settings::getOptBetaWeight(){
    return OptBetaWeight_;
}

std::string Settings::getOptSelection(){
    return OptSelection_;
}

std::string Settings::getOptWeightsSelection(){
    return OptWeightsSelection_;
}

std::string Settings::getTrianMethod(){
    return TrianMethod_;
}

std::string Settings::getTrianLocation(){
    return TrianLocation_;
}

int Settings::getnOptimizations(){
    return nOptimizations_;
}

int Settings::getnOptIterations(){
    return nOptIterations_;
}

int Settings::getNloptnOptimizations(){
    return NloptnOptimizations_;
}

double Settings::getNloptRelTolerance(){
    return NloptRelTolerance_;
}

double Settings::getNloptAbsTolerance(){
    return NloptAbsTolerance_;
}

double Settings::getNloptRepLowerBound(){
    return NloptRepLowerBound_;
}

double Settings::getNloptRepUpperBound(){
    return NloptRepUpperBound_;
}

double Settings::getNloptGlobalLowerBound(){
    return NloptGlobalLowerBound_;
}

double Settings::getNloptGlobalUpperBound(){
    return NloptGlobalUpperBound_;
}

double Settings::getNloptArapLowerBound(){
    return NloptArapLowerBound_;
}

double Settings::getNloptArapUpperBound(){
    return NloptArapUpperBound_;
}

std::string Settings::getExpFilePath() {
    return ExpFilePath_;
}

bool Settings::getDrawRaysSelection(){
    return drawRaysSelection_;
}

bool Settings::getShowSolution(){
    return showSolution_;
}

bool Settings::getShowScene(){
    return showScene_;
}

bool Settings::getStopExecutionOption(){
    return stop_;
}