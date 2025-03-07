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
 * This class loads and stores the settings for the SLAM system
 */

#ifndef SLAM_SETTINGS_H
#define SLAM_SETTINGS_H

#include "Calibration/CameraModel.h"

#include <memory>

class Settings {
public:
    /*
     * Default constructor: sets everything to default values
     */
    Settings();

    /*
     * Constructor reading parameters from file
     */
    Settings(const std::string& configFile);

    /*
     * Ostream operator overloading to dump settings to the terminal
     */
    friend std::ostream& operator<<(std::ostream& output, const Settings& D);

    //Getter methods
    std::shared_ptr<CameraModel> getCalibration();
    std::shared_ptr<CameraModel> getPHCalibration();
    std::vector<float> getDistortionParameters();
    int getImCols();
    int getImRows();

    std::string getBorderMask();
    int getFeaturesPerImage();
    int getNumberOfScales();
    float getScaleFactor();

    int getGridCols();
    int getGridRows();

    float getEpipolarTh();

    int getMatchingInitTh();
    int getMatchingGuidedTh();
    int getMatchingByProjectionTh();
    int getMatchingForTriangulationTh();
    int getMatchingFuseTh();

    float getMatchingInitRadius();

    int getMinCommonObs();

    float getMinMatches();
    float getMinCos();
    bool getCheckingSelection();
    float getDepthLimit();
    
    Eigen::Vector3f getFirstCameraPos();
    Eigen::Vector3f getSecondCameraPos();

    float getSimulatedRepError();
    int getDecimalsRepError();
    float getSimulatedDepthError();
    float getSimulatedDepthWeight();
    float getSimulatedDepthScaleC1();
    float getSimulatedDepthScaleC2();
    double getDepthMeasurementsScale();

    double getOptRepWeight();
    double getOptArapWeight();
    double getOptGlobalWeight();
    double getOptAlphaWeight();
    double getOptBetaWeight();

    std::string getOptSelection();
    std::string getOptWeightsSelection();
    std::string getTrianMethod();
    std::string getTrianLocation();

    int getnOptimizations();
    int getnOptIterations();

    int getNloptnOptimizations();
    double getNloptRelTolerance();
    double getNloptAbsTolerance();
    double getNloptRepLowerBound();
    double getNloptRepUpperBound();
    double getNloptGlobalLowerBound();
    double getNloptGlobalUpperBound();
    double getNloptArapLowerBound();
    double getNloptArapUpperBound();

    std::string getExpFilePath();

    bool getShowScene();
    bool getDrawRaysSelection();
    bool getShowSolution();
    bool getStopExecutionOption();

private:
    //Camera parameters
    std::shared_ptr<CameraModel> calibration_;  //Geometric calibration with projection and unprojection functions
    std::shared_ptr<CameraModel> pinHolecalibration_; 
    std::vector<float> vDistortion_;                //[Optional] Image distortion parameters
    int imCols_, imRows_;                           //Image size

    //Feature extractor parameters
    std::string borderMask_;
    int nFeatures_;                                 //Number of features to extract per image
    int nScales_;                                   //Number of scales in the image pyramid representation
    float fScaleFactor_;                            //Scale factor

    //Feature grid
    int nGridCols_, nGridRows_;                     //Feature grid size

    float fEpipolarTh_;                             //Epipolar threshold for the Essential matrix

    int nMatchingInitTh_;
    int nMatchingGuidedTh_;
    int nMatchingProjectionTh_;
    int nMatchingTriangulationTh_;
    int nMatchingFuseTh_;

    float nMatchingInitRadius_;

    int nMinCommonObs_;

    float fMinMatches_;
    float fMinCos_;
    bool checkingSelection_;
    float fDepthLimit_;

    Eigen::Vector3f C1Pose_;
    Eigen::Vector3f C2Pose_;

    float SimulatedRepError_;
    int DecimalsRepError_;
    float SimulatedDepthError_;
    float SimulatedDepthWeight_;
    float SimulatedDepthScaleC1_;
    float SimulatedDepthScaleC2_;
    double DepthMeasurementsScale_;

    double OptRepWeight_;
    double OptArapWeight_;
    double OptGlobalWeight_;
    double OptAlphaWeight_;
    double OptBetaWeight_;

    std::string OptSelection_;
    std::string OptWeightsSelection_;
    std::string TrianMethod_;
    std::string TrianLocation_;

    int NloptnOptimizations_;
    double NloptRelTolerance_;
    double NloptAbsTolerance_;
    double NloptRepLowerBound_;
    double NloptRepUpperBound_;
    double NloptGlobalLowerBound_;
    double NloptGlobalUpperBound_;
    double NloptArapLowerBound_;
    double NloptArapUpperBound_;

    std::string ExpFilePath_;

    int nOptimizations_;
    int nOptIterations_;

    bool showScene_;
    bool drawRaysSelection_;
    bool showSolution_;
    bool stop_;
};


#endif //SLAM_SETTINGS_H
