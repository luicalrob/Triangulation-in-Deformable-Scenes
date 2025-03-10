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
 * This class represents a Frame, the basic data structure used in the mapping
 * to store the visual information of an image
 */

#ifndef SLAM_FRAME_H
#define SLAM_FRAME_H

#include "Calibration/CameraModel.h"
#include "Map/MapPoint.h"

#include <unordered_set>

#include <sophus/se3.hpp>
#include <opencv2/opencv.hpp>

typedef std::vector<std::vector<std::vector<size_t>>> Grid;

class Frame {
public:
    Frame();

    /*
     * Constructor with the setting of the Frame like the number of features, the size of the grid,
     * calibration, scales, etc
     */
    Frame(const int nFeatures, const int nGridCols, const int nGridRows,
          const int nImCols, const int nImRows, int nScales, float fScaleFactor,
          const std::shared_ptr<CameraModel> calibration,
          const std::shared_ptr<CameraModel> phcalibration,
          const std::vector<float>& vDistortion = {},
          const double dScale = 0.0,
          const float depthError = 0.0);

    /*
     * Sets the pose of the Frame
     */
    void setPose(Sophus::SE3f& Tcw);

    /*
     * Gets all the KeyPoints of the Frame
     */
    std::vector<cv::KeyPoint>& getKeyPoints();

    /*
     * Gets the distorted KeyPoints of the Frame. Mainly used for visualization
     */
    std::vector<cv::KeyPoint>& getKeyPointsDistorted();

    /*
     * Gets the undistorted KeyPoint ad index idx
     */
    cv::KeyPoint getKeyPoint(const size_t idx);

    /*
     * Sets a new KeyPoint in the Frame at index idx
     */
    void setKeyPoint(cv::KeyPoint pKP, const size_t idx);

    /*
     * Gets the depth measure at index idx in the KeyFrame
     */
    float getDepthMeasure(const size_t idx);

    /*
     * Gets the depth measure of the depth image
     */
    double getDepthMeasure(float x, float y);

    /*
     * Gets all the depth measurements of the Frame
     */
    std::vector<float>& getDepthMeasurements();

    /*
     * Sets a new depth measure in the Frame at index idx
     */
    void setDepthMeasure(float depth, const size_t idx);

    /*
     * Gets all the MapPoints matched in the Frame
     */
    std::vector<std::shared_ptr<MapPoint>>& getMapPoints();

    /*
     * Gets the MapPoint of the Frame at index idx
     */
    std::shared_ptr<MapPoint> getMapPoint(const size_t idx);

    /*
     * Removes all MapPoints from the Frame
     */
    void clearMapPoints();

    /*
     * Returns the feature grid of the Frame
     */
    Grid getGrid();

    /*
     * Sets a new MapPoint in the Frame at index idx
     */
    void setMapPoint(size_t idx, std::shared_ptr<MapPoint> pMP);

    /*
     * Gets the descriptor matrix of the features of the Frame
     */
    cv::Mat& getDescriptors();

    /*
     * Gets the pose of the Frame
     */
    const Sophus::SE3f getPose() const;

    /*
     * Gets the calibration of the Frame
     */
    std::shared_ptr<CameraModel> getCalibration();

    std::shared_ptr<CameraModel> getPHCalibration();

    /*
     * Swaps the contents of 2 frames
     */
    void assign(Frame& F);

    /*
     * Undistorts and distributes in the feature grid the extracted features
     */
    void distributeFeatures();

    /*
     * Retrieves a list of features close to a given (x,y) position that fulfill the following conditions:
     *  -Their distance to (x,y) is less than radius
     *  -They have been observed in an scale grater or equal than minLevel and less or equal than maxLevel
     */
    void getFeaturesInArea(const float x, const float y, const float radius,
                           const int minLevel, const int maxLevel, std::vector<size_t>& vIndices);

    /*
     * Gets the number of scales used in the image pyramid
     */
    int getNumberOfScales();

    /*
     * Gets the corresponding scale factor for a given level of the image pyramid
     */
    float getScaleFactor(int octave);

    /*
     * Gets the corresponding inverse scale factor for a given level of the image pyramid
     */
    float getInvScaleFactor(int octave);

    /*
     * Gets the corresponding KeyPoint observation uncertainty for an observation made in a given
     * level of the image pyramid
     */
    float getSigma2(int octave);

    /*
     * Gets the corresponding inverse KeyPoint observation uncertainty for an observation made in a given
     * level of the image pyramid
     */
    float getInvSigma2(int octave);

    /*
     * Gets the number of columns of the feature grid
     */
    float getGridElementWidhtInv();

    /*
     * Gets the number ofrows of the feature grid
     */
    float getGridElementHeighInv();

    /*
     * Gets the min col where an undistorted feature can be seen
     */
    float getMinCol();

    /*
     * Gets the min row where an undistorted feature can be seen
     */
    float getMinRow();

    /*
     * Sets the image for the Frame
     */
    void setIm(cv::Mat& im);

    void setRgbIm(cv::Mat& im);

    /*
     * Sets the depth image for the Frame
     */
    void setDepthIm(cv::Mat& im);

    /*
     * Gets the image of the frame
     */
    cv::Mat getIm();

    cv::Mat getRgbIm();

    /*
     * Gets the depth image of the frame
     */
    cv::Mat getDepthIm();

    /*
     * Gets the depth scale of the depth image of the frame
     */
    double getDepthScale();

    /*
     * Gets Frame estimated depth scale (up to scale depth measurements) (real images)
     */
    double getEstimatedDepthScale();

    /*
     * Set KF estimated depth scale (up to scale depth measurements) (real images)
     */
    void setEstimatedDepthScale(double scale);

    /*
     * Gets the depth error introduced to the real depth measurements
     */
    float getDepthError();

    /*
     * Checks that all MapPoints matched are good i.e. their error is low. ONLY USED FOR DEBUG PURPOSES
     */
    void checkAllMapPointsAreGood();

    /*
     * Sets the timestamp fo the frame
     */
    void setTimestamp(double ts);

    /*
     * Gets the timestamp fo the frame
     */
    double getTimestamp();

private:
    //Initialization methods (to be only called at the constructor)
    void initializeGrid(const int nGridCols, const int nGridRows, const int nFeatures);
    void computeImageBoundaries(const std::vector<float>& vDistortion, const int nImCols, const int nImRows);
    void undistortKeys();

    //Undistorted and distorted KeyPoints, its decsriptor and matched MapPoints, related by the same index
    std::vector<cv::KeyPoint> vKeys_, vKeysDis_;
    std::vector<float> vDepthMeasurements_;
    cv::Mat descriptors_;
    std::vector<std::shared_ptr<MapPoint>> vMapPoints_;

    //Pose of the frame
    Sophus::SE3f Tcw_;

    //------------------------
    //     Calibration
    //------------------------
    std::shared_ptr<CameraModel> calibration_;  //Camera calibration
    std::shared_ptr<CameraModel> phcalibration_;
    std::vector<float> vDistortion_;                //Distortion parameters (optional)
    float minCol_, maxCol_;                         //Undistorted image boundaries (cols)
    float minRow_, maxRow_;                         //Undistorted image boundaries (rows)

    //------------------------
    // Point grid for faster feature matching
    //------------------------
    Grid grid_;
    float gridElementWidthInv_, gridElementHeightInv_;

    //For undistortion
    cv::Mat undistMat_;

    bool posInGrid(const float x, const float y, int &col, int &row);

    int nScales_;
    std::vector<float> vScaleFactor_, vInvScaleFactor_; //Scale and inverse scale factor for each pyramid level
    std::vector<float> vSigma2_, vInvSigma2_;           //Uncertainties for a KeyPoint extracted at a image scale

    cv::Mat im_;
    cv::Mat rgbIm_;
    cv::Mat depthIm_;
    double timestamp_;
    double imageDepthScale_ = 1.0; // for simulatng an unknown scale
    double estimatedDepthScale_ = 1.0; // scale estimated
    float depthError_;
};


#endif //SLAM_FRAME_H
