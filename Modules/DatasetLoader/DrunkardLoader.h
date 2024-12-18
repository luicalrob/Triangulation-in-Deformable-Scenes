#ifndef SLAM_DRUNKARDLOADER_H
#define SLAM_DRUNKARDLOADER_H

#include <string>
#include <vector>

#include "sophus/se3.hpp"

#include <opencv2/opencv.hpp>
#include "Utils/CommonTypes.h"

class DrunkardLoader {
public:
    /*
     * Loads the dataset stored at folderPath with the given timestamps at timesPath
     */
    DrunkardLoader(std::string folderPath, std::string timesPath);

    /*
     * Retrieves the i pose data. Returns false if the pose does not exit
     */
    bool getPoseData(size_t idx, PoseData& poseData);

    /*
     * Retrieves the i RGB image. Returns false if the image does not exit
     */
    bool getRGBImage(size_t idx, cv::Mat& im);

    /*
     * Retrieves the i RGB image. Returns false if the image does not exit
     */
    bool getDepthImage(size_t idx, cv::Mat& im);

    /*
     * Retrieves the timestamp for the i image pair .Returns false if the timestamp does not exit
     */
    bool getTimeStamp(size_t idx, double& timestamp);

    /*
     * Returns the number of images in the sequence
     */
    int getLenght();

    cv::Size getImageSize();

private:
    std::vector<std::string> vRGBPaths;     //RGB images paths
    std::vector<std::string> vDepthPaths;   //Depth images paths
    std::vector<double> vTimeStamps_;       //Vector with the timestamps
    std::vector<PoseData> vPoseData_;

    cv::Size imSize_;                       //Size of the images
};


#endif //SLAM_DRUNKARDLOADER_H
