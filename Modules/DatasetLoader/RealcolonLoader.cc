#include "RealcolonLoader.h"

#include <iostream>
#include <fstream>
#include <iomanip>
#include <filesystem>
#include <algorithm>

#include <sys/stat.h>

using namespace std;
namespace fs = std::filesystem;

RealcolonLoader::RealcolonLoader(std::string folderPath, std::string timesPath) {
    vRGBPaths.reserve(5000);
    vDepthPaths.reserve(5000);

    std::vector<std::string> rgbFiles;
    for (const auto &entry : fs::directory_iterator(folderPath + "/rgb")) {
        if (entry.is_regular_file()) {
            rgbFiles.push_back(entry.path().string());
        }
    }

    std::vector<std::string> depthFiles;
    for (const auto &entry : fs::directory_iterator(folderPath + "/depth")) {
        if (entry.is_regular_file()) {
            depthFiles.push_back(entry.path().string());
        }
    }

    std::sort(rgbFiles.begin(), rgbFiles.end());
    std::sort(depthFiles.begin(), depthFiles.end());

    vRGBPaths = std::move(rgbFiles);
    vDepthPaths = std::move(depthFiles);

    if (!vRGBPaths.empty()) {
        cv::Mat im;
        getRGBImage(0,im);
        imSize_ = im.size();
    }
}

bool RealcolonLoader::getPoseData(size_t idx, PoseData& poseData) {
    if(idx >= vTimeStamps_.size()) return false;

    poseData = vPoseData_[idx];

    return true;
}

bool RealcolonLoader::getRGBImage(size_t idx, cv::Mat& im) {
    if(idx >= vTimeStamps_.size()) return false;

    //cout << "[RealcolonLoader]: loading image at " << vImgsPairs_[idx].first << endl;
    im = cv::imread(vRGBPaths[idx], cv::IMREAD_UNCHANGED);

    return true;
}

bool RealcolonLoader::getDepthImage(size_t idx, cv::Mat& im) {
    if(idx >= vTimeStamps_.size()) return false;

    //cout << "[RealcolonLoader]: loading image at " << vImgsPairs_[idx].first << endl;
    im = cv::imread(vDepthPaths[idx], cv::IMREAD_UNCHANGED);

    return true;
}

bool RealcolonLoader::getTimeStamp(size_t idx, double &timestamp) {
    if(idx >= vTimeStamps_.size()) return false;

    timestamp = vTimeStamps_[idx];

    return true;
}

int RealcolonLoader::getLength() {
    return (int)vTimeStamps_.size();
}

cv::Size RealcolonLoader::getImageSize() {
    return imSize_;
}