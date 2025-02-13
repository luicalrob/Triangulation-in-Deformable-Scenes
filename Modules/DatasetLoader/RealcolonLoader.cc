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

    std::ifstream rgbFile(timesPath);
    if (!rgbFile.is_open()) {
        std::cerr << "[RealcolonLoader]: Could not open " << timesPath << std::endl;
        return;
    }

    std::string line;
    while (std::getline(rgbFile, line)) {
        if (!line.empty()) {
            std::stringstream ss(line);
            std::string filename;
            ss >> filename;
            vRGBPaths.push_back(folderPath + "/rgb/" + filename);
            vDepthPaths.push_back(folderPath + "/depth/" + filename);
        }
    }

    if (!vRGBPaths.empty()) {
        cv::Mat im;
        getRGBImage(0, im);
        imSize_ = im.size();
        std::cout << "imSize_: " << imSize_ << std::endl;
    }
}

bool RealcolonLoader::getPoseData(size_t idx, PoseData& poseData) {
    if(idx >= vRGBPaths.size()) return false;

    poseData = vPoseData_[idx];

    return true;
}

bool RealcolonLoader::getRGBImage(size_t idx, cv::Mat& im) {
    if(idx >= vRGBPaths.size()) return false;

    //cout << "[RealcolonLoader]: loading image at " << vImgsPairs_[idx].first << endl;
    im = cv::imread(vRGBPaths[idx], cv::IMREAD_UNCHANGED);

    return true;
}

bool RealcolonLoader::getDepthImage(size_t idx, cv::Mat& im) {
    if(idx >= vRGBPaths.size()) return false;

    //cout << "[RealcolonLoader]: loading image at " << vImgsPairs_[idx].first << endl;
    im = cv::imread(vDepthPaths[idx], cv::IMREAD_UNCHANGED);

    return true;
}

bool RealcolonLoader::getTimeStamp(size_t idx, double &timestamp) {
    if(idx >= vRGBPaths.size()) return false;

    timestamp = vTimeStamps_[idx];

    return true;
}

int RealcolonLoader::getLength() {
    return (int)vTimeStamps_.size();
}

cv::Size RealcolonLoader::getImageSize() {
    return imSize_;
}