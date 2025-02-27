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
    vTimeStamps_.reserve(5000);
    vRGBPaths_.reserve(5000);
    vDepthPaths_.reserve(5000);

    ifstream fTimes;
    fTimes.open(timesPath.c_str());
    if(!fTimes.is_open()){
        cerr << "[RealcolonLoader]: Could not load dataset at " << folderPath << endl;
        return;
    }

    string s;
    getline(fTimes,s);
    getline(fTimes,s);

    size_t i = 0;    
    while (!fTimes.eof()) {
        std::string s;
        std::getline(fTimes, s);
        
        if (!s.empty()) {
            std::stringstream ss(s);
            std::vector<std::string> tokens;
            std::string token;

            // Split the string by ';'
            while (std::getline(ss, token, ',')) {
                tokens.push_back(token);
            }

            // Ensure we have exactly 8 values (7 for pose + 1 timestamp)
            if (tokens.size() == 13) {
                double timestamp = std::stod(tokens[0]);
                vTimeStamps_.push_back(timestamp);

                PoseData pose;
                pose.tx = std::stod(tokens[2]);
                pose.ty = std::stod(tokens[3]);
                pose.tz = std::stod(tokens[4]);
                pose.qw = std::stod(tokens[5]);
                pose.qx = std::stod(tokens[6]);
                pose.qy = std::stod(tokens[7]);
                pose.qz = std::stod(tokens[8]);
                pose.isValid = true;
                vPoseData_.push_back(pose);       

                // Generate filenames with zero-padding
                std::ostringstream rgbName, depthName;

                rgbName << std::setw(4) << std::setfill('0') << i << ".jpg";
                depthName << std::setw(4) << std::setfill('0') << i << ".png";
                
                vRGBPaths_.push_back(folderPath + "/rgb/" + rgbName.str());
                vDepthPaths_.push_back(folderPath + "/depth/" + depthName.str());

                i++;
            } else {
                std::cerr << "[Error] Invalid line format: " << s << std::endl;
            }
        }
    }

    if (!vRGBPaths_.empty()) {
        cv::Mat im;
        getRGBImage(0, im);
        imSize_ = im.size();
        std::cout << "imSize_: " << imSize_ << std::endl;
    }
}

bool RealcolonLoader::getPoseData(size_t idx, PoseData& poseData) {
    if(idx >= vRGBPaths_.size()) return false;

    poseData = vPoseData_[idx];

    return true;
}

bool RealcolonLoader::getRGBImage(size_t idx, cv::Mat& im) {
    if(idx >= vRGBPaths_.size()) return false;

    //cout << "[RealcolonLoader]: loading image at " << vImgsPairs_[idx].first << endl;
    im = cv::imread(vRGBPaths_[idx], cv::IMREAD_UNCHANGED);

    return true;
}

bool RealcolonLoader::getDepthImage(size_t idx, cv::Mat& im) {
    if(idx >= vRGBPaths_.size()) return false;

    //cout << "[RealcolonLoader]: loading image at " << vImgsPairs_[idx].first << endl;
    im = cv::imread(vDepthPaths_[idx], cv::IMREAD_UNCHANGED);

    return true;
}

bool RealcolonLoader::getTimeStamp(size_t idx, double &timestamp) {
    if(idx >= vRGBPaths_.size()) return false;

    timestamp = vTimeStamps_[idx];

    return true;
}

int RealcolonLoader::getLength() {
    return (int)vTimeStamps_.size();
}

cv::Size RealcolonLoader::getImageSize() {
    return imSize_;
}