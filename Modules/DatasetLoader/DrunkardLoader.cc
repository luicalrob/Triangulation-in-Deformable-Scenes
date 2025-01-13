#include "DrunkardLoader.h"

#include <iostream>
#include <fstream>

#include <sys/stat.h>

using namespace std;

DrunkardLoader::DrunkardLoader(std::string folderPath, std::string timesPath) {
    ifstream fTimes;
    fTimes.open(timesPath.c_str());
    vTimeStamps_.reserve(5000);
    vRGBPaths.reserve(5000);
    vDepthPaths.reserve(5000);

    if(!fTimes.is_open()){
        cerr << "[drunkardLoader]: Could not load dataset at " << folderPath << endl;
        return;
    }

    while(!fTimes.eof()){
        string s;
        getline(fTimes,s);
        if(!s.empty()){
            stringstream ss;
            ss << s;

            string t;
            ss >> t;
            vTimeStamps_.push_back(std::stod(t));

            PoseData pose;
            ss >> pose.tx >> pose.ty >> pose.tz 
                >> pose.qx >> pose.qy >> pose.qz >> pose.qw;
                pose.isValid = true;
            vPoseData_.push_back(pose);

            vRGBPaths.push_back(folderPath + "/color/" + t + ".png");
            vDepthPaths.push_back(folderPath + "/depth/" + t + ".png");
        }
    }

    cv::Mat im;
    getRGBImage(0,im);
    imSize_ = im.size();
}

bool DrunkardLoader::getPoseData(size_t idx, PoseData& poseData) {
    if(idx >= vTimeStamps_.size()) return false;

    poseData = vPoseData_[idx];

    return true;
}

bool DrunkardLoader::getRGBImage(size_t idx, cv::Mat& im) {
    if(idx >= vTimeStamps_.size()) return false;

    //cout << "[DrunkardLoader]: loading image at " << vImgsPairs_[idx].first << endl;
    im = cv::imread(vRGBPaths[idx], cv::IMREAD_UNCHANGED);

    return true;
}

bool DrunkardLoader::getDepthImage(size_t idx, cv::Mat& im) {
    if(idx >= vTimeStamps_.size()) return false;

    //cout << "[DrunkardLoader]: loading image at " << vImgsPairs_[idx].first << endl;
    im = cv::imread(vDepthPaths[idx], cv::IMREAD_UNCHANGED);

    return true;
}

bool DrunkardLoader::getTimeStamp(size_t idx, double &timestamp) {
    if(idx >= vTimeStamps_.size()) return false;

    timestamp = vTimeStamps_[idx];

    return true;
}

int DrunkardLoader::getLength() {
    return (int)vTimeStamps_.size();
}

cv::Size DrunkardLoader::getImageSize() {
    return imSize_;
}
