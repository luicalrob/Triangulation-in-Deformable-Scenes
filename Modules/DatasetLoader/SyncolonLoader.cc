#include "SyncolonLoader.h"

#include <iostream>
#include <fstream>
#include <iomanip>

#include <sys/stat.h>

using namespace std;

SyncolonLoader::SyncolonLoader(std::string folderPath, std::string timesPath) {
    ifstream fTimes;
    fTimes.open(timesPath.c_str());
    vTimeStamps_.reserve(5000);
    vRGBPaths.reserve(5000);
    vDepthPaths.reserve(5000);

    if(!fTimes.is_open()){
        cerr << "[SyncolonLoader]: Could not load dataset at " << folderPath << endl;
        return;
    }

    //Skip first 2 lines
    string s;
    getline(fTimes,s);
    //getline(fTimes,s);

    size_t i = 0;    
    while (!fTimes.eof()) {
        std::string s;
        std::getline(fTimes, s);
        
        if (!s.empty()) {
            std::stringstream ss(s);
            std::vector<std::string> tokens;
            std::string token;

            // Split the string by ';'
            while (std::getline(ss, token, ';')) {
                tokens.push_back(token);
            }

            // Ensure we have exactly 8 values (7 for pose + 1 timestamp)
            if (tokens.size() == 8) {
                PoseData pose;
                pose.tx = std::stod(tokens[0])/1000;
                pose.ty = std::stod(tokens[1])/1000;
                pose.tz = std::stod(tokens[2])/1000;
                pose.qx = std::stod(tokens[3]);
                pose.qy = std::stod(tokens[4]);
                pose.qz = std::stod(tokens[5]);
                pose.qw = std::stod(tokens[6]);
                pose.isValid = true;
                vPoseData_.push_back(pose);

                double timestamp = std::stod(tokens[7]);
                vTimeStamps_.push_back(timestamp);

                // Generate filenames with zero-padding
                std::ostringstream rgbName, depthName;

                rgbName << std::setw(4) << std::setfill('0') << i << ".png";
                depthName << std::setw(4) << std::setfill('0') << i << ".png";
                
                vRGBPaths.push_back(folderPath + "/rgb/" + rgbName.str());
                vDepthPaths.push_back(folderPath + "/depth/" + depthName.str());

                i++;
            } else {
                std::cerr << "[Error] Invalid line format: " << s << std::endl;
            }
        }
    }

    cv::Mat im;
    getRGBImage(0,im);
    imSize_ = im.size();
}

bool SyncolonLoader::getPoseData(size_t idx, PoseData& poseData) {
    if(idx >= vTimeStamps_.size()) return false;

    poseData = vPoseData_[idx];

    return true;
}

bool SyncolonLoader::getRGBImage(size_t idx, cv::Mat& im) {
    if(idx >= vTimeStamps_.size()) return false;

    //cout << "[SyncolonLoader]: loading image at " << vImgsPairs_[idx].first << endl;
    im = cv::imread(vRGBPaths[idx], cv::IMREAD_UNCHANGED);

    return true;
}

bool SyncolonLoader::getDepthImage(size_t idx, cv::Mat& im) {
    if(idx >= vTimeStamps_.size()) return false;

    //cout << "[SyncolonLoader]: loading image at " << vImgsPairs_[idx].first << endl;
    im = cv::imread(vDepthPaths[idx], cv::IMREAD_UNCHANGED);

    return true;
}

bool SyncolonLoader::getTimeStamp(size_t idx, double &timestamp) {
    if(idx >= vTimeStamps_.size()) return false;

    timestamp = vTimeStamps_[idx];

    return true;
}

int SyncolonLoader::getLength() {
    return (int)vTimeStamps_.size();
}

cv::Size SyncolonLoader::getImageSize() {
    return imSize_;
}