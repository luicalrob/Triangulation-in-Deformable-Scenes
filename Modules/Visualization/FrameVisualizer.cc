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



#include "FrameVisualizer.h"

using namespace std;

FrameVisualizer::FrameVisualizer(const bool showScene): showScene_(showScene){}

void FrameVisualizer::setReferenceFrame(std::vector<cv::KeyPoint> &vKeys, cv::Mat &im) {
    vRefKeys_ = vKeys;
    refIm_ = im.clone();
}

void FrameVisualizer::drawFrameMatches(std::vector<cv::KeyPoint> &vKeys, cv::Mat &im, std::vector<int>& vMatches) {
    vector<cv::DMatch> vDMatches;
    vector<cv::KeyPoint> vGoodKeys1, vGoodKeys2;
    if(showScene_) {
        for(size_t i = 0, j = 0; i < vMatches.size(); i++){
            if(vMatches[i] != -1){
                vGoodKeys1.push_back(vRefKeys_[i]);
                vGoodKeys2.push_back(vKeys[vMatches[i]]);
                vDMatches.push_back(cv::DMatch(j,j,0));

                j++;
            }
        }

        cv::Mat imMatches;
        cv::drawMatches(refIm_,vGoodKeys1,im,vGoodKeys2,vDMatches,imMatches);
        cv::imshow("Matches between frames", imMatches);
    }
}

void FrameVisualizer::drawFrameTriangulatedMatches(const std::shared_ptr<Map> pMap, std::vector<cv::KeyPoint> &vKeys, cv::Mat &im, std::vector<int>& vMatches) {
    vector<cv::DMatch> vDMatches;
    vector<cv::KeyPoint> vGoodKeys1, vGoodKeys2;

    if(showScene_) {
        std::unordered_map<ID,std::shared_ptr<KeyFrame>>&  mKeyFrames = pMap->getKeyFrames();

        for (auto k1 = mKeyFrames.begin(); k1 != mKeyFrames.end(); ++k1) {
            for (auto k2 = std::next(k1); k2 != mKeyFrames.end(); ++k2) {
                std::shared_ptr<KeyFrame> pKF1 = k2->second;
                std::shared_ptr<KeyFrame> pKF2 = k1->second;
                int kf1ID = k2->first;
                int kf2ID = k1->first;

                vector<std::shared_ptr<MapPoint>>& v1MPs = pKF1->getMapPoints();
                vector<std::shared_ptr<MapPoint>>& v2MPs = pKF2->getMapPoints();
                

                for (size_t i = 0, j = 0; i < v1MPs.size(); i++) {
                    std::shared_ptr<MapPoint> pMPi1, pMPi2;
                    pMPi1 = v1MPs[i];
                    pMPi2 = v2MPs[i];
                    if (!pMPi1) continue;
                    if (!pMPi2) continue;

                    int index_in_kf1 = pMap->isMapPointInKeyFrame(pMPi1->getId(), kf1ID);
                    int index_in_kf2 = pMap->isMapPointInKeyFrame(pMPi2->getId(), kf2ID);

                    if(index_in_kf1 < 0 || index_in_kf2 < 0) continue;
                    size_t idx1 = (size_t)index_in_kf1;
                    size_t idx2 = (size_t)index_in_kf2;

                    cv::Point2f x1 = pKF1->getKeyPoint(idx1).pt;
                    cv::Point2f x2 = pKF2->getKeyPoint(idx2).pt;

                    vGoodKeys1.push_back(vRefKeys_[idx1]);
                    vGoodKeys2.push_back(vKeys[idx2]);
                    vDMatches.push_back(cv::DMatch(j,j,0));
                    j++;
                }
            }
        }

        cv::Mat imMatches;
        cv::drawMatches(refIm_,vGoodKeys1,im,vGoodKeys2,vDMatches,imMatches);
        cv::imshow("Matches triangulated between keyframes", imMatches);
    }
}

void FrameVisualizer::drawMatches(std::vector<cv::KeyPoint> &vKeys1, cv::Mat &im1, std::vector<cv::KeyPoint> &vKeys2, cv::Mat &im2, std::vector<int> &vMatches) {
    vector<cv::DMatch> vDMatches;
    vector<cv::KeyPoint> vGoodKeys1, vGoodKeys2;

    if(showScene_) {
        for(size_t i = 0, j = 0; i < vMatches.size(); i++){
            if(vMatches[i] != -1){
                vGoodKeys1.push_back(vKeys1[i]);
                vGoodKeys2.push_back(vKeys2[vMatches[i]]);
                vDMatches.push_back(cv::DMatch(j,j,0));

                j++;
            }
        }

        cv::Mat imMatches;
        cv::drawMatches(im1,vGoodKeys1,im2,vGoodKeys2,vDMatches,imMatches);
        cv::imshow("Matches between KeyFrames", imMatches);
    }
}

void FrameVisualizer::drawCurrentFeatures(std::vector<cv::KeyPoint> &vKeys, cv::Mat &im) {
    if(showScene_) {
        cv::Mat imWithKeys;
        cv::drawKeypoints(im,vKeys,imWithKeys);
        cv::imshow("KeyPoints extracted", imWithKeys);
    }
}

void FrameVisualizer::drawFeatures(const std::vector<cv::KeyPoint> &vKeys, cv::Mat &im, const std::string &windowName) {
    if(showScene_) {
        cv::Mat imWithKeys;
        cv::drawKeypoints(im, vKeys, imWithKeys);
        cv::imshow(windowName, imWithKeys);
    }
}


void FrameVisualizer::drawCurrentFrame(Frame &f, std::string text) {
    if(showScene_) {
        cv::Mat im = f.getIm().clone();
        cvtColor(im,im,cv::COLOR_GRAY2BGR);

        auto vKeys = f.getKeyPointsDistorted();
        auto vMapPoints = f.getMapPoints();

        int nMatches = 0;

        for(size_t i = 0; i < vKeys.size(); i++){
            if(vMapPoints[i]){
                cv::Point2f pt1, pt2;
                pt1.x=vKeys[i].pt.x-5;
                pt1.y=vKeys[i].pt.y-5;
                pt2.x=vKeys[i].pt.x+5;
                pt2.y=vKeys[i].pt.y+5;

                cv::rectangle(im,pt1,pt2,cv::Scalar(0,255,0));
                cv::circle(im,vKeys[i].pt,2,cv::Scalar(0,255,0),-1);

                nMatches++;
            }
        }

        stringstream s;
        s << "SLAM MODE | Matches: " << nMatches;

        int baseline=0;
        cv::Size textSize = cv::getTextSize(s.str(),cv::FONT_HERSHEY_PLAIN,1,1,&baseline);
        cv::Mat imText = cv::Mat(im.rows+textSize.height+10,im.cols,im.type());
        im.copyTo(imText.rowRange(0,im.rows).colRange(0,im.cols));
        imText.rowRange(im.rows,imText.rows) = cv::Mat::zeros(textSize.height+10,im.cols,im.type());
        cv::putText(imText,s.str(),cv::Point(5,imText.rows-5),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,255,255),1,8);

        std::string windowName = "SLAM: current Frame " + text;
        cv::imshow(windowName, imText);
    }
}

void FrameVisualizer::drawFrameDepthImage(Frame &f, std::string text) {
    if(showScene_) {
        cv::Mat im = f.getDepthIm().clone();

        stringstream s;
        s << "SLAM MODE | depth ";

        int baseline=0;
        cv::Size textSize = cv::getTextSize(s.str(),cv::FONT_HERSHEY_PLAIN,1,1,&baseline);
        cv::Mat imText = cv::Mat(im.rows+textSize.height+10,im.cols,im.type());
        im.copyTo(imText.rowRange(0,im.rows).colRange(0,im.cols));
        imText.rowRange(im.rows,imText.rows) = cv::Mat::zeros(textSize.height+10,im.cols,im.type());
        cv::putText(imText,s.str(),cv::Point(5,imText.rows-5),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,255,255),1,8);

        std::string windowName = "SLAM: depth image " + text;
        cv::imshow(windowName, imText);
    }
}

void FrameVisualizer::updateWindows() {
    cv::waitKey(10);
}