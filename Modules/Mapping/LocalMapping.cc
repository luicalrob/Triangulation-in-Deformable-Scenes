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

#include "Mapping/LocalMapping.h"
#include "Matching/DescriptorMatching.h"
#include "Utils/Geometry.h"
#include "Utils/CommonTypes.h"
#include <iterator>

using namespace std;

LocalMapping::LocalMapping() {
}

LocalMapping::LocalMapping(Settings& settings, std::shared_ptr<Map> pMap) {
    settings_ = settings;
    pMap_ = pMap;

    TrianMethod_ = settings_.getTrianMethod();
    TrianLocation_ = settings_.getTrianLocation();
}

void LocalMapping::doMapping(std::shared_ptr<KeyFrame> &pCurrKeyFrame, int &nMPs) {
    //Keep input keyframe
    currKeyFrame_ = pCurrKeyFrame;

    if(!currKeyFrame_)
        return;

    //Remove redundant MapPoints
    //mapPointCulling();

    //Triangulate new MapPoints
    triangulateNewMapPoints();

    //checkDuplicatedMapPoints();

    nMPs = pMap_->getMapPoints().size();
}

void LocalMapping::doSimulatedMapping(std::shared_ptr<KeyFrame> &pCurrKeyFrame, std::shared_ptr<KeyFrame> &pPrevKeyFrame, int &nMPs) {
    //Keep input keyframe
    currKeyFrame_ = pCurrKeyFrame;
    prevKeyFrame_ = pPrevKeyFrame;

    if(!currKeyFrame_ || !prevKeyFrame_)
        return;

    //Triangulate new MapPoints
    triangulateSimulatedMapPoints();

    nMPs = pMap_->getMapPoints().size();
}

void LocalMapping::mapPointCulling() {
    std::list<std::shared_ptr<MapPoint>>::iterator lit = mlpRecentAddedMapPoints.begin();

    int currentKF_id = currKeyFrame_->getId();
    int accepted_0 = 0;
    int rejected_ratio = 0;
    int rejected_2 = 0;
    int accepted = 0;

    //int borrar = mlpRecentAddedMapPoints.size();

    while(lit != mlpRecentAddedMapPoints.end()) {
        
        std::shared_ptr<MapPoint> pMP = *lit;

        int mpID = (int)pMP->getId();

        size_t mpPosition = std::distance(mlpRecentAddedMapPoints.begin(), lit);

        int firstKeyframeID = pMap_->firstKeyFrameOfMapPoint(mpID);
        int nObservations = pMap_->getNumberOfObservations(mpID);
        
        if(firstKeyframeID == -1) {  // pointmap deleted
            auto nextLit = mlpRecentAddedMapPoints.erase(lit);
            lit = nextLit; 
            accepted_0++;
        } else {

            shared_ptr<KeyFrame> KFToCheck = pMap_->getKeyFrame(firstKeyframeID);
            int nKFMapPoints = KFToCheck->getNummberOfMapPoints();
            int KFID = KFToCheck->getId();

            vector<pair<ID,int>> vKFcovisible = pMap_->getCovisibleKeyFrames(KFID);
            int nvisible = 0;
            int nsameview = 0;

            float percentageOfSimilar = 0.0;

            for(int i = 0; i < vKFcovisible.size(); i++){
                float percentageOfSimilar = static_cast<float>(pMap_->numberOfCommonObservationsBetweenKeyFrames(KFID, vKFcovisible[i].first)) / nKFMapPoints;
                if(pMap_->isMapPointInKeyFrame(mpID, vKFcovisible[i].first)!=-1) {
                    nsameview++;
                    nvisible++;
                }else if(percentageOfSimilar >= 0.7) {
                    nsameview++;
                }
            }
            float foundRatio = static_cast<float>(nvisible)/nsameview;
            // cout << "nvisible: " << nvisible << endl;
            // cout << "nsameview: " << nsameview << endl;
            // cout << "foundRatio: " << foundRatio << endl;

            if(foundRatio < 0.25){
                pMap_->removeMapPoint(mpID);
                auto nextLit = mlpRecentAddedMapPoints.erase(lit);
                lit = nextLit; 
                rejected_ratio++;
            }else if((currentKF_id-firstKeyframeID)>=2 && nObservations<=2) {
                pMap_->removeMapPoint(mpID);
                auto nextLit = mlpRecentAddedMapPoints.erase(lit);
                lit = nextLit; 
                rejected_2++;
            }else if((currentKF_id-firstKeyframeID)>=3) {
                auto nextLit = mlpRecentAddedMapPoints.erase(lit);
                lit = nextLit; 
                accepted++;
            }
        }
        lit++;

    }
    // cout << "MP ID: " << mpID << endl;
    // cout << "nvisible: " << nvisible << endl;
    // cout << "observations: " << nObservations << endl;
    // cout << "foundRatio: " << foundRatio << endl;
    // cout << "accepted: " << accepted << endl;
    // cout << "accepted_0: " << accepted_0 << endl;
    // cout << "rejected_ratio: " << rejected_ratio << endl;
    // cout << "rejected_2: " << rejected_2 << endl;

}

void LocalMapping::triangulateNewMapPoints() {
    //Get a list of the best covisible KeyFrames with the current one
    vector<pair<ID,int>> vKeyFrameCovisible = pMap_->getCovisibleKeyFrames(currKeyFrame_->getId());

    vector<int> vMatches(currKeyFrame_->getMapPoints().size());

    //Get data from the current KeyFrame
    shared_ptr<CameraModel> calibration1 = currKeyFrame_->getCalibration();
    Sophus::SE3f T1w = currKeyFrame_->getPose();

    int nTriangulated = 0;

    for(pair<ID,int> pairKeyFrame_Obs : vKeyFrameCovisible){
        int commonObservations = pairKeyFrame_Obs.second;
        if(commonObservations < 20)
            continue;

        shared_ptr<KeyFrame> pKF = pMap_->getKeyFrame(pairKeyFrame_Obs.first);
        if(pKF->getId() == currKeyFrame_->getId())
            continue;

        //Check that baseline between KeyFrames is not too short
        Eigen::Vector3f vBaseLine = currKeyFrame_->getPose().inverse().translation() - pKF->getPose().inverse().translation();
        float medianDepth = pKF->computeSceneMedianDepth();
        float ratioBaseLineDepth = vBaseLine.norm() / medianDepth;

        if(ratioBaseLineDepth < 0.01){
            continue;
        }

        Sophus::SE3f T2w = pKF->getPose();

        Sophus::SE3f T21 = T2w*T1w.inverse();
        Eigen::Matrix<float,3,3> E = computeEssentialMatrixFromPose(T21);

        //Match features between the current and the covisible KeyFrame
        //TODO: this can be further improved using the orb vocabulary
        int nMatches = searchForTriangulation(currKeyFrame_.get(),pKF.get(),settings_.getMatchingForTriangulationTh(),
                settings_.getEpipolarTh(),E,vMatches);

        vector<cv::KeyPoint> vTriangulated1, vTriangulated2;
        vector<int> vMatches_;
        //Try to triangulate a new MapPoint with each match
        for(size_t i = 0; i < vMatches.size(); i++){
            if(vMatches[i] != -1){
                shared_ptr<CameraModel> calibration2 = pKF->getCalibration();
                auto x1 = currKeyFrame_->getKeyPoint(i).pt;
                auto x2 = pKF->getKeyPoint(vMatches[i]).pt;

                Eigen::Vector3f xn1 = calibration1->unproject(x1).normalized();
                Eigen::Vector3f xn2 = calibration2->unproject(x2).normalized();
                Eigen::Vector3f x3D_1, x3D_2;

                if(TrianMethod_ == "DepthMeasurement") {
                    float d1 = currKeyFrame_->getDepthMeasure(i);
                    float d2 = pKF->getDepthMeasure(i);

                    xn1 = calibration1->unproject(x1, d1);
                    xn2 = calibration2->unproject(x2, d2);
                }

                if (!triangulate(xn1, xn2, T1w, T2w, x3D_1, x3D_2)) continue;

                if (!isValidTriangulation(xn1, xn2, T1w, T2w, x3D_1, x3D_2)) continue;

                //Check reprojection error

                Eigen::Vector2f p_p1;
                Eigen::Vector2f p_p2;
                calibration1->project(T1w*x3D_1, p_p1);
                calibration2->project(T2w*x3D_2, p_p2);

                cv::Point2f cv_p1(p_p1[0], p_p1[1]);
                cv::Point2f cv_p2(p_p2[0], p_p2[1]);

                float e1 = squaredReprojectionError(x1, cv_p1);
                float e2 = squaredReprojectionError(x2, cv_p2);

                //if(e1 > 5.991 || e2 > 5.991) continue;

                std::shared_ptr<MapPoint> map_point_1(new MapPoint(x3D_1));
                std::shared_ptr<MapPoint> map_point_2(new MapPoint(x3D_2));

                pMap_->insertMapPoint(map_point_1);
                pMap_->insertMapPoint(map_point_2);

                mlpRecentAddedMapPoints.push_back(map_point_1);
                mlpRecentAddedMapPoints.push_back(map_point_2);

                pMap_->addObservation(currKeyFrame_->getId(), map_point_1->getId(), i);
                pMap_->addObservation(pKF->getId(), map_point_2->getId(), vMatches[i]);  // vMatches[i] si las parejas no fuesen ordenadas

                currKeyFrame_->setMapPoint(i, map_point_1);
                pKF->setMapPoint(vMatches[i], map_point_2);

                nTriangulated++;
                nTriangulated++;
            }
        }

        std::cout << "Triangulated " << nTriangulated << " MapPoints." << std::endl;
    }
}

void LocalMapping::triangulateSimulatedMapPoints() {
    int nTriangulated = 0;

    int nMatches = movedPoints_.size();

    std::cout << "Number of matches: " << nMatches << std::endl;

    Sophus::SE3f T1w = prevKeyFrame_->getPose();
    Sophus::SE3f T2w = currKeyFrame_->getPose();

    std::shared_ptr<CameraModel> prevCalibration = prevKeyFrame_->getCalibration();
    std::shared_ptr<CameraModel> currCalibration = currKeyFrame_->getCalibration();

    //Try to triangulate a new MapPoint with each match
    for(size_t i = 0; i < nMatches; i++){
        cv::Point2f x1 = prevKeyFrame_->getKeyPoint(i).pt; // vMatches[i] si las parejas no fuesen ordenadas
        cv::Point2f x2 = currKeyFrame_->getKeyPoint(i).pt;

        Eigen::Vector3f xn1 = prevCalibration->unproject(x1).normalized();
        Eigen::Vector3f xn2 = currCalibration->unproject(x2).normalized();

        if(TrianMethod_ == "DepthMeasurement") {
            float d1 = prevKeyFrame_->getDepthMeasure(i);
            float d2 = currKeyFrame_->getDepthMeasure(i);

            xn1 = prevCalibration->unproject(x1, d1);
            xn2 = currCalibration->unproject(x2, d2);
        }

        Eigen::Vector3f x3D_1, x3D_2;

        if (!triangulate(xn1, xn2, T1w, T2w, x3D_1, x3D_2)) continue;

        if (!isValidTriangulation(xn1, xn2, T1w, T2w, x3D_1, x3D_2)) continue;

        //Check reprojection error
        Eigen::Vector2f p_p1;
        Eigen::Vector2f p_p2;
        prevCalibration->project(T1w*x3D_1, p_p1);
        currCalibration->project(T2w*x3D_2, p_p2);

        cv::Point2f cv_p1(p_p1[0], p_p1[1]);
        cv::Point2f cv_p2(p_p2[0], p_p2[1]);

        float e1 = squaredReprojectionError(x1, cv_p1);
        float e2 = squaredReprojectionError(x2, cv_p2);

        //if(e1 > 5.991 || e2 > 5.991) continue;

        std::shared_ptr<MapPoint> map_point_1(new MapPoint(x3D_1));
        std::shared_ptr<MapPoint> map_point_2(new MapPoint(x3D_2));

        pMap_->insertMapPoint(map_point_1);
        pMap_->insertMapPoint(map_point_2);

        pMap_->addObservation(prevKeyFrame_->getId(), map_point_1->getId(), i);  // vMatches[i] si las parejas no fuesen ordenadas
        pMap_->addObservation(currKeyFrame_->getId(), map_point_2->getId(), i);

        prevKeyFrame_->setMapPoint(i, map_point_1);
        currKeyFrame_->setMapPoint(i, map_point_2);

        nTriangulated++;
        nTriangulated++;
    }

    std::cout << "Triangulated " << nTriangulated << " MapPoints." << std::endl;

    prevKeyFrame_->setInitialDepthScale();
    currKeyFrame_->setInitialDepthScale();
}

void LocalMapping::checkDuplicatedMapPoints() {
    vector<pair<ID,int>> vKFcovisible = pMap_->getCovisibleKeyFrames(currKeyFrame_->getId());
    vector<shared_ptr<MapPoint>> vCurrMapPoints = currKeyFrame_->getMapPoints();

    for(int i = 0; i < vKFcovisible.size(); i++){
        if(vKFcovisible[i].first == currKeyFrame_->getId())
            continue;
        int nFused = fuse(pMap_->getKeyFrame(vKFcovisible[i].first),settings_.getMatchingFuseTh(),vCurrMapPoints,pMap_.get());
        pMap_->checkKeyFrame(vKFcovisible[i].first);
        pMap_->checkKeyFrame(currKeyFrame_->getId());
    }
}

bool LocalMapping::isValidTriangulation(const Eigen::Vector3f& xn1, const Eigen::Vector3f& xn2, 
                                        const Sophus::SE3f& T1w, const Sophus::SE3f& T2w, 
                                        const Eigen::Vector3f& x3D_1, const Eigen::Vector3f& x3D_2) {
    auto x_1 = T1w * x3D_1;
    auto x_2 = T2w * x3D_2;

    if (x_1[2] < 0.0 || x_2[2] < 0.0) return false;

    auto ray1 = (T1w.inverse().rotationMatrix() * xn1).normalized();
    auto ray2 = (T2w.inverse().rotationMatrix() * xn2).normalized();
    auto parallax = cosRayParallax(ray1, ray2);

    return parallax <= settings_.getMinCos();
}

bool LocalMapping::triangulate(const Eigen::Vector3f& xn1, const Eigen::Vector3f& xn2, 
                               const Sophus::SE3f& T1w, const Sophus::SE3f& T2w, 
                               Eigen::Vector3f& x3D_1, Eigen::Vector3f& x3D_2) {
    if (TrianMethod_ == "Classic") {
        triangulateClassic(xn1, xn2, T1w, T2w, x3D_1, x3D_2, TrianLocation_);
    } else if (TrianMethod_ == "ORBSLAM") {
        triangulateORBSLAM(xn1, xn2, T1w, T2w, x3D_1, x3D_2, TrianLocation_);
    } else if (TrianMethod_ == "DepthMeasurement") {
        triangulateDepth(xn1, xn2, T1w, T2w, x3D_1, x3D_2, TrianLocation_);
    } else {
        triangulateNRSLAM(xn1, xn2, T1w, T2w, x3D_1, x3D_2, TrianLocation_);
    }
    return true;
}