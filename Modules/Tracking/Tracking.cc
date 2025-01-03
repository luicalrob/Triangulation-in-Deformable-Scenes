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


#include "Tracking.h"

#include "Features/FAST.h"
#include "Features/ORB.h"

#include "Map/KeyFrame.h"
#include "Map/MapPoint.h"
#include "Matching/DescriptorMatching.h"
#include "Utils/Utils.h"

#include "Optimization/g2oBundleAdjustment.h"

using namespace std;

Tracking::Tracking(){}

Tracking::Tracking(Settings& settings, std::shared_ptr<FrameVisualizer>& visualizer,
                    std::shared_ptr<MapVisualizer>& mapVisualizer, std::shared_ptr<Map> map) {
    currFrame_ = Frame(settings.getFeaturesPerImage(),settings.getGridCols(),settings.getGridRows(),
                       settings.getImCols(),settings.getImRows(), settings.getNumberOfScales(), settings.getScaleFactor(),
                       settings.getCalibration(),settings.getDistortionParameters(), settings.getDepthMeasurementsScale());
    prevFrame_ = Frame(settings.getFeaturesPerImage(),settings.getGridCols(),settings.getGridRows(),
                       settings.getImCols(),settings.getImRows(),settings.getNumberOfScales(), settings.getScaleFactor(),
                       settings.getCalibration(),settings.getDistortionParameters(), settings.getDepthMeasurementsScale());

    featExtractor_ = shared_ptr<Feature>(new FAST(settings.getNumberOfScales(),settings.getScaleFactor(),settings.getFeaturesPerImage()*2,20,7));
    descExtractor_ = shared_ptr<Descriptor>(new ORB(settings.getNumberOfScales(),settings.getScaleFactor()));

    vMatches_ = vector<int>(settings.getFeaturesPerImage());

    vPrevMatched_ = vector<cv::Point2f>(settings.getFeaturesPerImage());

    status_ = NOT_INITIALIZED;
    bFirstIm_ = true;
    bMotionModel_ = false;

    monoInitializer_ = MonocularMapInitializer(settings.getFeaturesPerImage(),settings.getCalibration(),settings.getEpipolarTh(),
                                                settings.getMinCos(), settings.getTrianMethod(), settings.getTrianLocation());

    visualizer_ = visualizer;
    mapVisualizer_ = mapVisualizer;

    pMap_ = map;

    nLastKeyFrameId = 0;
    nFramesFromLastKF_ = 0;

    bInserted = false;

    filePath_ = settings.getExpFilePath();
    outFile_.imbue(std::locale("es_ES.UTF-8"));

    settings_ = settings;
}

bool Tracking::doTracking(const cv::Mat &im, const cv::Mat &depthIm, Sophus::SE3f &Tcw, int &nKF, int &nMPs, clock_t &timer) {
    currIm_ = im.clone();
    cv::Mat dIm = depthIm.clone();

    currFrame_.setPose(Tcw);

    //Update previous frame
    // if(status_ != NOT_INITIALIZED)
    //     prevFrame_.assign(currFrame_);
    
    Sophus::SE3f pos = prevFrame_.getPose();
    Eigen::Vector3f translation = pos.translation();

    currFrame_.setIm(currIm_);
    currFrame_.setDepthIm(dIm);

    //Extract features in the current image
    extractFeatures(im);

    visualizer_->drawCurrentFeatures(currFrame_.getKeyPointsDistorted(),currIm_);

    //If no map is initialized, perform monocular initialization
    if(status_ == NOT_INITIALIZED){
        Tcw_ = Tcw;
        if(monocularMapInitialization()){
            status_ = GOOD;
            //Tcw = currFrame_.getPose();

            //Update motion model
            //updateMotionModel();
        
            return true;
        }
        else{
            return false;
        }
    }
    //SLAM is initialized and tracking was good, track new frame
    else if(status_ == GOOD){
        //Mapping may has added/deleted MapPoints
        updateLastMapPoints();
        cout << "updateLastMapPoints " << endl;
        if(cameraTracking()){
            if(trackLocalMap()){
                //Check if we need to insert a new KeyFrame into the system
                if(needNewKeyFrame(nKF)){
                    promoteCurrentFrameToKeyFrame();
                }

                //Update motion model
                //updateMotionModel();

                //Tcw = currFrame_.getPose();

                visualizer_->drawCurrentFrame(currFrame_);

                return true;
            }
            else{
                status_ = LOST;
                timer = clock() - timer;
                nMPs = pMap_->getMapPoints().size();
	            cout << "[LOST] Seconds: " << fixed << setprecision(4) << ((float)timer)/CLOCKS_PER_SEC << endl;
                cout << "[LOST] Number of Keyframes: " << nKF << endl;
                cout << "[LOST] Number of MapPoints: " << nMPs << endl;
                return false;
            }
        }
        else{
            status_ = LOST;
            timer = clock() - timer;
            nMPs = pMap_->getMapPoints().size();
	        cout << "[LOST] Seconds: " << fixed << setprecision(4) << ((float)timer)/CLOCKS_PER_SEC << endl;
            cout << "[LOST] Number of Keyframes: " << nKF << endl;
            cout << "[LOST] Number of MapPoints: " << nMPs << endl;
            return false;
        }
    }
    //Camera tracking failed last frame, try to rellocalise
    else{
        //Not implemented yet
        return false;
    }
}

void Tracking::updateLastMapPoints() {
    if(bInserted){
        vector<shared_ptr<MapPoint>> vMps = pMap_->getKeyFrame(nLastKeyFrameId)->getMapPoints();
        Sophus::SE3f Tcw = pMap_->getKeyFrame(nLastKeyFrameId)->getPose();
        prevFrame_.setPose(Tcw);

        cout << "vMps.size(): " << vMps.size() << endl;
        for(size_t i = 0; i < vMps.size(); i++){
            if(vMps[i]){
                prevFrame_.setMapPoint(i,vMps[i]);
            }
            else{
                prevFrame_.setMapPoint(i,nullptr);
            }
        }

        bInserted = false;
    }
}

void Tracking::extractFeatures(const cv::Mat &im) {
    //Extracf image features
    featExtractor_->extract(im,currFrame_.getKeyPointsDistorted());

    //Compute descriptors to extracted features
    descExtractor_->describe(im,currFrame_.getKeyPointsDistorted(),currFrame_.getDescriptors());

    //Distribute keys and undistort them
    currFrame_.distributeFeatures();
}

bool Tracking::monocularMapInitialization() {
    //Set first frame received as the reference frame

    if(bFirstIm_){
        monoInitializer_.changeReference(currFrame_.getKeyPoints());
        prevFrame_.assign(currFrame_);
        prevFrame_.setPose(Tcw_);

        bFirstIm_ = false;

        visualizer_->setReferenceFrame(prevFrame_.getKeyPointsDistorted(),currIm_);

        for(size_t i = 0; i < vPrevMatched_.size(); i++){
            vPrevMatched_[i] = prevFrame_.getKeyPoint(i).pt;
        }

        return false;
    }

    //Find matches between previous and current frame
    int nMatches = searchForInitializaion(prevFrame_,currFrame_,settings_.getMatchingInitTh(),vMatches_,vPrevMatched_);

    std::cerr << "nMatches: "<< nMatches << std::endl;

    visualizer_->drawFrameMatches(currFrame_.getKeyPointsDistorted(),currIm_,vMatches_);

    //If not enough matches found, updtate reference frame
    if(nMatches < 70){
        monoInitializer_.changeReference(currFrame_.getKeyPoints());
        prevFrame_.assign(currFrame_);
        prevFrame_.setPose(Tcw_);

        visualizer_->setReferenceFrame(prevFrame_.getKeyPointsDistorted(),currIm_);

        for(size_t i = 0; i < vPrevMatched_.size(); i++){
            vPrevMatched_[i] = prevFrame_.getKeyPoint(i).pt;
        }

        return false;
    }

    //Try to initialize by finding an Essential matrix
    Sophus::SE3f Tcw_prev = prevFrame_.getPose();
    Sophus::SE3f Tcw = Tcw_;
    vector<Eigen::Vector3f> v3DPoints;
    v3DPoints.reserve(vMatches_.capacity());
    vector<bool> vTriangulated(vMatches_.capacity(),false);
    if(!monoInitializer_.initialize(prevFrame_, currFrame_, vMatches_, nMatches, v3DPoints, vTriangulated)){
        return false;
    }

    //Get map scale
    vector<float> vDepths;
    for(int i = 0, j = 0; i < vTriangulated.size(); i++, j+=2){
        if(vTriangulated[i]) {
            vDepths.push_back(v3DPoints[j](2));
            vDepths.push_back(v3DPoints[j+1](2));
        }
    }

    nth_element(vDepths.begin(),vDepths.begin()+vDepths.size()/2,vDepths.end());
    const float scale = vDepths[vDepths.size()/2];

    //Create map
    //Tcw.translation() = Tcw.translation() / scale;

    //currFrame_.setPose(Tcw);

    int nTriangulated = 0;

    for(int i = 0, j = 0; i < vTriangulated.size(); i++, j+=2){
        if(vTriangulated[i]){
            //Eigen::Vector3f v = v3DPoints[i] / scale;
            shared_ptr<MapPoint> pMP1(new MapPoint(v3DPoints[j]));
            shared_ptr<MapPoint> pMP2(new MapPoint(v3DPoints[j+1]));

            prevFrame_.setMapPoint(i,pMP1);
            currFrame_.setMapPoint(i,pMP2);

            pMap_->insertMapPoint(pMP1);
            pMap_->insertMapPoint(pMP2);

            Eigen::Vector3f x3D_mp1 = pMP1->getWorldPosition();
            Eigen::Vector3f x3D_mp2 = pMP2->getWorldPosition();

            nTriangulated++;
            nTriangulated++;
        }
    }

    cout << "Map initialized with " << nTriangulated << " MapPoints" << endl;

    shared_ptr<KeyFrame> kf0(new KeyFrame(prevFrame_));
    shared_ptr<KeyFrame> kf1(new KeyFrame(currFrame_));

    pMap_->insertKeyFrame(kf0);
    pMap_->insertKeyFrame(kf1);

    // Eigen::Vector3f O3 = kf0->getPose().translation();
    // std::cout << "kf0 id: " << kf0->getId() <<  std::endl;
    // std::cout << "kf0 Translation: " << O3[0]  << " " << O3[1]  << " "  << O3[2] << " " << std::endl;
    // Eigen::Vector3f O2 = kf1->getPose().translation();
    // std::cout << "kf1 id: " << kf1->getId() <<  std::endl;
    // std::cout << "kf1 Translation: " << O2[0]  << " " << O2[1]  << " "  << O2[2] << " " << std::endl;

    //Set observations into the map
    vector<shared_ptr<MapPoint>>& vMapPoints1 = kf0->getMapPoints();
    std::unordered_map<long unsigned int, size_t> ids_map;
    for(size_t i = 0; i < vMapPoints1.size(); i++){
        auto pMP = vMapPoints1[i];
        if(pMP){
            // std::cout << "addObservation pMP2 " << pMP->getId() << " i: " << i << " vMatches_[i]: " << vMatches_[i] << std::endl;
            //Add observation
            pMap_->addObservation(0,pMP->getId(),i);
            pMap_->addObservation(1,pMP->getId(),vMatches_[i]);
            ids_map[pMP->getId() + 1] = i;
        }
    }

    vector<shared_ptr<MapPoint>>& vMapPoints2 = kf1->getMapPoints();
    for(size_t idx = 0; idx < vMapPoints2.size(); idx++){
        auto pMP = vMapPoints2[idx];
        if(pMP){
            size_t i = 0;
            long unsigned int id = pMP->getId();
            if (ids_map.find(id) != ids_map.end()) {
                i = ids_map[id];
            } else {
                continue;
            }

            //Add observation
            // std::cout << "addObservation pMP2 " << pMP->getId() << " i: " << i << " vMatches_[i]: " << vMatches_[i] << std::endl;
            pMap_->addObservation(0,pMP->getId(),i);
            pMap_->addObservation(1,pMP->getId(),vMatches_[i]);
        }
    }

    //Run a Bundle Adjustment to refine the solution
    //bundleAdjustment(pMap_.get());
    //std::unordered_map<ID, std::shared_ptr<MapPoint>> mapPoints_corrected = pMap_->getMapPoints();


    std::cout << "\nINITIAL MEASUREMENTS: \n";
    outFile_.open(filePath_);
    if (outFile_.is_open()) {
        outFile_ << "INITIAL MEASUREMENTS: \n";

        outFile_.close();
    } else {
        std::cerr << "Unable to open file for writing" << std::endl;
    }
    
    stopWithMeasurements(pMap_, Tcw, mapVisualizer_, filePath_ , settings_.getDrawRaysSelection(), 
                            settings_.getStopExecutionOption(), settings_.getShowScene());

    deformationOptimization(pMap_, settings_, mapVisualizer_);

    // Tcw = kf1->getPose();
    // currFrame_.setPose(Tcw);

    // updateMotionModel();

    pLastKeyFrame_ = kf1;
    nLastKeyFrameId = kf1->getId();

    bInserted = true;

    return true;
}

bool Tracking::cameraTracking() {
    //Set pose estimation for the current frame with the motion model
    Sophus::SE3f currPose;
    // if(bMotionModel_){
    //     currPose = motionModel_ * prevFrame_.getPose();
    // }
    // else{
    //     currPose = prevFrame_.getPose();
    //     bMotionModel_ = true;
    // }

    //currFrame_.setPose(currPose);
    currFrame_.setPose(Tcw_);

    //Match features between current and previous frame
    int nMatches = guidedMatching(prevFrame_,currFrame_,settings_.getMatchingGuidedTh(),vMatches_,1);

    if(nMatches < 20){
        nMatches = guidedMatching(prevFrame_,currFrame_,settings_.getMatchingGuidedTh(),vMatches_,2);
    }

    //Run a pose optimization
    //nFeatTracked_ = poseOnlyOptimization(currFrame_);

    //currFrame_.checkAllMapPointsAreGood();

    //Update MapDrawer
    currPose = currFrame_.getPose();
    mapVisualizer_->updateCurrentPose(currPose);

    //We enforce a minimum of 20 MapPoint matches to consider the estimation as good

    cout << "guidedMatching nMatches: " << nMatches << endl;
    return nMatches >= 20;
}

bool Tracking::trackLocalMap() {
    //Get local map from the last KeyFrame
    //currFrame_.checkAllMapPointsAreGood();

    set<ID> sLocalMapPoints, sLocalKeyFrames, sFixedKeyFrames;
    pMap_->getLocalMapOfKeyFrame(nLastKeyFrameId,sLocalMapPoints,sLocalKeyFrames,sFixedKeyFrames);

    //Keep a record of the already tracked map points
    vector<shared_ptr<MapPoint>>& vTrackedMapPoints = currFrame_.getMapPoints();
    unordered_set<ID> sTrackedMapPoints;
    for(auto pMP : vTrackedMapPoints){
        if(pMP)
            sTrackedMapPoints.insert(pMP->getId());
    }

    //Project local map points into the current Frame
    vector<shared_ptr<MapPoint>> vMapPointsToMatch;
    vMapPointsToMatch.reserve(sLocalKeyFrames.size());
    for(auto nMapPointId : sLocalMapPoints){
        //Check if this local MapPoint is already been tracked
        if(sTrackedMapPoints.count(nMapPointId) != 0){
            continue;
        }

        vMapPointsToMatch.push_back(pMap_->getMapPoint(nMapPointId));
    }


    int nMatches = searchWithProjection(currFrame_,settings_.getMatchingByProjectionTh(),vMapPointsToMatch);


    //Run a pose optimization
    //nFeatTracked_ = poseOnlyOptimization(currFrame_);

    //currFrame_.checkAllMapPointsAreGood();

    //Update MapDrawer
    Sophus::SE3f currPose = currFrame_.getPose();
    mapVisualizer_->updateCurrentPose(currPose);

    //We enforce a minimum of 20 MapPoint matches to consider the estimation as good
    return nMatches >= 20;
}

bool Tracking::needNewKeyFrame(int &nKF) {
    /*
     * Your code for Lab 4 - Task 1 here!
     */
    int minTrackedFeat = 100;
    int maxFramesBtwKF = 0;  //2, 4, 6
    if(nFramesFromLastKF_ > maxFramesBtwKF || nFeatTracked_ < minTrackedFeat || status_ == LOST) {
        if(status_ != LOST) {
            nKF++;
        }
        nFramesFromLastKF_ = 0;
        return true;
    }
    nFramesFromLastKF_++;
    return false;
}

void Tracking::promoteCurrentFrameToKeyFrame() {
    //Promote current frame to KeyFrame
    pLastKeyFrame_ = shared_ptr<KeyFrame>(new KeyFrame(currFrame_));

    //Insert KeyFrame into the map
    pMap_->insertKeyFrame(pLastKeyFrame_);

    //Add all obsevations into the map
    nLastKeyFrameId = pLastKeyFrame_->getId();
    vector<shared_ptr<MapPoint>>& vMapPoints = pLastKeyFrame_->getMapPoints();
    for(int i = 0; i < vMapPoints.size(); i++){
        MapPoint* pMP = vMapPoints[i].get();
        if(pMP)
            pMap_->addObservation(nLastKeyFrameId,pMP->getId(),i);
    }
    pMap_->checkKeyFrame(pLastKeyFrame_->getId());

    bInserted = true;
}

std::shared_ptr<KeyFrame> Tracking::getLastKeyFrame() {
    shared_ptr<KeyFrame> toReturn = pLastKeyFrame_;
    pLastKeyFrame_ = nullptr;

    return toReturn;
}

void Tracking::updateMotionModel() {
    motionModel_ = currFrame_.getPose() * prevFrame_.getPose().inverse();
}
