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


#include "Mapping.h"

#include "Features/FAST.h"
#include "Features/ORB.h"

#include "Map/KeyFrame.h"
#include "Map/MapPoint.h"
#include "Matching/DescriptorMatching.h"
#include "Utils/Utils.h"
#include "Utils/Geometry.h"

#include "Optimization/g2oBundleAdjustment.h"

using namespace std;

Mapping::Mapping(){}

Mapping::Mapping(Settings& settings, std::shared_ptr<FrameVisualizer>& visualizer,
                    std::shared_ptr<MapVisualizer>& mapVisualizer, std::shared_ptr<Map> map) {
    refFrame_ = Frame(settings.getFeaturesPerImage(),settings.getGridCols(),settings.getGridRows(),
                       settings.getImCols(),settings.getImRows(),settings.getNumberOfScales(), settings.getScaleFactor(),
                       settings.getCalibration(),settings.getDistortionParameters(), settings.getSimulatedDepthScaleC1());

    currFrame_ = Frame(settings.getFeaturesPerImage(),settings.getGridCols(),settings.getGridRows(),
                       settings.getImCols(),settings.getImRows(), settings.getNumberOfScales(), settings.getScaleFactor(),
                       settings.getCalibration(),settings.getDistortionParameters(), settings.getSimulatedDepthScaleC2());

    featExtractor_ = shared_ptr<Feature>(new FAST(settings.getNumberOfScales(),settings.getScaleFactor(),settings.getFeaturesPerImage()*2,20,7, settings.getBorderMask()));
    descExtractor_ = shared_ptr<Descriptor>(new ORB(settings.getNumberOfScales(),settings.getScaleFactor()));

    vMatches_ = vector<int>(settings.getFeaturesPerImage());

    vPrevMatched_ = vector<cv::Point2f>(settings.getFeaturesPerImage());

    TrianMethod_ = settings.getTrianMethod();
    TrianLocation_ = settings.getTrianLocation();
    depthLimit_ = settings.getDepthLimit();

    status_ = NOT_INITIALIZED;
    bFirstIm_ = true;

    monoInitializer_ = MonocularMapInitializer(settings.getFeaturesPerImage(),settings.getCalibration(),settings.getEpipolarTh(), settings.getMinCos(), 
                                        settings.getTrianMethod(), settings.getTrianLocation(), settings.getCheckingSelection(), settings.getDepthLimit());

    visualizer_ = visualizer;
    mapVisualizer_ = mapVisualizer;

    pMap_ = map;

    filePath_ = settings.getExpFilePath();
    outFile_.imbue(std::locale("es_ES.UTF-8"));

    settings_ = settings;
}

bool Mapping::doMapping(const cv::Mat &im, const cv::Mat &depthIm, Sophus::SE3f &Tcw, int &nKF, int &nMPs, clock_t &timer) {
    currIm_ = im.clone();
    cv::Mat dIm = depthIm.clone();

    currFrame_.setIm(currIm_);
    currFrame_.setDepthIm(dIm);

    //Extract features in the current image
    extractFeatures(im);

    visualizer_->drawCurrentFeatures(currFrame_.getKeyPointsDistorted(),currIm_);

    //If no map is initialized, perform monocular initialization
    if(status_ == NOT_INITIALIZED){
        if(monocularMapInitialization()){
            status_ = GOOD;
            Tcw = currFrame_.getPose();
        
            return true;
        }
        else{
            return false;
        }
    } else {
        //Not implemented yet
        return false;
    }
}

void Mapping::doSimulatedMapping(std::shared_ptr<KeyFrame> &pCurrKeyFrame, std::shared_ptr<KeyFrame> &pPrevKeyFrame, int &nMPs) {
    //Keep input keyframe
    currKeyFrame_ = pCurrKeyFrame;
    refKeyFrame_ = pPrevKeyFrame;

    if(!currKeyFrame_ || !refKeyFrame_)
        return;

    //Triangulate new MapPoints
    triangulateSimulatedMapPoints();

    nMPs = pMap_->getMapPoints().size();
}

void Mapping::extractFeatures(const cv::Mat &im) {
    //Extracf image features
    featExtractor_->extract(im,currFrame_.getKeyPointsDistorted());

    //Compute descriptors to extracted features
    descExtractor_->describe(im,currFrame_.getKeyPointsDistorted(),currFrame_.getDescriptors());

    //Distribute keys and undistort them
    currFrame_.distributeFeatures();
}

bool Mapping::monocularMapInitialization() {
    //Set first frame received as the reference frame

    if(bFirstIm_){
        refFrame_.assign(currFrame_);
        visualizer_->setReferenceFrame(refFrame_.getKeyPointsDistorted(),currIm_);

        bFirstIm_ = false;

        return false;
    }

    //Find matches between previous and current frame
    int nMatches = searchForInitializaion(refFrame_,currFrame_,settings_.getMatchingInitTh(), settings_.getMatchingInitRadius(),vMatches_);

    std::cerr << "nMatches: "<< nMatches << std::endl;

    visualizer_->drawFrameMatches(currFrame_.getKeyPointsDistorted(),currIm_,vMatches_);

    //If not enough matches found, updtate reference frame
    if(nMatches < 30){
        refFrame_.assign(currFrame_);
        visualizer_->setReferenceFrame(refFrame_.getKeyPointsDistorted(),currIm_);

        return false;
    }

    //Try to initialize by finding an Essential matrix
    Sophus::SE3f Tcw;
    vector<Eigen::Vector3f> v3DPoints_w;
    vector<Eigen::Vector3f> v3DPoints_c;
    v3DPoints_w.reserve(vMatches_.capacity());
    v3DPoints_c.reserve(vMatches_.capacity());
    vector<bool> vTriangulated(vMatches_.capacity(),false);
    float parallax;
    if(!monoInitializer_.initialize(refFrame_, currFrame_, vMatches_, nMatches, Tcw, v3DPoints_w, vTriangulated, parallax)){
        return false;
    }
    
    Sophus::SE3f Tw = Sophus::SE3f();
    refFrame_.setPose(Tw);
    currFrame_.setPose(Tcw);

    refKeyFrame_ = std::make_shared<KeyFrame>(refFrame_);
    currKeyFrame_ = std::make_shared<KeyFrame>(currFrame_);
    
    pMap_->insertKeyFrame(refKeyFrame_);
    pMap_->insertKeyFrame(currKeyFrame_);

    Sophus::SE3f T1w = refKeyFrame_->getPose();
    Sophus::SE3f T2w = currKeyFrame_->getPose();

    int nTriangulated = 0;
    double scale1 = 0, scale2 = 0;
    float n_points = 0;

    for(int i = 0, j = 0; i < vTriangulated.size(); i++, j+=2){
        if(vTriangulated[i]){
            shared_ptr<MapPoint> pMP1(new MapPoint(v3DPoints_w[j]));
            shared_ptr<MapPoint> pMP2(new MapPoint(v3DPoints_w[j+1]));

            cv::Point2f x1 = refKeyFrame_->getKeyPoint(i).pt;
            cv::Point2f x2 = currKeyFrame_->getKeyPoint(vMatches_[i]).pt;

            double d1 = refKeyFrame_->getDepthMeasure(x1.x, x1.y);
            double d2 = currKeyFrame_->getDepthMeasure(x2.x, x2.y);

            if(d1 == 0.0 || d2 == 0.0)
            continue;
            
            pMap_->insertMapPoint(pMP1);
            pMap_->insertMapPoint(pMP2);

            pMap_->addObservation(refKeyFrame_->getId(), pMP1->getId(), i);
            pMap_->addObservation(currKeyFrame_->getId(), pMP2->getId(), vMatches_[i]);

            refKeyFrame_->setMapPoint(i, pMP1);
            currKeyFrame_->setMapPoint(i, pMP2);

            //scale
            double d1_up_to_scale = refKeyFrame_->getDepthMeasure(x1.x, x1.y, false);
            double d2_up_to_scale = currKeyFrame_->getDepthMeasure(x2.x, x2.y, false);
            v3DPoints_c[j] = T1w * v3DPoints_w[j];
            v3DPoints_c[j+1] = T2w * v3DPoints_w[j+1];

            scale1 += d1_up_to_scale / v3DPoints_c[j][2];
            scale2 += d2_up_to_scale / v3DPoints_c[j+1][2];

            nTriangulated += 2;
            n_points ++;
        }
    }

    scale1 = scale1 / n_points;
    scale2 = scale2 / n_points;

    refKeyFrame_->setEstimatedDepthScale(scale1);
    currKeyFrame_->setEstimatedDepthScale(scale2);

    cout << "Map initialized with " << nTriangulated << " MapPoints" << endl;

    Eigen::Vector3f t1 = T1w.inverse().translation();
    Eigen::Vector3f t2 = T2w.inverse().translation();
    float norm = (t2 - t1).norm();

    outFile_.open(filePath_);
    if (outFile_.is_open()) {
        outFile_ << "Translation norm between cameras (mm): " << norm * 1000 << '\n';
        outFile_ << "Parallax: " << parallax << '\n';
        outFile_ << "nMatches: " << nMatches << '\n';
        outFile_ << "nMapPoints: " << nTriangulated << '\n';

        outFile_.close();
    } else {
        std::cerr << "Unable to open file for writing" << std::endl;
    }

    visualizer_->drawFrameTriangulatedMatches(pMap_, currFrame_.getKeyPointsDistorted(),currIm_,vMatches_);

    return true;
}

void Mapping::triangulateSimulatedMapPoints() {
    int nTriangulated = 0;

    int nMatches = movedPoints_.size();

    std::cout << "Number of matches: " << nMatches << std::endl;

    Sophus::SE3f T1w = refKeyFrame_->getPose();
    Sophus::SE3f T2w = currKeyFrame_->getPose();

    std::shared_ptr<CameraModel> prevCalibration = refKeyFrame_->getCalibration();
    std::shared_ptr<CameraModel> currCalibration = currKeyFrame_->getCalibration();

    //Try to triangulate a new MapPoint with each match
    for(size_t i = 0; i < nMatches; i++){
        cv::Point2f x1 = refKeyFrame_->getKeyPoint(i).pt; // vMatches[i] si las parejas no fuesen ordenadas
        cv::Point2f x2 = currKeyFrame_->getKeyPoint(i).pt;

        Eigen::Vector3f xn1 = prevCalibration->unproject(x1).normalized();
        Eigen::Vector3f xn2 = currCalibration->unproject(x2).normalized();

        if(TrianMethod_ == "DepthMeasurement") {
            float d1 = refKeyFrame_->getDepthMeasure(i);
            float d2 = currKeyFrame_->getDepthMeasure(i);

            xn1 = prevCalibration->unproject(x1, d1);
            xn2 = currCalibration->unproject(x2, d2);
        }

        Eigen::Vector3f x3D_1, x3D_2;

        if (!useTriangulationMethod(xn1, xn2, T1w, T2w, x3D_1, x3D_2, TrianMethod_, TrianLocation_)) continue;

        if (!isValidParallax(xn1, xn2, T1w, T2w, x3D_1, x3D_2)) continue;

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

        pMap_->addObservation(refKeyFrame_->getId(), map_point_1->getId(), i);  // vMatches[i] si las parejas no fuesen ordenadas
        pMap_->addObservation(currKeyFrame_->getId(), map_point_2->getId(), i);

        refKeyFrame_->setMapPoint(i, map_point_1);
        currKeyFrame_->setMapPoint(i, map_point_2);

        nTriangulated++;
        nTriangulated++;
    }

    std::cout << "Triangulated " << nTriangulated << " MapPoints." << std::endl;

    refKeyFrame_->setInitialDepthScaleInSimulationImages();
    currKeyFrame_->setInitialDepthScaleInSimulationImages();
}

bool Mapping::isValidParallax(const Eigen::Vector3f& xn1, const Eigen::Vector3f& xn2, 
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

