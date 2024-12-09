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
#include "Optimization/g2oBundleAdjustment.h"
#include "Matching/DescriptorMatching.h"
#include "Utils/Geometry.h"
#include "Utils/CommonTypes.h"
#include <iterator>
#include <nlopt.hpp>
#include "Optimization/nloptOptimization.h"
#include "Optimization/EigenOptimization.h"

#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>

using namespace std;

LocalMapping::LocalMapping() {
}

LocalMapping::LocalMapping(Settings& settings, std::shared_ptr<FrameVisualizer>& visualizer,
                    std::shared_ptr<MapVisualizer>& mapVisualizer, std::shared_ptr<Map> pMap) {
    settings_ = settings;
    pMap_ = pMap;

    visualizer_ = visualizer;
    mapVisualizer_ = mapVisualizer;

    showScene_ = settings_.getShowScene();

    repBalanceWeight_ = settings_.getOptRepWeight();
    arapBalanceWeight_ = settings_.getOptArapWeight();
    globalBalanceWeight_ = settings_.getOptGlobalWeight();
    alphaWeight_ = settings_.getOptAlphaWeight();
    betaWeight_ = settings_.getOptBetaWeight();

    OptSelection_ = settings_.getOptSelection();
    OptWeightsSelection_ = settings_.getOptWeightsSelection();
    TrianMethod_ = settings_.getTrianMethod();
    TrianLocation_ = settings_.getTrianLocation();

    nOptimizations_ = settings_.getnOptimizations();
    nOptIterations_ = settings_.getnOptIterations();

    NloptnOptimizations_ = settings_.getNloptnOptimizations();
    NloptRelTolerance_ = settings_.getNloptRelTolerance();
    NloptAbsTolerance_ = settings_.getNloptAbsTolerance();
    NloptRepLowerBound_ = settings_.getNloptRepLowerBound();
    NloptRepUpperBound_ = settings_.getNloptRepUpperBound();
    NloptGlobalLowerBound_ = settings_.getNloptGlobalLowerBound();
    NloptGlobalUpperBound_ = settings_.getNloptGlobalUpperBound();
    NloptArapLowerBound_ = settings_.getNloptArapLowerBound();
    NloptArapUpperBound_ = settings_.getNloptArapUpperBound();

    filePath_ = "./Data/Experiment.txt";
    outFile_.imbue(std::locale("es_ES.UTF-8"));
}

void LocalMapping::doMapping(std::shared_ptr<KeyFrame> &pCurrKeyFrame, int &nMPs) {
    //Keep input keyframe
    currKeyFrame_ = pCurrKeyFrame;

    if(!currKeyFrame_)
        return;

    //Remove redundant MapPoints
    mapPointCulling();

    //Triangulate new MapPoints
    triangulateNewMapPoints();

    checkDuplicatedMapPoints();

    //Run deformation optimization
    optimization();

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
                Eigen::Vector3f x3D_1;
                Eigen::Vector3f x3D_2;

                if (TrianMethod_ == "Classic") {
                    triangulateClassic(xn1, xn2, T1w, T2w, x3D_1, x3D_2, TrianLocation_);
                } else if (TrianMethod_ == "ORBSLAM") {
                    triangulateORBSLAM(xn1, xn2, T1w, T2w, x3D_1, x3D_2, TrianLocation_);
                } else if (TrianMethod_ == "DepthMeasurement") {
                    float d1 = currKeyFrame_->getDepthMeasure(i);
                    float d2 = pKF->getDepthMeasure(i); // vMatches[i] si las parejas no fuesen ordenadas

                    xn1 = calibration1->unproject(x1, d1);
                    xn2 = calibration2->unproject(x2, d2);

                    triangulateDepth(xn1, xn2, T1w, T2w, x3D_1, x3D_2, TrianLocation_, d1, d2);
                } else {
                    triangulateNRSLAM(xn1, xn2, T1w, T2w, x3D_1, x3D_2, TrianLocation_);
                }

                //Check positive depth
                auto x_1 = T1w * x3D_1;
                auto x_2 = T2w * x3D_2;
                if(x_1[2] < 0.0 || x_2[2] < 0.0) continue;

                //Check parallax
                auto ray1 = (T1w.inverse().rotationMatrix() * xn1).normalized();
                auto ray2 = (T2w.inverse().rotationMatrix() * xn2).normalized();
                auto parallax = cosRayParallax(ray1, ray2);

                if(parallax > settings_.getMinCos()) continue;

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

void LocalMapping::optimization() {
    std::cout << "\nINITIAL MEASUREMENTS: \n";
    outFile_.open(filePath_);
    if (outFile_.is_open()) {
        outFile_ << "INITIAL MEASUREMENTS: \n";

        outFile_.close();
    } else {
        std::cerr << "Unable to open file for writing" << std::endl;
    }
    
    measureRelativeMapErrors(pMap_, filePath_);
    measureAbsoluteMapErrors(pMap_, originalPoints_, movedPoints_, filePath_);

    // stop
    // Uncomment for step by step execution (pressing esc key)
    // if (stop_) {
    //     std::cout << "Press esc to continue... " << std::endl;
    //     while((cv::waitKey(10) & 0xEFFFFF) != 27){
    //         mapVisualizer_->update(drawRaysSelection_);
    //     }
    // } else {
    //     if(showScene_) {
    //         mapVisualizer_->update(drawRaysSelection_);
    //     }
    // }

    double optimizationUpdate = 100;
    for(int i = 1; i <= nOptimizations_ && optimizationUpdate >= (0.00001*movedPoints_.size()); i++){ 
        // correct error
        if (OptSelection_ == "open3DArap") {
            arapOpen3DOptimization(pMap_.get());
        } else if(OptSelection_ == "twoOptimizations") {
            if (OptWeightsSelection_ == "nlopt") {
                nlopt::opt opt(nlopt::LN_NELDERMEAD, 3);
                
                std::vector<double> lb = {NloptRepLowerBound_, NloptGlobalLowerBound_, NloptArapLowerBound_};
                std::vector<double> ub = {NloptRepUpperBound_, NloptGlobalUpperBound_, NloptArapUpperBound_};
                opt.set_lower_bounds(lb);
                opt.set_upper_bounds(ub);

                OptimizationData optData;
                optData.pMap = pMap_->clone();
                optData.nOptIterations = nOptIterations_;
                optData.repErrorStanDesv = simulatedRepErrorStanDesv_;
                optData.alpha = alphaWeight_;
                optData.beta = betaWeight_;
                optData.depthUncertainty = SimulatedDepthErrorStanDesv_;

                opt.set_min_objective(outerObjective, &optData);

                std::vector<double> x = {repBalanceWeight_, globalBalanceWeight_, arapBalanceWeight_};

                opt.set_xtol_rel(NloptRelTolerance_);
                opt.set_xtol_abs(NloptAbsTolerance_);
                opt.set_maxeval(NloptnOptimizations_);

                double minf;
                nlopt::result result = opt.optimize(x, minf);

                std::cout << "\nWEIGHTS OPTIMIZED" << std::endl;
                std::cout << "Optimized repBalanceWeight: " << x[0] << std::endl;
                std::cout << "Optimized globalBalanceWeight: " << x[1] << std::endl;
                std::cout << "Optimized arapBalanceWeight: " << x[2] << std::endl;
                std::cout << "Final minimized ABSOLUTE error: " << minf << std::endl;

                std::cout << "\nFinal optimization with optimized weights:\n" << std::endl;

                arapOptimization(pMap_.get(), x[0], x[1], x[2], alphaWeight_, betaWeight_, SimulatedDepthErrorStanDesv_, 
                                    nOptIterations_, &optimizationUpdate);

                repBalanceWeight_ = x[0];
                globalBalanceWeight_ = x[1];
                arapBalanceWeight_ = x[2];
            } else {
                Eigen::VectorXd x(3);
                // Initial values
                x[0] = repBalanceWeight_;
                x[1] = globalBalanceWeight_;
                x[2] = arapBalanceWeight_;

                EigenOptimizationFunctor functor(pMap_->clone(), nOptIterations_, simulatedRepErrorStanDesv_, alphaWeight_, betaWeight_, 
                                                    SimulatedDepthErrorStanDesv_); 
                
                Eigen::NumericalDiff<EigenOptimizationFunctor> numDiff(functor);
                Eigen::LevenbergMarquardt<Eigen::NumericalDiff<EigenOptimizationFunctor>, double> levenbergMarquardt(numDiff);
                levenbergMarquardt.parameters.ftol = 1e-3;   // Loosen tolerance for function value change
                levenbergMarquardt.parameters.xtol = 1e-3;   // Loosen tolerance for parameter change
                levenbergMarquardt.parameters.gtol = 1e-3;   // Loosen tolerance for gradient
                levenbergMarquardt.parameters.maxfev = 10;  // Maintain maximum number of function evaluations
                //levenbergMarquardt.parameters.epsilon = 1e-5;


                int ret = levenbergMarquardt.minimize(x);
                std::cout << "Return code: " << ret << std::endl;

                std::cout << "Number of iterations: " << levenbergMarquardt.iter << std::endl;

                std::cout << "\nWEIGHTS OPTIMIZED" << std::endl;
                std::cout << "Optimized repBalanceWeight: " << x[0] << std::endl;
                std::cout << "Optimized globalBalanceWeight: " << x[1] << std::endl;
                std::cout << "Optimized arapBalanceWeight: " << x[2] << std::endl;

                std::cout << "\nFinal optimization with optimized weights:\n" << std::endl;

                arapOptimization(pMap_.get(), x[0], x[1], x[2], alphaWeight_, betaWeight_, SimulatedDepthErrorStanDesv_, 
                                    nOptIterations_, &optimizationUpdate);
            }
        } else {
            arapOptimization(pMap_.get(), repBalanceWeight_, globalBalanceWeight_, arapBalanceWeight_, alphaWeight_, betaWeight_,
                            SimulatedDepthErrorStanDesv_, nOptIterations_, &optimizationUpdate);
        }

        std::cout << "\nOptimization COMPLETED... " << i << " / " << nOptimizations_ << " iterations." << std::endl;
        std::cout << "\nOptimization change: " << optimizationUpdate << std::endl;
        
        if(showScene_) {
            mapVisualizer_->update(drawRaysSelection_);
        }

        if (i != nOptimizations_) {
            std::cout << i << " / " << nOptimizations_ << " MEASUREMENTS: \n";
            outFile_.open(filePath_, std::ios::app);
            if (outFile_.is_open()) {
                outFile_ << i << " / " << nOptimizations_ << " MEASUREMENTS: \n";

                outFile_.close();
            } else {
                std::cerr << "Unable to open file for writing" << std::endl;
            }

            measureRelativeMapErrors(pMap_, filePath_);
            measureAbsoluteMapErrors(pMap_, originalPoints_, movedPoints_, filePath_);
        }
    }

    //arapBundleAdjustment(pMap_.get());
    outFile_.open(filePath_, std::ios::app);
    if (outFile_.is_open()) {
        outFile_ << "FINAL MEASUREMENTS: \n";

        outFile_.close();
    } else {
        std::cerr << "Unable to open file for writing" << std::endl;
    }
}