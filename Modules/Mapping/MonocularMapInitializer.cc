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

#include "Utils/Geometry.h"
#include "Mapping/MonocularMapInitializer.h"
#include <cmath>

using namespace std;

MonocularMapInitializer::MonocularMapInitializer(){}

MonocularMapInitializer::MonocularMapInitializer(const int nFeatures, shared_ptr<CameraModel> calibration, const float fEpipolarTh, float fMinParallax,
                                                const std::string TrianMethod, const std::string TrianLocation, const bool checkingSelection, 
                                                const float depthLimit){
    //Reserve memory
    refKeys_.reserve(nFeatures);
    currKeys_.reserve(nFeatures);

    vMatches_.reserve(nFeatures);

    refRays_.resize(nFeatures,3);
    currRays_.resize(nFeatures,3);

    refKeysMatched_.resize(nFeatures);
    currKeysMatched_.resize(nFeatures);

    calibration_ = calibration;

    fEpipolarTh_ = fEpipolarTh;

    fMinParallax_ = fMinParallax;
    checkingSelection_ = checkingSelection;
    depthLimit_ = depthLimit;

    TrianMethod_ = TrianMethod;
    TrianLocation_ = TrianLocation;
}

bool MonocularMapInitializer::initialize(Frame refFrame, Frame currFrame, const std::vector<int>& vMatches, const int nMatches,
                                         std::vector<Eigen::Vector3f>& v3DPoints, std::vector<bool>& vTriangulated, float& parallax) {
    //Set up data
    refKeys_ = refFrame.getKeyPoints();
    currKeys_ = currFrame.getKeyPoints();
    vMatches_ = vMatches;

    prevCalibration_ = refFrame.getCalibration();
    currCalibration_ = currFrame.getCalibration();

    refFrame_ = std::make_shared<Frame>(refFrame);
    currFrame_ = std::make_shared<Frame>(currFrame);

    Sophus::SE3f T1w = refFrame.getPose();
    Sophus::SE3f T2w = currFrame.getPose();

    //Unproject matches points to its bearing rays
    std::vector<size_t> vRansacToFrameIndeces;
    vRansacToFrameIndeces.reserve(nMatches);
    size_t currMatIdx = 0;

    refKeysMatched_.clear();
    currKeysMatched_.clear();

    for(size_t i = 0; i < vMatches.size(); i++){
        if(vMatches_[i] != -1){
            refRays_.block(currMatIdx,0,1,3) = prevCalibration_->unproject(refKeys_[i].pt.x,refKeys_[i].pt.y).normalized();
            currRays_.block(currMatIdx,0,1,3) = currCalibration_->unproject(currKeys_[vMatches[i]].pt.x,currKeys_[vMatches[i]].pt.y).normalized();

            refKeysMatched_.push_back(refKeys_[i].pt);
            currKeysMatched_.push_back(currKeys_[vMatches[i]].pt);

            vRansacToFrameIndeces[currMatIdx] = i;

            currMatIdx++;
        }
    }

    std::fill(vTriangulated.begin(), vTriangulated.end(), false);

    if(checkingSelection_){
        vector<bool> vInliersOfE(currMatIdx,false);

        // If enough matches found, find an Essential matrix with RANSAC
        int nInliersOfE;
        Eigen::Matrix3f E = findEssentialWithRANSAC(nMatches,vInliersOfE, nInliersOfE);

        for(size_t i = 0; i < vInliersOfE.size(); i++){
            if(vInliersOfE[i]){
                vTriangulated[vRansacToFrameIndeces[i]] = true;
            }
        }
    } else {
        for(size_t i = 0; i < nMatches; i++){
            vTriangulated[vRansacToFrameIndeces[i]] = true;
        }
    }

    return reconstructPoints(T1w,T2w,v3DPoints,vTriangulated, parallax);

}

int MonocularMapInitializer::computeMaxTries(const float fInlierFraction, const float fSuccessLikelihood){
    return log(1 - fSuccessLikelihood) /
                 log(1 - pow(fInlierFraction,8));
}

Eigen::Matrix3f MonocularMapInitializer::findEssentialWithRANSAC(const int nMatches, vector<bool>& vInliers, int& nInliers) {
    int bestScore = 0;
    vector<bool> vBestInliers;
    vBestInliers.reserve(nMatches);
    Eigen::Matrix<float,3,3> bestE;

    srand(10);

    //Cluster data for random selection
    vector<int> vLabels;
    vector<cv::Point2f> vCenters;
    cv::kmeans(refKeysMatched_,8,vLabels,cv::TermCriteria( cv::TermCriteria::EPS, 10, 1.0),
               3, cv::KMEANS_PP_CENTERS, vCenters);

    vector<vector<size_t>> vvClusters = vector<vector<size_t>>(8);

    //Assign indices to clusters with the given labels
    for(size_t i = 0; i < vLabels.size(); i++){
        vvClusters[vLabels[i]].push_back(i);
    }

    int nMaxIters = computeMaxTries(0.8,0.95);
    int nCurrIt = 0;

    //Do all iterations
    while(nCurrIt < nMaxIters){
        nCurrIt++;

        //get minimum set of data
        Eigen::Matrix<float,8,3> refRays, currRays;
        for(int i = 0; i < 8; i++){
            random_shuffle(vvClusters[i].begin(),vvClusters[i].end());
            size_t idx = vvClusters[i][0];

            refRays.block<1,3>(i,0) = refRays_.block(idx,0,1,3);
            currRays.block<1,3>(i,0) = currRays_.block(idx,0,1,3);
        }

        //Compute model with the sample
        //Eigen::Matrix<float,3,3> E = computeE(refRays,currRays);

        Sophus::SE3f T1w = refFrame_->getPose();
        Sophus::SE3f T2w = currFrame_->getPose();
        Sophus::SE3f T12 = T1w*T2w.inverse();
        Eigen::Matrix<float,3,3> E = computeEssentialMatrixFromPose(T12);

        //Get score and inliers for the computed model
        int score = computeScoreAndInliers(nMatches,E,vInliers);
        if(score > bestScore){
            bestScore = score;
            vBestInliers = vInliers;
            bestE = E;
        }
    }

    vInliers = vBestInliers;
    nInliers = bestScore;

    return bestE;
}

Eigen::Matrix3f MonocularMapInitializer::computeE(Eigen::Matrix<float, 8, 3> &refRays,
                                                  Eigen::Matrix<float, 8, 3> &currRays) {
    Eigen::Matrix<float,8,9> A;
    for(size_t i = 0; i < 8; i++){
        A.block<1,3>(i,0) = refRays.row(i) * currRays(i,0);
        A.block<1,3>(i,3) = refRays.row(i) * currRays(i,1);
        A.block<1,3>(i,6) = refRays.row(i) * currRays(i,2);
    }

    Eigen::JacobiSVD<Eigen::Matrix<float,8,9>> svd(A,Eigen::ComputeFullV);
    svd.computeV();
    Eigen::Matrix<float,9,1> eigenVec = svd.matrixV().col(8);
    Eigen::Matrix3f E;
    E.row(0) = eigenVec.block<3,1>(0,0).transpose();
    E.row(1) = eigenVec.block<3,1>(3,0).transpose();
    E.row(2) = eigenVec.block<3,1>(6,0).transpose();

    Eigen::JacobiSVD<Eigen::Matrix<float,3,3>> svd_(E,Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Vector3f v;
    v << 1, 1, 0;
    Eigen::Matrix<float,3,3> Ef = svd_.matrixU()*v.asDiagonal()*svd_.matrixV().transpose();

    return -Ef;
}


int MonocularMapInitializer::computeScoreAndInliers(const int nMatched, Eigen::Matrix<float,3,3>& E,vector<bool>& vInliers) {
    Eigen::MatrixXf r1_hat = (E*refRays_.block(0,0,nMatched,3).transpose()).transpose().rowwise().normalized();

    auto err = (M_PI/2 - (r1_hat.array() *
                currRays_.block(0,0,nMatched,3).rowwise().normalized().array())
                .rowwise().sum().acos()).abs() < fEpipolarTh_;

    int score = 0;
    fill(vInliers.begin(),vInliers.end(),false);
    for(int i = 0; i < nMatched; i++){
        if(err(i)){
            vInliers[i] = true;
            score++;
        }
    }

    return score;
}

bool MonocularMapInitializer::reconstructEnvironment(Eigen::Matrix3f& E, Sophus::SE3f& T1w, Sophus::SE3f& T2w, std::vector<Eigen::Vector3f>& v3DPoints,
                                                     std::vector<bool>& vTriangulated, int& nInliers, float& parallax) {
    //Compute rays of the inliers points
    Eigen::MatrixXf refRays(nInliers,3), currRays(nInliers,3);
    size_t currMatIdx = 0;
    for(int i = 0; i < vTriangulated.size(); i++){
        if(vTriangulated[i]){
            refRays.row(currMatIdx) = calibration_->unproject(refKeys_[i].pt.x,refKeys_[i].pt.y).normalized();
            currRays.row(currMatIdx) = calibration_->unproject(currKeys_[vMatches_[i]].pt.x,currKeys_[vMatches_[i]].pt.y).normalized();

            currMatIdx++;
        }
    }

    //Reconstruct camera poses
    reconstructCameras(E,T2w,refRays,currRays);

    //Reconstruct 3D points (try with the 2 possible translations)
    return reconstructPoints(T1w,T2w,v3DPoints,vTriangulated, parallax);
}

void MonocularMapInitializer::reconstructCameras(Eigen::Matrix3f &E, Sophus::SE3f &T2w, Eigen::MatrixXf& rays1, Eigen::MatrixXf& rays2) {
    //Decompose E into R1, R2 and t
    Eigen::Matrix3f R1,R2,Rgood;
    Eigen::Vector3f t;
    decomposeE(E,R1,R2,t);

    //Choose the smallest rotation
    Rgood = (R2.trace() > R1.trace()) ? R2 : R1;

    //Get correct translation
    float away = ((Rgood*rays1.transpose()-rays2.transpose()).array() *
                 (rays2.transpose().colwise()-t).array()).colwise().sum().sign().sum();

    t = (signbit(away)) ? -t : t;

    T2w = Sophus::SE3f(Rgood,t);
}

void MonocularMapInitializer::decomposeE(Eigen::Matrix3f &E, Eigen::Matrix3f &R1, Eigen::Matrix3f &R2,
                                         Eigen::Vector3f &t) {
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(E,Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3f W;
    W << 0, -1, 0, 1, 0, 0, 0, 0, 1;

    R1 = svd.matrixU() * W.transpose() * svd.matrixV().transpose();
    if(R1.determinant() < 0)
        R1 = -R1;

    R2 = svd.matrixU() * W * svd.matrixV().transpose();
    if(R2.determinant() < 0)
        R2 = -R2;

    t = svd.matrixU().col(2).normalized();
}

bool MonocularMapInitializer::reconstructPoints(const Sophus::SE3f &T1w, const Sophus::SE3f &T2w, std::vector<Eigen::Vector3f> &v3DPoints,
                                                std::vector<bool>& vTriangulated, float& parallax) {
    vector<float> vParallax;
    int nTriangulated = 0;

    Eigen::Vector3f O2 = T1w.inverse().translation();
    Eigen::Vector3f O3 = T2w.inverse().translation();
    cout << "Tw1 Translation: " << O2[0]  << " " << O2[1]  << " "  << O2[2] << " " << endl;
    Eigen::Quaternionf qp = T1w.unit_quaternion();
    //std::cout << "T2w Quaternion: w = " << qp.w() << ", x = " << qp.x() << ", y = " << qp.y() << ", z = " << qp.z() << std::endl;
    cout << "Tw2 Translation: " << O3[0]  << " " << O3[1]  << " "  << O3[2] << " " << endl;
    Eigen::Quaternionf qc = T2w.unit_quaternion();
    //std::cout << "T1w Quaternion: w = " << qc.w() << ", x = " << qc.x() << ", y = " << qc.y() << ", z = " << qc.z() << std::endl;
    float norm = (O2 - O3).norm();
    std::cout << "Norm of translation between cameras: " << norm << std::endl;


    int  err1 = 0, err2 = 0, depth1 = 0, depth2 = 0, parallax_ = 0;

    std::cerr << "vTriangulated: "<< vTriangulated.size() << std::endl;
    for(size_t i = 0, j = 0; i < vTriangulated.size(); i++, j+=2){
        if(vTriangulated[i]){
            //Unproject KeyPoints to rays
            cv::Point2f x1 = refKeys_[i].pt;
            cv::Point2f x2 = currKeys_[vMatches_[i]].pt;
            Eigen::Vector3f r1 = prevCalibration_->unproject(x1).normalized();
            Eigen::Vector3f r2 = currCalibration_->unproject(x2).normalized();


            if(TrianMethod_ == "DepthMeasurement") {
                double d1 = refFrame_->getDepthMeasure(refKeys_[i].pt.x, refKeys_[i].pt.y);
                double d2 = currFrame_->getDepthMeasure(currKeys_[vMatches_[i]].pt.x, currKeys_[vMatches_[i]].pt.y);

                r1 = prevCalibration_->unproject(x1, d1);
                r2 = currCalibration_->unproject(x2, d2);
            }

            //Triangulate point
            Eigen::Vector3f p3D_1, p3D_2;
            if (!useTriangulationMethod(r1, r2, T1w, T2w, p3D_1, p3D_2, TrianMethod_, TrianLocation_)) continue;

            if (!p3D_1.allFinite() || !p3D_2.allFinite() || p3D_1.isZero() || p3D_2.isZero()) {
                vTriangulated[i] = false;
                continue;
            }
            
            //Check the parallax of the triangulated point
            Eigen::Vector3f p3D_c1 = T1w * p3D_1;
            Eigen::Vector3f p3D_c2 = T2w * p3D_2;
            auto ray1 = (T1w.inverse().rotationMatrix() * r1).normalized();
            auto ray2 = (T2w.inverse().rotationMatrix() * r2).normalized();
            float cosParallaxPoint = cosRayParallax(ray1, ray2);

            //Check that the point has been triangulated in front of the first camera (possitive depth)
            if(p3D_c1(2) < 0.0f || p3D_c1(2) > depthLimit_){
                vTriangulated[i] = false;
                depth1++;
                continue;
            }

            //Check Reprojection error
            cv::Point2f uv1 = prevCalibration_->project(p3D_c1);
            
            if(checkingSelection_){
                if(squaredReprojectionError(refKeys_[i].pt,uv1) > 5.991){
                    vTriangulated[i] = false;
                    err1++;
                    continue;
                }
            }

            if(p3D_c2(2) < 0.0f || p3D_c2(2) > depthLimit_){
                vTriangulated[i] = false;
                depth2++;
                continue;
            }

            cv::Point2f uv2 = currCalibration_->project(p3D_c2);

            if(checkingSelection_){
                if(squaredReprojectionError(currKeys_[vMatches_[i]].pt,uv2) > 5.991){
                    vTriangulated[i] = false;
                    err2++;
                    continue;
                }
            }

            v3DPoints[j] = p3D_1;
            v3DPoints[j+1] = p3D_2;

            nTriangulated += 2;
            vParallax.push_back(cosParallaxPoint);

        }
    }

    std::cerr << "nTriangulated: "<< nTriangulated << std::endl;
    
    if(nTriangulated == 0)
        return false;

    sort(vParallax.begin(),vParallax.end());
    size_t idx = min(50,int(vParallax.size()-1));
    float _parallax = vParallax[idx];

    if (_parallax < 0 || _parallax > 1) {
        std::cout << "Parallax must be between 0 and 1." << std::endl;
        return false;
    }
    float radians = acos(_parallax);
    float degrees = radians * (180.0 / M_PI);

    parallax = degrees;
    std::cerr << "_parallax: "<< degrees << std::endl;

    if(nTriangulated >= 25 && degrees > fMinParallax_) {
        return true;
    }
    else{
        return false;
    }
}
