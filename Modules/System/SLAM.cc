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

#include "SLAM.h"

#include "Map/KeyFrame.h"
#include "Map/MapPoint.h"
#include "Optimization/g2oBundleAdjustment.h"
#include "Utils/Geometry.h"
#include "Optimization/nloptOptimization.h"
#include "Optimization/EigenOptimization.h"

#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>

#include <nlopt.hpp>

#include <memory>
#include "Utils/CommonTypes.h"

SLAM::SLAM(const std::string &settingsFile) {
    //Load settings from file
    cout << "Loading system settings from: " << settingsFile << endl;
    settings_ = Settings(settingsFile);
    cout << settings_ << endl;

    //Create map
    pMap_ = shared_ptr<Map>(new Map(settings_.getMinCommonObs()));

    //Create visualizers
    mapVisualizer_ = shared_ptr<MapVisualizer>(new MapVisualizer(pMap_));

    prevFrame_ = Frame(settings_.getFeaturesPerImage(),settings_.getGridCols(),settings_.getGridRows(),
                       settings_.getImCols(),settings_.getImRows(),settings_.getNumberOfScales(), settings_.getScaleFactor(),
                       settings_.getCalibration(),settings_.getDistortionParameters());

    currFrame_ = Frame(settings_.getFeaturesPerImage(),settings_.getGridCols(),settings_.getGridRows(),
                       settings_.getImCols(),settings_.getImRows(), settings_.getNumberOfScales(), settings_.getScaleFactor(),
                       settings_.getCalibration(),settings_.getDistortionParameters());

    prevCalibration_ = prevFrame_.getCalibration();
    currCalibration_ = currFrame_.getCalibration();

    currIm_ = cv::Mat::zeros(settings_.getImRows(), settings_.getImCols(), CV_8UC3);
    currIm_.setTo(cv::Scalar(255, 255, 255));

    C1Pose_ = settings_.getFirstCameraPos();
    C2Pose_ = settings_.getSecondCameraPos();

    simulatedRepErrorStanDesv_ = settings_.getSimulatedRepError();

    arapBalanceWeight_ = settings_.getOptArapWeight();
    reprojectionBalanceWeight_ = settings_.getOptReprojectionWeight();

    OptSelection_ = settings_.getOptSelection();
    OptWeightsSelection_ = settings_.getOptWeightsSelection();
    TrianSelection_ = settings_.getTrianSelection();

    nOptimizations_ = settings_.getnOptimizations();
    nOptIterations_ = settings_.getnOptIterations();

    NloptnOptimizations_ = settings_.getNloptnOptimizations();
    NloptRelTolerance_ = settings_.getNloptRelTolerance();
    NloptAbsTolerance_ = settings_.getNloptAbsTolerance();
    NloptRepLowerBound_ = settings_.getNloptRepLowerBound();
    NloptRepUpperBound_ = settings_.getNloptRepUpperBound();
    NloptArapLowerBound_ = settings_.getNloptArapLowerBound();
    NloptArapUpperBound_ = settings_.getNloptArapUpperBound();

    drawRaysSelection_ = settings_.getDrawRaysSelection();
    showSolution_ = settings_.getShowSolution();
}


void SLAM::loadPoints(const std::string &originalFile, const std::string &movedFile) {
    std::ifstream originalFileStream(originalFile);
    if (!originalFileStream.is_open()) {
        std::cerr << "Error opening original points file: " << originalFile << std::endl;
        return;
    }

    std::ifstream movedFileStream(movedFile);
    if (!movedFileStream.is_open()) {
        std::cerr << "Error opening moved points file: " << movedFile << std::endl;
        return;
    }

    originalPoints_.clear();  // Clear previous points if any
    movedPoints_.clear();     // Clear previous points if any

    std::string line;
    while (std::getline(originalFileStream, line)) {
        std::istringstream iss(line);
        float x, y, z;
        if (!(iss >> x >> y >> z)) {
            std::cerr << "Error reading original points file format" << std::endl;
            continue;
        }
        Eigen::Vector3f originalPoint(x, y, z);
        originalPoints_.push_back(originalPoint);

        // Read corresponding moved point
        if (!std::getline(movedFileStream, line)) {
            std::cerr << "Error reading moved points file format" << std::endl;
            continue;
        }
        std::istringstream issMoved(line);
        float xMoved, yMoved, zMoved;
        if (!(issMoved >> xMoved >> yMoved >> zMoved)) {
            std::cerr << "Error reading moved points file format" << std::endl;
            continue;
        }
        Eigen::Vector3f movedPoint(xMoved, yMoved, zMoved);
        movedPoints_.push_back(movedPoint);
    }

    // Close files
    originalFileStream.close();
    movedFileStream.close();

    std::cout << "Loaded " << originalPoints_.size() << " MapPoints from files." << std::endl;
}

void SLAM::setCameraPoses(const Eigen::Vector3f firstCamera, const Eigen::Vector3f secondCamera) {
    // set camera poses and orientation
    Eigen::Matrix3f R = Eigen::Matrix3f::Identity();
    Sophus::SE3f T1w(R, firstCamera);
    prevFrame_.setPose(T1w);

    R = lookAt(secondCamera, movedPoints_[0]);
    Sophus::SE3f T2w(R, secondCamera);
    currFrame_.setPose(T2w);
    Tcw_ = T2w;

    mapVisualizer_->updateCurrentPose(T2w);
}

void SLAM::viusualizeSolution() {
    size_t nTriangulated = 0;
    size_t nMatches = movedPoints_.size();

    std::cout << "Number of matches: " << nMatches << std::endl;

    Sophus::SE3f T1w = prevFrame_.getPose();
    Sophus::SE3f T2w = currFrame_.getPose();

    for(size_t i = 0; i < nMatches; i++){
        Eigen::Vector3f x3D_1 = originalPoints_[i];
        Eigen::Vector3f x3D_2 = movedPoints_[i];

        std::shared_ptr<MapPoint> map_point_1(new MapPoint(x3D_1));
        std::shared_ptr<MapPoint> map_point_2(new MapPoint(x3D_2));

        pMap_->insertMapPoint(map_point_1);
        pMap_->insertMapPoint(map_point_2);

        insertedIndexes_.push_back(i);

        pMap_->addObservation(prevKeyFrame_->getId(), map_point_1->getId(), i);  // vMatches[i] si las parejas no fuesen ordenadas
        pMap_->addObservation(currKeyFrame_->getId(), map_point_2->getId(), i);

        prevKeyFrame_->setMapPoint(i, map_point_1); // vMatches[i]?
        currKeyFrame_->setMapPoint(i, map_point_2);

        nTriangulated++;
        nTriangulated++;
    }

    std::cout << "Triangulated " << nTriangulated << " MapPoints." << std::endl;

    // stop
    //Uncomment for step by step execution (pressing esc key)
    cv::namedWindow("Test Window");

    // visualize
    mapVisualizer_->update(drawRaysSelection_);
    mapVisualizer_->updateCurrentPose(Tcw_);
}

void SLAM::createKeyPoints() {
    std::default_random_engine generator;
    std::normal_distribution<float> distribution(0.0f, simulatedRepErrorStanDesv_);

    Sophus::SE3f T1w = prevFrame_.getPose();
    Sophus::SE3f T2w = currFrame_.getPose();

    for(size_t i = 0; i < movedPoints_.size(); ++i) {
        cv::Point2f original_p2D;
        cv::Point2f moved_p2D;

        Eigen::Vector3f p3Dcam1 = T1w * originalPoints_[i];
        Eigen::Vector3f p3Dcam2 = T2w * movedPoints_[i];

        original_p2D = prevCalibration_->project(p3Dcam1);
        moved_p2D = currCalibration_->project(p3Dcam2);

        // Add Gaussian reprojection noise in units of pixels
        original_p2D.x = std::round(original_p2D.x + distribution(generator));
        original_p2D.y = std::round(original_p2D.y + distribution(generator));
        moved_p2D.x = std::round(moved_p2D.x + distribution(generator));
        moved_p2D.y = std::round(moved_p2D.y + distribution(generator));

        cv::KeyPoint original_keypoint(original_p2D, 1.0f); // 1.0f is the size of the keypoint
        cv::KeyPoint moved_keypoint(moved_p2D, 1.0f);

        prevFrame_.setKeyPoint(original_keypoint, i);
        currFrame_.setKeyPoint(moved_keypoint, i);
    }

    // Promote to keyframes for visualization
    prevKeyFrame_ = std::make_shared<KeyFrame>(prevFrame_);
    currKeyFrame_ = std::make_shared<KeyFrame>(currFrame_);
    pMap_->insertKeyFrame(prevKeyFrame_);
    pMap_->insertKeyFrame(currKeyFrame_);

    visualizer_->drawFeatures(prevFrame_.getKeyPoints(), currIm_, "Previous Frame KeyPoints");
    visualizer_->drawFeatures(currFrame_.getKeyPoints(), currIm_, "Current Frame KeyPoints");

}

void SLAM::mapping() {
    int nTriangulated = 0;

    // Each moved point correspond with the point in the same index in the original vector
    // vector<int> vMatches(currKeyFrame_->getMapPoints().size()); // if we had to search for matches
    // vector<int> vMatches(currFrame_.getKeyPoints().size());
    int nMatches = movedPoints_.size();

    std::cout << "Number of matches: " << nMatches << std::endl;

    Sophus::SE3f T1w = prevFrame_.getPose();
    Sophus::SE3f T2w = currFrame_.getPose();

    //Try to triangulate a new MapPoint with each match
    for(size_t i = 0; i < nMatches; i++){ //vMatches.size()
        // if(vMatches[i] != -1){
        auto x1 = prevKeyFrame_->getKeyPoint(i).pt; // vMatches[i] si las parejas no fuesen ordenadas
        auto x2 = currKeyFrame_->getKeyPoint(i).pt;

        Eigen::Vector3f xn1 = prevCalibration_->unproject(x1).normalized();
        Eigen::Vector3f xn2 = currCalibration_->unproject(x2).normalized();

        Eigen::Vector3f x3D_1;
        Eigen::Vector3f x3D_2;
        Eigen::Vector3f x3D_prev;

        x3D_prev = originalPoints_[i];

        if (TrianSelection_ == "TwoPoints") {
            triangulateTwoPoints(xn1, xn2, T1w, T2w, x3D_1, x3D_2);
        } else if (TrianSelection_ == "InRaysNearPrevSolution") {
            triangulateInRaysNearPrevSolution(xn1, xn2, T1w, T2w, x3D_1, x3D_2, x3D_prev);
        } else {
            triangulateInRays(xn1, xn2, T1w, T2w, x3D_1, x3D_2);
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
        prevCalibration_->project(T1w*x3D_1, p_p1);
        currCalibration_->project(T2w*x3D_2, p_p2);

        cv::Point2f cv_p1(p_p1[0], p_p1[1]);
        cv::Point2f cv_p2(p_p2[0], p_p2[1]);

        auto e1 = squaredReprojectionError(x1, cv_p1);
        auto e2 = squaredReprojectionError(x2, cv_p2);

        //if(e1 > 5.991 || e2 > 5.991) continue;

        std::shared_ptr<MapPoint> map_point_1(new MapPoint(x3D_1));
        std::shared_ptr<MapPoint> map_point_2(new MapPoint(x3D_2));

        pMap_->insertMapPoint(map_point_1);
        pMap_->insertMapPoint(map_point_2);

        // Save the index "i" of the original/moved match
        insertedIndexes_.push_back(i);

        pMap_->addObservation(prevKeyFrame_->getId(), map_point_1->getId(), i);  // vMatches[i] si las parejas no fuesen ordenadas
        pMap_->addObservation(currKeyFrame_->getId(), map_point_2->getId(), i);

        prevKeyFrame_->setMapPoint(i, map_point_1); // vMatches[i]?
        currKeyFrame_->setMapPoint(i, map_point_2);

        nTriangulated++;
        nTriangulated++;
        // }
    }

    std::cout << "Triangulated " << nTriangulated << " MapPoints." << std::endl;

    // stop
    //Uncomment for step by step execution (pressing esc key)
    cv::namedWindow("Test Window");
    std::cout << "Press esc to continue... " << std::endl;
    while((cv::waitKey(10) & 0xEFFFFF) != 27){
        mapVisualizer_->update(drawRaysSelection_);
    }

    std::cout << "\nINITIAL MEASUREMENTS: \n";
    measureRelativeErrors();
    measureAbsoluteErrors();

    // correct error
    if (OptSelection_ == "open3DArap") {
        arapOpen3DOptimization(pMap_.get());
    } else if(OptSelection_ == "twoOptimizations") {
        if (OptWeightsSelection_ == "nlopt") {
            nlopt::opt opt(nlopt::LN_NELDERMEAD, 2);
            
            std::vector<double> lb = {NloptRepLowerBound_, NloptArapLowerBound_};
            std::vector<double> ub = {NloptRepUpperBound_, NloptArapUpperBound_};
            opt.set_lower_bounds(lb);
            opt.set_upper_bounds(ub);

            OptimizationData optData;
            optData.pMap = pMap_->clone();
            optData.nOptIterations = nOptIterations_;
            optData.repErrorStanDesv = simulatedRepErrorStanDesv_;

            opt.set_min_objective(outerObjective, &optData);

            std::vector<double> x = {reprojectionBalanceWeight_, arapBalanceWeight_};

            opt.set_xtol_rel(NloptRelTolerance_);
            opt.set_xtol_abs(NloptAbsTolerance_);
            opt.set_maxeval(NloptnOptimizations_);

            double minf;
            nlopt::result result = opt.optimize(x, minf);

            std::cout << "\nWEIGHTS OPTIMIZED" << std::endl;
            std::cout << "Optimized repBalanceWeight: " << x[0] << std::endl;
            std::cout << "Optimized arapBalanceWeight: " << x[1] << std::endl;
            std::cout << "Final minimized ABSOLUTE error: " << minf << std::endl;

            std::cout << "\nFinal optimization with optimized weights:\n" << std::endl;

            arapOptimization(pMap_.get(), x[0], x[1], nOptIterations_);
        } else {
            Eigen::VectorXd x(2);
            // Initial values
            x[0] = reprojectionBalanceWeight_;
            x[1] = arapBalanceWeight_;

            EigenOptimizationFunctor functor(pMap_->clone(), nOptIterations_, simulatedRepErrorStanDesv_); 
            
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
            std::cout << "Optimized arapBalanceWeight: " << x[1] << std::endl;

            std::cout << "\nFinal optimization with optimized weights:\n" << std::endl;

            arapOptimization(pMap_.get(), x[0], x[1], nOptIterations_);
        }
    } else {
        arapOptimization(pMap_.get(), reprojectionBalanceWeight_, arapBalanceWeight_, nOptIterations_);
    }

    //arapBundleAdjustment(pMap_.get());

    std::cout << "\nBundle adjustment completed... fisrt " << nOptIterations_ << " iterations." << std::endl;

    if (nOptimizations_ > 1) {
        for(int i = 1; i < nOptimizations_; i++){ 
            measureRelativeErrors();
            measureAbsoluteErrors();

            mapVisualizer_->update(drawRaysSelection_);
            mapVisualizer_->updateCurrentPose(Tcw_);

            arapOptimization(pMap_.get(), reprojectionBalanceWeight_, arapBalanceWeight_, nOptIterations_);
            
            std::cout << "\nBundle adjustment completed... other " << nOptIterations_ << " iterations: (" << i + 1 << " time)" << std::endl;
        }
    }

    // visualize
    mapVisualizer_->update(drawRaysSelection_);
    mapVisualizer_->updateCurrentPose(Tcw_);
}

void SLAM::measureAbsoluteErrors() {

    Sophus::SE3f T1w = prevFrame_.getPose();
    Sophus::SE3f T2w = currFrame_.getPose();

    Sophus::SE3f T1w_corrected = pMap_->getKeyFrame(0)->getPose();
    Sophus::SE3f T2w_corrected = pMap_->getKeyFrame(1)->getPose();

    // Show position and orientation errors in pose 1
    Eigen::Vector3f position_error1 = T1w_corrected.translation() - T1w.translation();
    Eigen::Vector3f orientation_error1 = (T1w_corrected.so3().inverse() * T1w.so3()).log();

    // std::cout << "\nError in position 1:\n";
    // std::cout << "x: " << position_error1.x() << " y: " << position_error1.y() << " z: " << position_error1.z() << std::endl;
    // std::cout << "Error in orientation 1:\n";
    // std::cout << "x: " << orientation_error1.x() << " y: " << orientation_error1.y() << " z: " << orientation_error1.z() << std::endl;

    // Show position and orientation errors in pose 2
    Eigen::Vector3f position_error2 = T2w_corrected.translation() - T2w.translation();
    Eigen::Vector3f orientation_error2 = (T2w_corrected.so3().inverse() * T2w.so3()).log();

    // std::cout << "\nError in position 2:\n";
    // std::cout << "x: " << position_error2.x() << " y: " << position_error2.y() << " z: " << position_error2.z() << std::endl;
    // std::cout << "Error in orientation 2:\n";
    // std::cout << "x: " << orientation_error2.x() << " y: " << orientation_error2.y() << " z: " << orientation_error2.z() << std::endl;

    // 3D Error measurement in map points
    std::unordered_map<ID, std::shared_ptr<MapPoint>> mapPoints_corrected = pMap_->getMapPoints();

    float total_movement = 0.0f;
    float total_error_original = 0.0f;
    float total_error_moved = 0.0f;
    float total_error = 0.0f;
    int point_count = insertedIndexes_.size()*2;

    for(size_t i = 0, j = 0; j < insertedIndexes_.size(); i += 2, j++) {
        std::shared_ptr<MapPoint> mapPoint1 = pMap_->getMapPoint(i);
        std::shared_ptr<MapPoint> mapPoint2 = pMap_->getMapPoint(i+1);
        Eigen::Vector3f opt_original_position = mapPoint1->getWorldPosition();
        Eigen::Vector3f opt_moved_position = mapPoint2->getWorldPosition();

        Eigen::Vector3f original_position = originalPoints_[insertedIndexes_[j]];
        Eigen::Vector3f moved_position = movedPoints_[insertedIndexes_[j]];

        Eigen::Vector3f movement = original_position - moved_position;
        Eigen::Vector3f original_error = opt_original_position - original_position;
        Eigen::Vector3f moved_error = opt_moved_position - moved_position;
        float error_magnitude_movement = movement.norm();
        float error_magnitude_original = original_error.norm();
        float error_magnitude_moved = moved_error.norm();
        total_movement += error_magnitude_movement;
        total_error_original += error_magnitude_original;
        total_error_moved += error_magnitude_moved;
        total_error += error_magnitude_moved + error_magnitude_original;

        // std::cout << "\nError for point: " << mapPoint1->getId() << " and " << mapPoint2->getId() << "\n";
        // std::cout << "Position " << insertedIndexes_[j] << "\n";
        // std::cout << "Mappoint x: " << opt_original_position.x() << " y: " << opt_original_position.y() << " z: " << opt_original_position.z() << std::endl;
        // std::cout << "point x: " << original_position.x() << " y: " << original_position.y() << " z: " << original_position.z() << std::endl;
        // std::cout << "moved Mappoint x: " << opt_moved_position.x() << " y: " << opt_moved_position.y() << " z: " << opt_moved_position.z() << std::endl;
        // std::cout << "moved point x: " << moved_position.x() << " y: " << moved_position.y() << " z: " << moved_position.z() << std::endl;
        // std::cout << "x: " << total_error << std::endl;
    }

    // std::cout << "point_count: " << point_count << std::endl;
    if (point_count > 0) {
        std::cout << "\nABSOLUTE MEASUREMENTS: \n";

        float average_movement = total_movement / insertedIndexes_.size();
        //std::cout << "\nTotal movement: " << total_movement << std::endl;
        std::cout << "Average movement: " << average_movement << std::endl;
        float average_error_original = total_error_original / insertedIndexes_.size();
        //std::cout << "\nTotal error in ORIGINAL 3D: " << total_error_original << std::endl;
        //std::cout << "Average error in ORIGINAL 3D: " << average_error_original << std::endl;
        float average_error_moved = total_error_moved / insertedIndexes_.size();
        //std::cout << "\nTotal error in MOVED 3D: " << total_error_moved << std::endl;
        //std::cout << "Average error in MOVED 3D: " << average_error_moved << std::endl;
        float average_error = total_error / point_count;
        //std::cout << "\nTotal error in 3D: " << total_error << std::endl;
        std::cout << "Average error in 3D: " << average_error << "\n" << std::endl;
    } else {
        std::cout << "No points to compare." << std::endl;
    }

    // stop
    // Uncomment for step by step execution (pressing esc key)
    std::cout << "Press esc to continue... " << std::endl;
    while((cv::waitKey(10) & 0xEFFFFF) != 27){
        mapVisualizer_->update(drawRaysSelection_);
    }
}

void SLAM::measureRelativeErrors(){
    std::vector<Eigen::Vector3d> relativeErrors;
    std::vector<double> squaredNormRelativeErrors;
    Eigen::Vector3d meanRelativeError = Eigen::Vector3d::Zero();
    double meanSquaredNormRelativeError = 0;

    Eigen::Vector2d meanRepErrorUVC1 = Eigen::Vector2d::Zero();
    double meanRepErrorC1 = 0;
    double desvRepErrorC1 = 0;
    Eigen::Vector2d meanRepErrorUVC2 = Eigen::Vector2d::Zero();
    double meanRepErrorC2 = 0; 
    double desvRepErrorC2 = 0;
    Eigen::Vector2d meanRepErrorUV = Eigen::Vector2d::Zero();
    double meanRepError = 0;
    double desvRepError = 0;
    
    size_t nMatches = 0;
    size_t validPairs = 0;

    std::unordered_map<ID,KeyFrame_>&  mKeyFrames = pMap_->getKeyFrames();

    std::cout << "\nKEYFRAMES k AND k+1 MEASUREMENTS: \n";
    for (auto k1 = mKeyFrames.begin(); k1 != mKeyFrames.end(); ++k1) {
        for (auto k2 = std::next(k1); k2 != mKeyFrames.end(); ++k2) {
            KeyFrame_ pKF1 = k2->second;
            KeyFrame_ pKF2 = k1->second;
            std::cout << "Pair: (" << k1->first << ", " << k2->first<< ")\n";

            Eigen::Matrix3d Rs_global = Eigen::Matrix3d::Identity();
            Eigen::Vector3d Ts = Eigen::Vector3d::Zero();
            std::pair<Eigen::Matrix3d, Eigen::Vector3d> transformation = pMap_->getGlobalKeyFramesTransformation(k2->first, k1->first);

            Rs_global = transformation.first;
            Ts = transformation.second;

            std::cout << "Global rotation: " << Rs_global<< "\n";
            std::cout << "Global translation: " << Ts<< "\n";

            vector<MapPoint_>& v1MPs = pKF1->getMapPoints();
            vector<MapPoint_>& v2MPs = pKF2->getMapPoints();
            
            // MESH CREATION
            std::vector<Eigen::Vector3d> v1Positions = extractPositions(v1MPs);
            std::vector<Eigen::Vector3d> v2Positions = extractPositions(v2MPs);

            std::shared_ptr<open3d::geometry::TriangleMesh> mesh;

            std::vector<Eigen::Matrix3d> Rs(v1Positions.size(), Eigen::Matrix3d::Identity());

            // Perform Delaunay Reconstruction
            mesh = ComputeDelaunayTriangulation3D(v1Positions);
            mesh->ComputeAdjacencyList();

            auto posIndexes = createVectorMap(mesh->vertices_, v1Positions);

            std::map<size_t, size_t> invertedPosIndexes;
            for (const auto& pair : posIndexes) {
                invertedPosIndexes[pair.second] = pair.first;
            } 
            
            std::shared_ptr<CameraModel> pCamera1 = pKF1->getCalibration();
            g2o::SE3Quat camera1Pose = g2o::SE3Quat(pKF1->getPose().unit_quaternion().cast<double>(),pKF1->getPose().translation().cast<double>());
            std::shared_ptr<CameraModel> pCamera2 = pKF2->getCalibration();
            g2o::SE3Quat camera2Pose = g2o::SE3Quat(pKF2->getPose().unit_quaternion().cast<double>(),pKF2->getPose().translation().cast<double>());

            for (size_t i = 0; i < v1MPs.size(); i++) {
                MapPoint_ pMPi1 = v1MPs[i];
                MapPoint_ pMPi2 = v2MPs[i];
                if (!pMPi1) continue;
                if (!pMPi2) continue;


                //Reprojection error
                //C1
                cv::Point2f uv = pKF1->getKeyPoint(i).pt;
                Eigen::Vector2d obs;
                obs << uv.x, uv.y;  //Observed point in the image
                   
                Eigen::Vector3d p3Dw = pMPi1->getWorldPosition().cast<double>();    //Predicted 3D world position  of the point

                Eigen::Vector3d p3Dc = camera1Pose.map(p3Dw);
                Eigen::Vector2f projected;
                pCamera1->project(p3Dc.cast<float>(), projected);

                Eigen::Vector2d pixelsError = (obs - projected.cast<double>());
                meanRepErrorUVC1 += pixelsError.cwiseAbs();;
                meanRepErrorUV += pixelsError.cwiseAbs();;

                //C2
                uv = pKF2->getKeyPoint(i).pt;
                obs << uv.x, uv.y;  //Observed point in the image
                   
                p3Dw = pMPi2->getWorldPosition().cast<double>();    //Predicted 3D world position  of the point

                p3Dc = camera2Pose.map(p3Dw);
                projected;
                pCamera2->project(p3Dc.cast<float>(), projected);

                pixelsError = (obs - projected.cast<double>());    
                meanRepErrorUVC2 += pixelsError.cwiseAbs();
                meanRepErrorUV += pixelsError.cwiseAbs();


                //ARAP error
                auto it = invertedPosIndexes.find(i);
                size_t meshIndex = 0;
                if (it != invertedPosIndexes.end()) {
                    meshIndex = it->second;
                } else {
                    continue;
                }

                if (mesh->adjacency_list_[meshIndex].empty()) continue;

                std::unordered_set<int> jIndexes = mesh->adjacency_list_[meshIndex];

                for (int j : jIndexes) {
                    Eigen::Vector3d pi1 = v1Positions[i];
                    Eigen::Vector3d pj1 = v1Positions[posIndexes[j]];
                    Eigen::Vector3d pi2 = v2Positions[i];
                    Eigen::Vector3d pj2 = v2Positions[posIndexes[j]];

                    if (!pi1.allFinite() || !pj1.allFinite() || !pi2.allFinite() || !pj2.allFinite()) continue;

                    Eigen::Vector3d d1 = pi1 - pj1;
                    Eigen::Vector3d d2 = pi2 - pj2;
                    Eigen::Vector3d diff = d2 - d1;
                    double squaredNorm = diff.squaredNorm();
                      
                    meanRelativeError += diff;
                    meanSquaredNormRelativeError += squaredNorm;
                    validPairs++;
                }
                nMatches++;
            }

            if (validPairs > 1) {
                meanRelativeError /= static_cast<double>(validPairs);
                meanSquaredNormRelativeError /= static_cast<double>(validPairs);

                meanRepErrorUVC1 /= static_cast<double>(nMatches);
                meanRepErrorUVC2 /= static_cast<double>(nMatches);
                meanRepErrorUV /= static_cast<double>(2*nMatches);

                meanRepErrorC1 = (meanRepErrorUVC1[0] + meanRepErrorUVC1[1]) / 2.0;
                meanRepErrorC2 = (meanRepErrorUVC2[0] + meanRepErrorUVC2[1]) / 2.0;
                meanRepError = (meanRepErrorUV[0] + meanRepErrorUV[1]) / 2.0;
                
                //std::cout << "\nTotal movement: " << total_movement << std::endl;
                std::cout << std::fixed << std::setprecision(10);
                //std::cout << "Average relative error: " << meanRelativeError[0] << ", " << meanRelativeError[1] << ", " << meanRelativeError[2] << std::endl;

                // calculate the sum of squared differences from the mean for standard deviation
                Eigen::Vector2d sumSquaredDifferencesUVC1 = Eigen::Vector2d::Zero();
                Eigen::Vector2d sumSquaredDifferencesUVC2 = Eigen::Vector2d::Zero();
                Eigen::Vector2d sumSquaredDifferencesUV = Eigen::Vector2d::Zero();

                for (size_t i = 0; i < v1MPs.size(); i++) {
                    MapPoint_ pMPi1 = v1MPs[i];
                    MapPoint_ pMPi2 = v2MPs[i];
                    if (!pMPi1 || !pMPi2) continue;

                    // Reprojection error for C1
                    cv::Point2f uv = pKF1->getKeyPoint(i).pt;
                    Eigen::Vector2d obs;
                    obs << uv.x, uv.y;

                    Eigen::Vector3d p3Dw = pMPi1->getWorldPosition().cast<double>();
                    Eigen::Vector3d p3Dc = camera1Pose.map(p3Dw);
                    Eigen::Vector2f projected;
                    pCamera1->project(p3Dc.cast<float>(), projected);

                    Eigen::Vector2d pixelsError = (obs - projected.cast<double>()).cwiseAbs();
                    sumSquaredDifferencesUVC1 += (pixelsError - meanRepErrorUVC1).cwiseAbs2();
                    sumSquaredDifferencesUV += (pixelsError - meanRepErrorUV).cwiseAbs2();

                    // Reprojection error for C2
                    uv = pKF2->getKeyPoint(i).pt;
                    obs << uv.x, uv.y;

                    p3Dw = pMPi2->getWorldPosition().cast<double>();
                    p3Dc = camera2Pose.map(p3Dw);
                    pCamera2->project(p3Dc.cast<float>(), projected);

                    pixelsError = (obs - projected.cast<double>()).cwiseAbs();
                    sumSquaredDifferencesUVC2 += (pixelsError - meanRepErrorUVC2).cwiseAbs2();
                    sumSquaredDifferencesUV += (pixelsError - meanRepErrorUV).cwiseAbs2();
                }

                // Variance calculation
                Eigen::Vector2d varianceUVC1 = sumSquaredDifferencesUVC1 / static_cast<double>(nMatches);
                Eigen::Vector2d varianceUVC2 = sumSquaredDifferencesUVC2 / static_cast<double>(nMatches);
                Eigen::Vector2d varianceUV = sumSquaredDifferencesUV / static_cast<double>(2 * nMatches);

                // Standard deviation (sqrt of variance)
                Eigen::Vector2d stdDevUVC1 = varianceUVC1.cwiseSqrt();
                Eigen::Vector2d stdDevUVC2 = varianceUVC2.cwiseSqrt();
                Eigen::Vector2d stdDevUV = varianceUV.cwiseSqrt();

                desvRepErrorC1 = (stdDevUVC1[0] + stdDevUVC1[1]) / 2.0;
                desvRepErrorC2 = (stdDevUVC2[0] + stdDevUVC2[1]) / 2.0;
                desvRepError = (stdDevUV[0] + stdDevUV[1]) / 2.0;

                std::cout << "Pixels C1 error (average): " << meanRepErrorC1 << std::endl;
                std::cout << "Pixels C1 error (standard desv): " << desvRepErrorC1 << std::endl;
                std::cout << "Pixels C2 error (average): " << meanRepErrorC2 << std::endl;
                std::cout << "Pixels C2 error (standard desv): " << desvRepErrorC2 << std::endl;
                std::cout << "Pixels error (average): " << meanRepError << std::endl;
                std::cout << "Pixels error (standard desv): " << desvRepError << std::endl;
                
                std::cout << "Average squared norm relative error: " << meanSquaredNormRelativeError << std::endl;
            } else {
                std::cout << "No points to compare." << std::endl;
            }
        }
    }
}

Eigen::Matrix3f SLAM::lookAt(const Eigen::Vector3f& camera_pos, const Eigen::Vector3f& target_pos, const Eigen::Vector3f& up_vector) {
    Eigen::Vector3f forward = (target_pos - camera_pos).normalized();
    Eigen::Vector3f right = up_vector.cross(forward).normalized();
    Eigen::Vector3f up = forward.cross(right).normalized();

    Eigen::Matrix3f rotation;
    rotation.col(0) = right;
    rotation.col(1) = up;
    rotation.col(2) = forward;

    return rotation;
}

Eigen::Vector3f SLAM::getFirstCameraPos(){
    return C1Pose_;
}

Eigen::Vector3f SLAM::getSecondCameraPos(){
    return C2Pose_;
}

bool SLAM::getShowSolution(){
    return showSolution_;
}

