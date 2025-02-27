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

#include "Optimization/g2oBundleAdjustment.h"
#include "Optimization/g2oTypes.h"
#include "Utils/Geometry.h"
#include "Utils/CommonTypes.h"
#include "Optimization/nloptOptimization.h"
#include "Optimization/EigenOptimization.h"
#include "Utils/Measurements.h"
#include "Utils/Utils.h"

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/sba/types_sba.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

#include <random> 

void bundleAdjustment(Map* pMap){
    unordered_map<KeyFrame_,size_t> mKeyFrameId;
    unordered_map<MapPoint_,size_t> mMapPointId;

    //Get all KeyFrames from map
    std::unordered_map<ID,KeyFrame_>&  mKeyFrames = pMap->getKeyFrames();

    //Create optimizer
    g2o::SparseOptimizer optimizer;
    
    std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver =  g2o::make_unique<g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>>();

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
            g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver))
    );

    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(false);

    const float thHuber2D = sqrt(5.99);

    size_t currId = 0;

    //Set optimization
    for(pair<ID,KeyFrame_> pairKF: mKeyFrames){
        KeyFrame_ pKF = pairKF.second;

        //Set KeyFrame data
        Sophus::SE3f kfPose = pKF->getPose();
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(g2o::SE3Quat(kfPose.unit_quaternion().cast<double>(),kfPose.translation().cast<double>()));
        vSE3->setId(currId);
        if(pKF->getId()==0){
            vSE3->setFixed(true);
        }
        optimizer.addVertex(vSE3);

        mKeyFrameId[pKF] = currId;
        currId++;

        //Set MapPoints
        vector<MapPoint_>& vMPs = pKF->getMapPoints();
        for (size_t mpIndex = 0; mpIndex < vMPs.size(); mpIndex++) {
            MapPoint_ pMP = vMPs[mpIndex];
            if (!pMP) continue;

            //Check if this MapPoint has been already added to the optimization
            if (mMapPointId.count(pMP) == 0) {
                VertexSBAPointXYZ* vPoint = new VertexSBAPointXYZ();
                Eigen::Vector3d p3D = pMP->getWorldPosition().cast<double>();
                vPoint->setEstimate(p3D);
                vPoint->setId(currId);
                vPoint->setMarginalized(true);
                optimizer.addVertex(vPoint);

                mMapPointId[pMP] = currId;
                currId++;
            }

            //Set edge
            cv::Point2f uv = pKF->getKeyPoint(mpIndex).pt;
            int octave = pKF->getKeyPoint(mpIndex).octave;
            Eigen::Matrix<double,2,1> obs;
            obs << uv.x, uv.y;

            EdgeSE3ProjectXYZ* e = new EdgeSE3ProjectXYZ();

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(mMapPointId[pMP])));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(mKeyFrameId[pKF])));
            e->setMeasurement(obs);
            e->setInformation(Eigen::Matrix2d::Identity() * pKF->getInvSigma2(octave));

            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(thHuber2D);

            e->pCamera = pKF->getCalibration();

            optimizer.addEdge(e);
        }
    }

    //Run optimization
    optimizer.initializeOptimization();
    optimizer.optimize(20);

    //Recover results
    for(pair<KeyFrame_,ID> pairKeyFrameId : mKeyFrameId){
        KeyFrame_ pKF = pairKeyFrameId.first;
        g2o::VertexSE3Expmap* vertex = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pairKeyFrameId.second));
        Sophus::SE3f sophusPose(vertex->estimate().to_homogeneous_matrix().cast<float>());
        pKF->setPose(sophusPose);
    }

    for(pair<MapPoint_,ID> pairMapPointId : mMapPointId){
        MapPoint_ pMP = pairMapPointId.first;
        VertexSBAPointXYZ* vPoint = static_cast<VertexSBAPointXYZ*>(optimizer.vertex(pairMapPointId.second));
        Eigen::Vector3f p3D = vPoint->estimate().cast<float>();
        pMP->setWorldPosition(p3D);
    }
}

int poseOnlyOptimization(Frame& currFrame){
    //Create optimizer
    g2o::SparseOptimizer optimizer;
    std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver =  g2o::make_unique<g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>>();

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
            g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver))
    );

    optimizer.setAlgorithm(solver);

    const float thHuber2D = sqrt(5.99);

    //Set camera vertex
    Sophus::SE3f fPose = currFrame.getPose();
    g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
    vSE3->setEstimate(g2o::SE3Quat(fPose.unit_quaternion().cast<double>(),fPose.translation().cast<double>()));
    vSE3->setId(0);

    optimizer.addVertex(vSE3);

    //Set edges
    vector<MapPoint_>& vMapPoints = currFrame.getMapPoints();
    vector<EdgeSE3ProjectXYZOnlyPose*> vVertex(vMapPoints.size(),nullptr);
    vector<bool> vInlier(vMapPoints.size(),false);

    for(size_t mpIndex = 0; mpIndex < vMapPoints.size(); mpIndex++){
        MapPoint_ pMP = vMapPoints[mpIndex];

        if(!pMP){
            continue;
        }

        cv::Point2f uv = currFrame.getKeyPoint(mpIndex).pt;
        int octave = currFrame.getKeyPoint(mpIndex).octave;
        Eigen::Matrix<double,2,1> obs;
        obs << uv.x, uv.y;

        EdgeSE3ProjectXYZOnlyPose* e = new EdgeSE3ProjectXYZOnlyPose();

        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
        e->setMeasurement(obs);
        e->setInformation(Eigen::Matrix2d::Identity()*currFrame.getInvSigma2(octave));

        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
        e->setRobustKernel(rk);
        rk->setDelta(thHuber2D);

        e->pCamera = currFrame.getCalibration();
        e->Xworld = pMP->getWorldPosition().cast<double>();

        optimizer.addEdge(e);
        vVertex[mpIndex] = e;
        vInlier[mpIndex] = true;
    }

    for(int mpIndex = 0; mpIndex < 4; mpIndex++){
        vSE3->setEstimate(g2o::SE3Quat(fPose.unit_quaternion().cast<double>(),fPose.translation().cast<double>()));

        optimizer.initializeOptimization(0);
        optimizer.optimize(10);

        int nBad = 0;
        for(size_t j = 0; j < vVertex.size(); j++){
            EdgeSE3ProjectXYZOnlyPose* e = vVertex[j];
            if(!e)
                continue;

            if(!vInlier[mpIndex])
                e->computeError();

            const float chi2 = e->chi2();

            if(chi2>5.991){
                vInlier[j] = false;
                e->setLevel(1);
            }
            else{
                vInlier[j] = true;
                e->setLevel(0);
            }

            if(mpIndex == 2){
                e->setRobustKernel(0);
            }
        }
    }

    int nGood = 0;
    for(int mpIndex = 0; mpIndex < vInlier.size(); mpIndex++){
        if(!vInlier[mpIndex]){
            currFrame.setMapPoint(mpIndex, nullptr);
        }
        else{
            nGood++;
        }
    }

    //Recover optimized data
    Sophus::SE3f sophusPose(vSE3->estimate().to_homogeneous_matrix().cast<float>());
    currFrame.setPose(sophusPose);

    return nGood;
}

void localBundleAdjustment(Map* pMap, ID currKeyFrameId){
    unordered_map<KeyFrame_,size_t> mKeyFrameId, mFixedKeyFrameId;
    unordered_map<MapPoint_,size_t> mMapPointId;

    //Get local map
    set<ID> sLocalMapPoints, sLocalKeyFrames, sFixedKeyFrames;
    pMap->getLocalMapOfKeyFrame(currKeyFrameId,sLocalMapPoints,sLocalKeyFrames,sFixedKeyFrames);

    //Create optimizer
    g2o::SparseOptimizer optimizer;
    std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver =  g2o::make_unique<g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>>();

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
            g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver))
    );

    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(false);

    const float thHuber2D = sqrt(5.99);

    vector<EdgeSE3ProjectXYZ*> vEdges;
    vector<ID> vEdgesKF;
    vector<ID> vEdgesMP;
    vector<size_t> vMpInKf;

    vEdges.reserve(sLocalKeyFrames.size()*sLocalMapPoints.size());
    vEdgesKF.reserve(sLocalKeyFrames.size()*sLocalMapPoints.size());
    vEdgesMP.reserve(sLocalKeyFrames.size()*sLocalMapPoints.size());
    vMpInKf.reserve(sLocalKeyFrames.size()*sLocalMapPoints.size());

    size_t currId = 0;
    //Set optimization
    for(ID kfID : sLocalKeyFrames){
        KeyFrame_ pKF = pMap->getKeyFrame(kfID);

        //Set KeyFrame data
        Sophus::SE3f kfPose = pKF->getPose();
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(g2o::SE3Quat(kfPose.unit_quaternion().cast<double>(),kfPose.translation().cast<double>()));
        vSE3->setId(currId);
        if(pKF->getId()==0){
            vSE3->setFixed(true);
        }
        optimizer.addVertex(vSE3);

        mKeyFrameId[pKF] = currId;
        currId++;

        //Set MapPoints
        vector<MapPoint_>& vMPs = pKF->getMapPoints();
        for (size_t mpIndex = 0; mpIndex < vMPs.size(); mpIndex++) {
            MapPoint_ pMP = vMPs[mpIndex];
            if (!pMP) continue;


            assert(sLocalMapPoints.count(pMP->getId()) != 0);
            //Check if this MapPoint has been already added to the optimization
            if (mMapPointId.count(pMP) == 0) {
                VertexSBAPointXYZ* vPoint = new VertexSBAPointXYZ();
                Eigen::Vector3d p3D = pMP->getWorldPosition().cast<double>();
                vPoint->setEstimate(p3D);
                vPoint->setId(currId);
                vPoint->setMarginalized(true);
                optimizer.addVertex(vPoint);

                mMapPointId[pMP] = currId;
                currId++;
            }

            //Set edge
            cv::Point2f uv = pKF->getKeyPoint(mpIndex).pt;
            int octave = pKF->getKeyPoint(mpIndex).octave;
            Eigen::Matrix<double,2,1> obs;
            obs << uv.x, uv.y;

            EdgeSE3ProjectXYZ* e = new EdgeSE3ProjectXYZ();

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(mMapPointId[pMP])));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(mKeyFrameId[pKF])));
            e->setMeasurement(obs);
            e->setInformation(Eigen::Matrix2d::Identity() * pKF->getInvSigma2(octave));

            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(thHuber2D);

            e->pCamera = pKF->getCalibration();

            vEdges.push_back(e);
            vEdgesKF.push_back(kfID);
            vEdgesMP.push_back(pMP->getId());
            vMpInKf.push_back(mpIndex);

            optimizer.addEdge(e);
        }
    }

    //Set fixed KeyFrames
    for(ID fixedKFId : sFixedKeyFrames){
        KeyFrame_ pKF = pMap->getKeyFrame(fixedKFId);

        //Set KeyFrame data
        Sophus::SE3f kfPose = pKF->getPose();
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(g2o::SE3Quat(kfPose.unit_quaternion().cast<double>(),kfPose.translation().cast<double>()));
        vSE3->setId(currId);
        vSE3->setFixed(true);

        optimizer.addVertex(vSE3);

        mFixedKeyFrameId[pKF] = currId;
        currId++;

        //Set MapPoints
        vector<MapPoint_>& vMPs = pKF->getMapPoints();
        for (size_t mpIndex = 0; mpIndex < vMPs.size(); mpIndex++) {
            MapPoint_ pMP = vMPs[mpIndex];
            if (!pMP) continue;
            if(sLocalMapPoints.count(pMP->getId()) == 0) continue;

            assert(mMapPointId.count(pMP) > 0);

            //Set edge
            cv::Point2f uv = pKF->getKeyPoint(mpIndex).pt;
            int octave = pKF->getKeyPoint(mpIndex).octave;
            Eigen::Matrix<double,2,1> obs;
            obs << uv.x, uv.y;

            EdgeSE3ProjectXYZ* e = new EdgeSE3ProjectXYZ();

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(mMapPointId[pMP])));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(mFixedKeyFrameId[pKF])));
            e->setMeasurement(obs);
            e->setInformation(Eigen::Matrix2d::Identity() * pKF->getInvSigma2(octave));

            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(thHuber2D);

            e->pCamera = pKF->getCalibration();

            vEdges.push_back(e);
            vEdgesKF.push_back(fixedKFId);
            vEdgesMP.push_back(pMP->getId());
            vMpInKf.push_back(mpIndex);

            optimizer.addEdge(e);
        }
    }

    //Run optimization
    optimizer.initializeOptimization();
    optimizer.optimize(5);

    //Remove outliers
    for(size_t mpIndex = 0; mpIndex < vEdges.size(); mpIndex++){
        EdgeSE3ProjectXYZ* e = vEdges[mpIndex];

        if(e->chi2() > 5.991 || !e->isDepthPositive()){
            e->setLevel(1);
        }

        e->setRobustKernel(0);
    }

    optimizer.initializeOptimization(0);
    optimizer.optimize(10);

    //Remove outlier observations from map
    for(size_t mpIndex = 0; mpIndex < vEdges.size(); mpIndex++){
        EdgeSE3ProjectXYZ* e = vEdges[mpIndex];

        //Remove observation
        if(e->chi2() > 5.991 || !e->isDepthPositive()){
            //Remove from KeyFrame
            pMap->getKeyFrame(vEdgesKF[mpIndex])->setMapPoint(vMpInKf[mpIndex], nullptr);

            //Remove from map
            pMap->removeObservation(vEdgesKF[mpIndex],vEdgesMP[mpIndex]);

            pMap->checkKeyFrame(vEdgesKF[mpIndex]);
        }
    }

    //Recover results
    for(pair<KeyFrame_,size_t> pairKeyFrameId : mKeyFrameId){
        KeyFrame_ pKF = pairKeyFrameId.first;
        g2o::VertexSE3Expmap* vertex = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pairKeyFrameId.second));
        Sophus::SE3f sophusPose(vertex->estimate().to_homogeneous_matrix().cast<float>());
        pKF->setPose(sophusPose);
    }

    for(pair<MapPoint_,size_t> pairMapPointId : mMapPointId){
        MapPoint_ pMP = pairMapPointId.first;
        VertexSBAPointXYZ* vPoint = static_cast<VertexSBAPointXYZ*>(optimizer.vertex(pairMapPointId.second));
        Eigen::Vector3f p3D = vPoint->estimate().cast<float>();
        pMP->setWorldPosition(p3D);
    }
}

void deformationOptimization(std::shared_ptr<Map> pMap, Settings& settings, std::shared_ptr<MapVisualizer>& mapVisualizer,
                            const std::vector<Eigen::Vector3f> originalPoints, const std::vector<Eigen::Vector3f> movedPoints) {
    float simulatedRepErrorStanDesv = settings.getSimulatedRepError();
    float SimulatedDepthErrorStanDesv = settings.getSimulatedDepthWeight()/1000;

    double repBalanceWeight = settings.getOptRepWeight();
    double arapBalanceWeight = settings.getOptArapWeight();
    double globalBalanceWeight = settings.getOptGlobalWeight();
    double alphaWeight = settings.getOptAlphaWeight();
    double betaWeight = settings.getOptBetaWeight();

    std::string OptSelection = settings.getOptSelection();
    std::string OptWeightsSelection = settings.getOptWeightsSelection();

    int nOptimizations = settings.getnOptimizations();
    int nOptIterations = settings.getnOptIterations();

    int NloptnOptimizations = settings.getNloptnOptimizations();
    double NloptRelTolerance = settings.getNloptRelTolerance();
    double NloptAbsTolerance = settings.getNloptAbsTolerance();
    double NloptRepLowerBound = settings.getNloptRepLowerBound();
    double NloptRepUpperBound = settings.getNloptRepUpperBound();
    double NloptGlobalLowerBound = settings.getNloptGlobalLowerBound();
    double NloptGlobalUpperBound = settings.getNloptGlobalUpperBound();
    double NloptArapLowerBound = settings.getNloptArapLowerBound();
    double NloptArapUpperBound = settings.getNloptArapUpperBound();

    bool drawRaysSelection = settings.getDrawRaysSelection();

    std::ofstream outFile_;
    std::string filePath_ = settings.getExpFilePath();
    outFile_.imbue(std::locale("es_ES.UTF-8"));

    std::unordered_map<ID, std::shared_ptr<MapPoint>> mapPoints_corrected = pMap->getMapPoints();

    double optimizationUpdate = 100;
    for(int i = 1; i <= nOptimizations && optimizationUpdate >= (0.0005*mapPoints_corrected.size()); i++){ 
        // correct error
        if (OptSelection == "open3DArap") {
            arapOpen3DOptimization(pMap.get());
        } else if(OptSelection == "twoOptimizations") {
            if (OptWeightsSelection == "nlopt") {
                std::cout << "START new optimization. "  << std::endl;

                std::unordered_map<ID, std::shared_ptr<MapPoint>> mapPoints_corrected = pMap->getMapPoints();
                nlopt::opt opt(nlopt::LN_NELDERMEAD, 3);
                
                std::vector<double> lb = {NloptRepLowerBound, NloptGlobalLowerBound, NloptArapLowerBound};
                std::vector<double> ub = {NloptRepUpperBound, NloptGlobalUpperBound, NloptArapUpperBound};
                opt.set_lower_bounds(lb);
                opt.set_upper_bounds(ub);

                OptimizationData optData;
                optData.pMap = pMap->clone();
                optData.nOptIterations = nOptIterations;
                optData.repErrorStanDesv = simulatedRepErrorStanDesv;
                optData.alpha = alphaWeight;
                optData.beta = betaWeight;
                optData.depthUncertainty = SimulatedDepthErrorStanDesv;

                opt.set_min_objective(outerObjective, &optData);

                std::vector<double> x = {repBalanceWeight, globalBalanceWeight, arapBalanceWeight};

                opt.set_xtol_rel(NloptRelTolerance);
                opt.set_xtol_abs(NloptAbsTolerance);
                opt.set_maxeval(NloptnOptimizations);

                double minf;
                nlopt::result result = opt.optimize(x, minf);

                std::cout << "\nWEIGHTS OPTIMIZED" << std::endl;
                std::cout << "Optimized repBalanceWeight: " << x[0] << std::endl;
                std::cout << "Optimized globalBalanceWeight: " << x[1] << std::endl;
                std::cout << "Optimized arapBalanceWeight: " << x[2] << std::endl;
                std::cout << "Final minimized ABSOLUTE error: " << minf << std::endl;

                std::cout << "\nFinal optimization with optimized weights:\n" << std::endl;

                arapOptimization(pMap.get(), x[0], x[1], x[2], alphaWeight, betaWeight, SimulatedDepthErrorStanDesv, 
                                    nOptIterations, &optimizationUpdate);

                repBalanceWeight = x[0];
                globalBalanceWeight = x[1];
                arapBalanceWeight = x[2];
            } else {
                Eigen::VectorXd x(3);
                // Initial values
                x[0] = repBalanceWeight;
                x[1] = globalBalanceWeight;
                x[2] = arapBalanceWeight;

                EigenOptimizationFunctor functor(pMap->clone(), nOptIterations, simulatedRepErrorStanDesv, alphaWeight, betaWeight, 
                                                    SimulatedDepthErrorStanDesv); 
                
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

                arapOptimization(pMap.get(), x[0], x[1], x[2], alphaWeight, betaWeight, SimulatedDepthErrorStanDesv, 
                                    nOptIterations, &optimizationUpdate);
            }
        } else {
            arapOptimization(pMap.get(), repBalanceWeight, globalBalanceWeight, arapBalanceWeight, alphaWeight, betaWeight,
                            SimulatedDepthErrorStanDesv, nOptIterations, &optimizationUpdate);
        }

        std::cout << "\nOptimization COMPLETED... " << i << " / " << nOptimizations << " iterations." << std::endl;
        std::cout << "\nOptimization change: " << optimizationUpdate << std::endl;
        
        
        mapVisualizer->update(drawRaysSelection);

        if (i != nOptimizations) {
            std::cout << i << " / " << nOptimizations << " MEASUREMENTS: \n";
            outFile_.open(filePath_, std::ios::app);
            if (outFile_.is_open()) {
                outFile_ << i << " / " << nOptimizations << " MEASUREMENTS: \n";

                outFile_.close();
            } else {
                std::cerr << "Unable to open file for writing" << std::endl;
            }

            measureRelativeMapErrors(pMap, filePath_);
            if(originalPoints.empty() || movedPoints.empty()) {
                measureRealAbsoluteMapErrors(pMap, filePath_);
            } else {
                measureSimAbsoluteMapErrors(pMap, originalPoints, movedPoints, filePath_);
            }
        }
    }

    outFile_.open(filePath_, std::ios::app);
    if (outFile_.is_open()) {
        outFile_ << "FINAL MEASUREMENTS: \n";

        outFile_.close();
    } else {
        std::cerr << "Unable to open file for writing" << std::endl;
    }

    mapVisualizer->update(drawRaysSelection);
}

void arapOptimization(Map* pMap, double repBalanceWeight, double globalBalanceWeight, double arapBalanceWeight, double alphaWeight, 
                        double betaWeight, float DepthError, int nOptIterations, double* optimizationUpdate){
    unordered_map<KeyFrame_,size_t> mKeyFrameId;
    unordered_map<MapPoint_,size_t> mMapPointId;
    unordered_map<RotationMatrix_,size_t> mRotId;
    unordered_map<TransformationMatrix_,size_t> mTGlobalId;

    //Get all KeyFrames from map
    std::unordered_map<ID,KeyFrame_>&  mKeyFrames = pMap->getKeyFrames();

    // Create optimizer
    g2o::SparseOptimizer optimizer;

    std::unique_ptr<g2o::BlockSolverX::LinearSolverType> linearSolver =  g2o::make_unique<g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>>();

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
            g2o::make_unique<g2o::BlockSolverX>(std::move(linearSolver))
    );

    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(false);

    const float thHuber2D = sqrt(5.99);
    const float deltaMono = sqrt(100.991);

    Sophus::SE3f T_global;

    size_t currId = 0;

    //Set optimization

    // Add ARAP edges
    for (auto k1 = mKeyFrames.begin(); k1 != mKeyFrames.end(); ++k1) { // [DUDA] deberia hacerlo diferente?
        for (auto k2 = std::next(k1); k2 != mKeyFrames.end(); ++k2) {
            KeyFrame_ pKF1 = k2->second;
            KeyFrame_ pKF2 = k1->second;
            int kf1ID = k2->first;
            int kf2ID = k1->first;
            // std::cout << "\nPair: (" << k1->first << ", " << k2->first<< ")\n";

            vector<MapPoint_>& v1MPs = pKF1->getMapPoints();
            vector<MapPoint_>& v2MPs = pKF2->getMapPoints(); // [DUDA] Deben darse como puntos 3D diferentes porque los optimizo como diferentes
            // [DUDA] Primero asumire que todos son diferentes, deberia fusionar una vez hecha la optimización si quedan muy juntos??

            // MESH CREATION
            std::vector<Eigen::Vector3d> v1Positions = extractPositions(v1MPs);
            std::vector<Eigen::Vector3d> v2Positions = extractPositions(v2MPs);

            // OPEN3D LIBRARY
            std::shared_ptr<open3d::geometry::TriangleMesh> mesh;
            mesh = ComputeDelaunayTriangulation3D(v1Positions);
            mesh->ComputeAdjacencyList();
            std::unordered_map<Eigen::Vector2i,
                        double,
                        open3d::utility::hash_eigen<Eigen::Vector2i>> edge_weights = ComputeEdgeWeightsCot(mesh, 0);

            T_global = pMap->getGlobalKeyFramesTransformation(k2->first, k1->first);

            Eigen::Matrix3d rotation;
            Eigen::Vector3d translation;

            if (T_global.translation().norm() == 0 &&  T_global.rotationMatrix().isApprox(Eigen::Matrix3f::Identity())) {
                // std::cout << "Estimate initial Rotation and Translation \n";
                // EstimateRotationAndTranslation(v1Positions, v2Positions, rotation, translation);
                // std::cout << "Rotation: " << rotation << "\n";
                // std::cout << "translation: " << translation << "\n";
                rotation.setIdentity();
                translation.setZero();
                T_global = Sophus::SE3f(rotation.cast<float>(), translation.cast<float>());
            }

            auto posIndexes = createVectorMap(mesh->vertices_, v1Positions);

            std::map<size_t, size_t> invertedPosIndexes;

            for (const auto& pair : posIndexes) {
                invertedPosIndexes[pair.second] = pair.first;
            }

            std::vector<Sophus::SO3d> Rs(v1Positions.size(),Sophus::SO3d(Sophus::SO3d::exp(Eigen::Vector3d::Zero())));
            computeR(mesh, v1Positions, v2Positions, Rs);

            // Visualize OPEN3D the meshes
            // open3d::visualization::Visualizer visualizer;
            // visualizer.CreateVisualizerWindow("Mesh Visualization");
            // Eigen::Vector3d red(1.0, 0.0, 0.0);
            // mesh->vertex_colors_.resize(mesh->vertices_.size(), red);
            // visualizer.AddGeometry(mesh);
            // visualizer.Run();
            // visualizer.DestroyVisualizerWindow();

            TransformationMatrix_ T = std::make_shared<Sophus::SE3f>(T_global);

            g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
            vSE3->setEstimate(g2o::SE3Quat(T_global.unit_quaternion().cast<double>(),T_global.translation().cast<double>()));
            vSE3->setId(currId);
            optimizer.addVertex(vSE3);
            mTGlobalId[T] = currId;
            currId++;

            double ds_1 = pKF1->getEstimatedDepthScale();
            double ds_2 = pKF2->getEstimatedDepthScale();

            VertexDepthScale* vScale1 = new VertexDepthScale();
            vScale1->setEstimate(ds_1);
            vScale1->setId(currId);
            optimizer.addVertex(vScale1);
            mKeyFrameId[pKF1] = currId;
            currId++;

            VertexDepthScale* vScale2 = new VertexDepthScale();
            vScale2->setEstimate(ds_2);
            vScale2->setId(currId);
            optimizer.addVertex(vScale2);
            mKeyFrameId[pKF2] = currId;
            currId++;

            for (size_t mpIndex = 0; mpIndex < v1MPs.size(); mpIndex++) {
                MapPoint_ pMPi1 = v1MPs[mpIndex];
                MapPoint_ pMPi2 = v2MPs[mpIndex];
                if (!pMPi1) continue;
                if (!pMPi2) continue;

                MapPoint_ firstPointToOptimize = pMPi1;
                MapPoint_ secondPointToOptimize = pMPi2;

                //Check if this MapPoint has been already added to the optimization
                if (mMapPointId.count(firstPointToOptimize) == 0) {
                    VertexSBAPointXYZ* vPoint = new VertexSBAPointXYZ();
                    Eigen::Vector3d p3D = firstPointToOptimize->getWorldPosition().cast<double>();
                    vPoint->setEstimate(p3D);
                    vPoint->setId(currId);
                    //vPoint->setMarginalized(true);
                    optimizer.addVertex(vPoint);

                    mMapPointId[firstPointToOptimize] = currId;
                    //std::cout << "Id: (" << currId << ")\n";
                    currId++;
                }

                
                if (mMapPointId.count(secondPointToOptimize) == 0) {
                    VertexSBAPointXYZ* vPoint = new VertexSBAPointXYZ();
                    Eigen::Vector3d p3D = secondPointToOptimize->getWorldPosition().cast<double>();
                    vPoint->setEstimate(p3D);
                    vPoint->setId(currId);
                    //vPoint->setMarginalized(true);
                    optimizer.addVertex(vPoint);

                    mMapPointId[secondPointToOptimize] = currId;
                    //std::cout << "ID: (" << currId << ")\n";
                    currId++;
                }

                // REPROJECTION ERROR //
                //Set fisrt projection edge

                int index_in_kf1 = pMap->isMapPointInKeyFrame(pMPi1->getId(), kf1ID);
                int index_in_kf2 = pMap->isMapPointInKeyFrame(pMPi2->getId(), kf2ID);

                if(index_in_kf1 < 0 || index_in_kf2 < 0) continue;
                size_t idx1 = (size_t)index_in_kf1;
                size_t idx2 = (size_t)index_in_kf2;

                cv::Point2f uv1 = pKF1->getKeyPoint(idx1).pt;
                int octave = pKF1->getKeyPoint(idx1).octave;
                Eigen::Matrix<double,2,1> obs;
                obs << uv1.x, uv1.y;

                EdgeSE3ProjectXYZPerKeyFrameOnlyPoints* eKF1 = new EdgeSE3ProjectXYZPerKeyFrameOnlyPoints();

                eKF1->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(mMapPointId[firstPointToOptimize])));
                eKF1->setMeasurement(obs);
                eKF1->setInformation(Eigen::Matrix2d::Identity() * pKF1->getInvSigma2(octave) * repBalanceWeight);

                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                rk->setDelta(deltaMono);
                eKF1->setRobustKernel(rk);

                eKF1->pCamera = pKF1->getCalibration();
                Sophus::SE3f kfPose = pKF1->getPose();
                eKF1->cameraPose = g2o::SE3Quat(kfPose.unit_quaternion().cast<double>(),kfPose.translation().cast<double>());

                optimizer.addEdge(eKF1);

                //Set second projection edge
                cv::Point2f uv2 = pKF2->getKeyPoint(idx2).pt;
                octave = pKF2->getKeyPoint(idx2).octave;
                obs << uv2.x, uv2.y;

                EdgeSE3ProjectXYZPerKeyFrameOnlyPoints* eKF2 = new EdgeSE3ProjectXYZPerKeyFrameOnlyPoints();

                eKF2->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(mMapPointId[secondPointToOptimize])));
                eKF2->setMeasurement(obs);
                eKF2->setInformation(Eigen::Matrix2d::Identity() * pKF2->getInvSigma2(octave) * repBalanceWeight);

                rk = new g2o::RobustKernelHuber;
                rk->setDelta(deltaMono);
                eKF2->setRobustKernel(rk);

                eKF2->pCamera = pKF2->getCalibration();
                kfPose = pKF2->getPose();
                eKF2->cameraPose = g2o::SE3Quat(kfPose.unit_quaternion().cast<double>(),kfPose.translation().cast<double>());

                optimizer.addEdge(eKF2);

                // DEPTH ERROR //
                //Set fisrt depth edge
                double d1 = pKF1->getDepthMeasure(uv1.x, uv1.y, false);
                EdgeDepthCorrection* eD1 = new EdgeDepthCorrection();

                eD1->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(mMapPointId[firstPointToOptimize])));
                eD1->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(mKeyFrameId[pKF1])));
                eD1->setMeasurement(d1);
                Eigen::Matrix<double, 1, 1> informationMatrixDepth;
                double depthUncertainty = static_cast<double>(DepthError);
                informationMatrixDepth(0, 0) = 1/(depthUncertainty * depthUncertainty);
                eD1->setInformation(informationMatrixDepth);
                kfPose = pKF1->getPose();
                eD1->cameraPose = g2o::SE3Quat(kfPose.unit_quaternion().cast<double>(),kfPose.translation().cast<double>());
                //eD1->scale = ds_1;
                optimizer.addEdge(eD1);

                // double d1 = pKF1->getDepthMeasure(uv1.x, uv1.y, false);
                // EdgeDepthWithoutScaleCorrection* eD1 = new EdgeDepthWithoutScaleCorrection();

                // eD1->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(mMapPointId[firstPointToOptimize])));
                // eD1->setMeasurement(d1);
                // Eigen::Matrix<double, 1, 1> informationMatrixDepth;
                // double depthUncertainty = static_cast<double>(DepthError);
                // informationMatrixDepth(0, 0) = 1/(depthUncertainty * depthUncertainty);
                // eD1->setInformation(informationMatrixDepth);
                // kfPose = pKF1->getPose();
                // eD1->cameraPose = g2o::SE3Quat(kfPose.unit_quaternion().cast<double>(),kfPose.translation().cast<double>());
                // eD1->scale = ds_1;
                // optimizer.addEdge(eD1);

                //Set second depth edge
                double d2 = pKF2->getDepthMeasure(uv2.x, uv2.y, false);

                EdgeDepthCorrection* eD2 = new EdgeDepthCorrection();

                eD2->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(mMapPointId[secondPointToOptimize])));
                eD2->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(mKeyFrameId[pKF2])));
                eD2->setMeasurement(d2);
                eD2->setInformation(informationMatrixDepth);
                kfPose = pKF2->getPose();
                eD2->cameraPose = g2o::SE3Quat(kfPose.unit_quaternion().cast<double>(),kfPose.translation().cast<double>());
                optimizer.addEdge(eD2);

                // double d2 = pKF2->getDepthMeasure(uv2.x, uv2.y, false);

                // EdgeDepthWithoutScaleCorrection* eD2 = new EdgeDepthWithoutScaleCorrection();

                // eD2->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(mMapPointId[secondPointToOptimize])));
                // eD2->setMeasurement(d2);
                // eD2->setInformation(informationMatrixDepth);
                // kfPose = pKF2->getPose();
                // eD2->cameraPose = g2o::SE3Quat(kfPose.unit_quaternion().cast<double>(),kfPose.translation().cast<double>());
                // eD2->scale = ds_2;
                // optimizer.addEdge(eD2);


                auto it = invertedPosIndexes.find(mpIndex);
                size_t i = 0;
                if (it != invertedPosIndexes.end()) {
                    i = it->second;
                    // std::cout << "v1Position: (" << v1Positions[mpIndex][0] << ", " << v1Positions[mpIndex][1] << ", " << v1Positions[mpIndex][2] << ")\n";
                    // std::cout << "v2Position: (" << v2Positions[mpIndex][0] << ", " << v2Positions[mpIndex][1] << ", " << v2Positions[mpIndex][2] << ")\n";
                    // std::cout << "point mesh: (" << mesh->vertices_[i][0] << ", " << mesh->vertices_[i][1] << ", " << mesh->vertices_[i][2] << ")\n";
                    // std::cout << "index: (" << mpIndex << ")\n";
                } else {
                    continue;
                }

                std::unordered_set<int> jIndexes = mesh->adjacency_list_[i];

                if (jIndexes.empty()) continue;

                double distancesInvTipDesv = getInvUncertainty(mesh, v1Positions, v2Positions, i);

                for (int j : jIndexes) {
                    MapPoint_ pMPj1 = v1MPs[posIndexes[j]];
                    MapPoint_ pMPj2 = v2MPs[posIndexes[j]];
                    if (!pMPj1) continue;
                    if (!pMPj2) continue;
                    MapPoint_ firstjPointToOptimize = pMPj1;
                    MapPoint_ secondjPointToOptimize = pMPj2;

                    if (mMapPointId.count(firstjPointToOptimize) == 0) {
                        VertexSBAPointXYZ* vPoint = new VertexSBAPointXYZ();
                        Eigen::Vector3d p3D = firstjPointToOptimize->getWorldPosition().cast<double>();
                        vPoint->setEstimate(p3D);
                        vPoint->setId(currId);
                        //vPoint->setMarginalized(true);
                        optimizer.addVertex(vPoint);

                        mMapPointId[firstjPointToOptimize] = currId;
                        //std::cout << "ID: (" << currId << ")\n";
                        currId++;
                    }

                    if (mMapPointId.count(secondjPointToOptimize) == 0) {
                        VertexSBAPointXYZ* vPoint = new VertexSBAPointXYZ();
                        Eigen::Vector3d p3D = secondjPointToOptimize->getWorldPosition().cast<double>();
                        vPoint->setEstimate(p3D);
                        vPoint->setId(currId);
                        //vPoint->setMarginalized(true);
                        optimizer.addVertex(vPoint);

                        mMapPointId[secondjPointToOptimize] = currId;
                        //std::cout << "ID: (" << currId << ")\n";
                        currId++;
                    }

                    // Eigen::Vector3f t1 = pKF1->getPose().inverse().translation();
                    // Eigen::Vector3f t2 = pKF2->getPose().inverse().translation();
                    // float norm = (t2 - t1).norm();

                    //Set ARAP edge
                    EdgeARAP* eArap = new EdgeARAP();

                    //eArap->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(mMapPointId[firstPointToOptimize])));
                    eArap->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(mMapPointId[firstPointToOptimize])));
                    eArap->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(mMapPointId[secondPointToOptimize])));
                    eArap->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(mMapPointId[firstjPointToOptimize])));
                    eArap->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(mMapPointId[secondjPointToOptimize])));
                    eArap->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(mTGlobalId[T])));

                    eArap->Ri = Rs[i];
                    eArap->Rj = Rs[j];
                    eArap->weight = edge_weights[GetOrderedEdge(i, j)];
                    eArap->alpha = alphaWeight;
                    eArap->beta = betaWeight;
                    double area = static_cast<double>(mesh->GetSurfaceArea());
                    eArap->area = area;
                    eArap->nPoints = mesh->vertices_.size();

                    Eigen::Matrix<double, 1, 1> informationMatrixArap;
                    informationMatrixArap(0, 0) = (arapBalanceWeight) * std::pow(mesh->triangles_.size(), 2);// * std::pow(area, 5); // * distancesInvTipDesv;

                    eArap->setInformation(informationMatrixArap);
                    double measurementArap = 0.0;
                    eArap->setMeasurement(measurementArap);

                    optimizer.addEdge(eArap);
                }
            }
        }
    }

    //Run optimization
    optimizer.initializeOptimization();

    std::cout << "Starting... \n";
    optimizer.optimize(nOptIterations);
    // tras unas iteraciones con las soluciones o modificando lo que tienes

    std::cout << "Optimized \n";

    for(pair<KeyFrame_,ID> pairKeyFrameId : mKeyFrameId){
        KeyFrame_ pKF = pairKeyFrameId.first;
        VertexDepthScale* vertex = static_cast<VertexDepthScale*>(optimizer.vertex(pairKeyFrameId.second));
        double dScale(vertex->estimate());
        pKF->setEstimatedDepthScale(dScale);
    }

    if (optimizationUpdate) {
        *optimizationUpdate = 0;
    }

    for(pair<MapPoint_,ID> pairMapPointId : mMapPointId){
        MapPoint_ pMP = pairMapPointId.first;
        VertexSBAPointXYZ* vPoint = static_cast<VertexSBAPointXYZ*>(optimizer.vertex(pairMapPointId.second));
        Eigen::Vector3f p3D = vPoint->estimate().cast<float>();

        if (optimizationUpdate) {
            Eigen::Vector3f originalPosition = pMP->getWorldPosition();
            double changeMagnitude = (originalPosition - p3D).norm();
            *optimizationUpdate += changeMagnitude;
        }

        pMP->setWorldPosition(p3D);
    }

    for(pair<RotationMatrix_,ID> pairRotationMatrixId : mRotId){
        RotationMatrix_ Rots = pairRotationMatrixId.first;
        VertexSO3* mRot = static_cast<VertexSO3*>(optimizer.vertex(pairRotationMatrixId.second));
        Sophus::SO3d Rotation = mRot->estimate();
        // std::cout << "Rotation matrix:\n" << Rotation << std::endl;
    }

    Sophus::SE3f TGlobal;

    for(pair<TransformationMatrix_,ID> pairTransformationGlobalMatrixId : mTGlobalId){
        TransformationMatrix_ T = pairTransformationGlobalMatrixId.first;
        g2o::VertexSE3Expmap* mT = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pairTransformationGlobalMatrixId.second));
        TGlobal = Sophus::SE3f(mT->estimate().to_homogeneous_matrix().cast<float>());
        //std::cout << "Global Rotation matrix:\n" << Rotation << std::endl;
    }
    pMap->insertGlobalKeyFramesTransformation(0, 1, TGlobal);
}

void arapOpen3DOptimization(Map* pMap){
    unordered_map<KeyFrame_,size_t> mKeyFrameId;
    unordered_map<MapPoint_,size_t> mMapPointId;
    unordered_map<RotationMatrix_,size_t> mRotId;

    //Get all KeyFrames from map
    std::unordered_map<ID,KeyFrame_>&  mKeyFrames = pMap->getKeyFrames();

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> distrib(0, 9);

    for (auto k1 = mKeyFrames.begin(); k1 != mKeyFrames.end(); ++k1) { // [DUDA] deberia hacerlo diferente?
        for (auto k2 = std::next(k1); k2 != mKeyFrames.end(); ++k2) {
            KeyFrame_ pKF1 = k2->second;
            KeyFrame_ pKF2 = k1->second;
            std::cout << "\n Pair: (" << k1->first << ", " << k2->first<< ")\n";

            vector<MapPoint_>& v1MPs = pKF1->getMapPoints();
            vector<MapPoint_>& v2MPs = pKF2->getMapPoints(); // [DUDA] Deben darse como puntos 3D diferentes porque los optimizo como diferentes
            // [DUDA] Primero asumire que todos son diferentes, deberia fusionar una vez hecha la optimización si quedan muy juntos??
            

            // MESH CREATION
            std::vector<Eigen::Vector3d> v1Positions = extractPositions(v1MPs);
            std::vector<Eigen::Vector3d> v2Positions = extractPositions(v2MPs);

            std::shared_ptr<open3d::geometry::TriangleMesh> mesh;

            // Perform Delaunay Reconstruction
            mesh = ComputeDelaunayTriangulation3D(v1Positions);
            mesh->ComputeAdjacencyList();

            auto posIndexes1 = createVectorMap(mesh->vertices_, v1Positions);
            std::cout << "mesh size 1: (" << mesh->vertices_.size() << ")\n";

            std::map<size_t, size_t> invertedPosIndexes1;
            for (const auto& pair : posIndexes1) {
                invertedPosIndexes1[pair.second] = pair.first;
            } 

            std::vector<int> constraint_vertex_indices(10);  // Initialize vector with 200 elements

            // for (int mpIndex = 0; mpIndex < 10; ++mpIndex) {
            //     //constraint_vertex_indices[mpIndex] = distrib(gen);  // Assign values from 0 to 199
            //     constraint_vertex_indices[mpIndex] = mpIndex;
            // }

            std::shared_ptr<open3d::geometry::TriangleMesh> deformed_mesh = mesh->DeformAsRigidAsPossible(
                constraint_vertex_indices, v2Positions, 500,  // max_iter = 50
                open3d::geometry::TriangleMesh::DeformAsRigidAsPossibleEnergy::Spokes, 0.01
            );
            // std::shared_ptr<open3d::geometry::TriangleMesh> deformed_mesh = mesh->DeformAsRigidAsPossible();

            for (size_t mpIndex = 0; mpIndex < v1MPs.size(); mpIndex++) {
                MapPoint_ pMPi1 = v1MPs[mpIndex];
                MapPoint_ pMPi2 = v2MPs[mpIndex];

                auto it1 = invertedPosIndexes1.find(mpIndex);
                size_t meshIndex1 = 0;
                if (it1 != invertedPosIndexes1.end()) {
                    meshIndex1 = it1->second;
                    // std::cout << "v1Position: (" << v1Positions[mpIndex][0] << ", " << v1Positions[mpIndex][1] << ", " << v1Positions[mpIndex][2] << ")\n";
                    // std::cout << "point mesh: (" << mesh->vertices_[meshIndex1][0] << ", " << mesh->vertices_[meshIndex1][1] << ", " << mesh->vertices_[meshIndex1][2] << ")\n";
                    // std::cout << "index: (" << mpIndex << ")\n";
                } else {
                    continue;
                }

                Eigen::Vector3f p3D1 = mesh->vertices_[meshIndex1].cast<float>();
                Eigen::Vector3f p3D2 = deformed_mesh->vertices_[meshIndex1].cast<float>();
                pMPi1->setWorldPosition(p3D1);
                pMPi2->setWorldPosition(p3D2);
            }

            std::cout << "DeformAsRigidAsPossible completed!\n";

            v1MPs = pKF1->getMapPoints();
            v2MPs = pKF2->getMapPoints(); 
            
            v1Positions = extractPositions(v1MPs);
            v2Positions = extractPositions(v2MPs);

            // Perform Delaunay Reconstruction
            mesh = ComputeDelaunayTriangulation3D(v1Positions);
            mesh->ComputeAdjacencyList();

            posIndexes1 = createVectorMap(mesh->vertices_, v1Positions);

            for (const auto& pair : posIndexes1) {
                invertedPosIndexes1[pair.second] = pair.first;
            } 
        }
    }
}

double getInvUncertainty(std::shared_ptr<open3d::geometry::TriangleMesh> mesh, 
                        std::vector<Eigen::Vector3d> v1Positions, 
                        std::vector<Eigen::Vector3d> v2Positions,
                        size_t i){
    auto posIndexes = createVectorMap(mesh->vertices_, v1Positions);

    std::unordered_map<Eigen::Vector2i,
                        double,
                        open3d::utility::hash_eigen<Eigen::Vector2i>> edge_weights = ComputeEdgeWeightsCot(mesh, 0);

    double Si = 0.0;

    std::unordered_set<int> jIndexes = mesh->adjacency_list_[i];
    
    for (int j : jIndexes) {
        double weight = edge_weights[GetOrderedEdge(i, j)];

        Eigen::Vector3d pi1 = v1Positions[posIndexes[i]];
        Eigen::Vector3d pj1 = v1Positions[posIndexes[j]];
        Eigen::Vector3d pi2 = v2Positions[posIndexes[i]];
        Eigen::Vector3d pj2 = v2Positions[posIndexes[j]];

        Eigen::Vector3d undeformed_eij = pi1 - pj1;
        Eigen::Vector3d deformed_eij = pi2 - pj2;

        Si += weight * undeformed_eij.transpose() * deformed_eij;
    }

    return Si;
}