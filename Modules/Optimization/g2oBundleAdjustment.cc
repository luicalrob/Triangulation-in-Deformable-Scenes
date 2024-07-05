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

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/sba/types_sba.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>


#include <g2o/solvers/dense/linear_solver_dense.h>

using namespace std;

typedef shared_ptr<MapPoint> MapPoint_;
typedef shared_ptr<KeyFrame> KeyFrame_;

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
        for (size_t i = 0; i < vMPs.size(); i++) {
            MapPoint_ pMP = vMPs[i];
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
            cv::Point2f uv = pKF->getKeyPoint(i).pt;
            int octave = pKF->getKeyPoint(i).octave;
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

    for(size_t i = 0; i < vMapPoints.size(); i++){
        MapPoint_ pMP = vMapPoints[i];

        if(!pMP){
            continue;
        }

        cv::Point2f uv = currFrame.getKeyPoint(i).pt;
        int octave = currFrame.getKeyPoint(i).octave;
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
        vVertex[i] = e;
        vInlier[i] = true;
    }

    for(int i = 0; i < 4; i++){
        vSE3->setEstimate(g2o::SE3Quat(fPose.unit_quaternion().cast<double>(),fPose.translation().cast<double>()));

        optimizer.initializeOptimization(0);
        optimizer.optimize(10);

        int nBad = 0;
        for(size_t j = 0; j < vVertex.size(); j++){
            EdgeSE3ProjectXYZOnlyPose* e = vVertex[j];
            if(!e)
                continue;

            if(!vInlier[i])
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

            if(i == 2){
                e->setRobustKernel(0);
            }
        }
    }

    int nGood = 0;
    for(int i = 0; i < vInlier.size(); i++){
        if(!vInlier[i]){
            currFrame.setMapPoint(i, nullptr);
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
        for (size_t i = 0; i < vMPs.size(); i++) {
            MapPoint_ pMP = vMPs[i];
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
            cv::Point2f uv = pKF->getKeyPoint(i).pt;
            int octave = pKF->getKeyPoint(i).octave;
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
            vMpInKf.push_back(i);

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
        for (size_t i = 0; i < vMPs.size(); i++) {
            MapPoint_ pMP = vMPs[i];
            if (!pMP) continue;
            if(sLocalMapPoints.count(pMP->getId()) == 0) continue;

            assert(mMapPointId.count(pMP) > 0);

            //Set edge
            cv::Point2f uv = pKF->getKeyPoint(i).pt;
            int octave = pKF->getKeyPoint(i).octave;
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
            vMpInKf.push_back(i);

            optimizer.addEdge(e);
        }
    }

    //Run optimization
    optimizer.initializeOptimization();
    optimizer.optimize(5);

    //Remove outliers
    for(size_t i = 0; i < vEdges.size(); i++){
        EdgeSE3ProjectXYZ* e = vEdges[i];

        if(e->chi2() > 5.991 || !e->isDepthPositive()){
            e->setLevel(1);
        }

        e->setRobustKernel(0);
    }

    optimizer.initializeOptimization(0);
    optimizer.optimize(10);

    //Remove outlier observations from map
    for(size_t i = 0; i < vEdges.size(); i++){
        EdgeSE3ProjectXYZ* e = vEdges[i];

        //Remove observation
        if(e->chi2() > 5.991 || !e->isDepthPositive()){
            //Remove from KeyFrame
            pMap->getKeyFrame(vEdgesKF[i])->setMapPoint(vMpInKf[i], nullptr);

            //Remove from map
            pMap->removeObservation(vEdgesKF[i],vEdgesMP[i]);

            pMap->checkKeyFrame(vEdgesKF[i]);
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


void arapOptimization(Map* pMap){
    unordered_map<KeyFrame_,size_t> mKeyFrameId;
    unordered_map<MapPoint_,size_t> mMapPointId;

    //Get all KeyFrames from map
    std::unordered_map<ID,KeyFrame_>&  mKeyFrames = pMap->getKeyFrames();
    

    // Create optimizer
    g2o::SparseOptimizer optimizer;

    std::unique_ptr<g2o::BlockSolverX::LinearSolverType> linearSolver =  g2o::make_unique<g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>>();

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
            g2o::make_unique<g2o::BlockSolverX>(std::move(linearSolver))
    );

    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(false);

    const float thHuber2D = sqrt(5.99);

    size_t currId = 0;

    //Set optimization

    // Add ARAP edges
    for (auto k1 = mKeyFrames.begin(); k1 != mKeyFrames.end(); ++k1) { // [DUDA] deberia hacerlo diferente?
        for (auto k2 = std::next(k1); k2 != mKeyFrames.end(); ++k2) {
            KeyFrame_ pKF1 = k1->second;
            KeyFrame_ pKF2 = k2->second;
            std::cout << "Pair: (" << k1->first << ", " << k2->first<< ")\n";

            //Check if this MapPoint has been already added to the optimization
            if (mKeyFrameId.count(pKF1) == 0) {
                //Set KeyFrame data
                Sophus::SE3f kfPose = pKF1->getPose();
                g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
                vSE3->setEstimate(g2o::SE3Quat(kfPose.unit_quaternion().cast<double>(),kfPose.translation().cast<double>()));
                vSE3->setId(currId);
                if(pKF1->getId()==0){
                    vSE3->setFixed(true);
                }
                optimizer.addVertex(vSE3);

                mKeyFrameId[pKF1] = currId;
                currId++;
            }

            if (mKeyFrameId.count(pKF2) == 0) {
                //Set KeyFrame data
                Sophus::SE3f kfPose = pKF2->getPose();
                g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
                vSE3->setEstimate(g2o::SE3Quat(kfPose.unit_quaternion().cast<double>(),kfPose.translation().cast<double>()));
                vSE3->setId(currId);
                if(pKF2->getId()==0){
                    vSE3->setFixed(true);
                }
                optimizer.addVertex(vSE3);

                mKeyFrameId[pKF2] = currId;
                currId++;
            }

            vector<MapPoint_>& v1MPs = pKF1->getMapPoints();
            vector<MapPoint_>& v2MPs = pKF2->getMapPoints(); // [DUDA] Deben darse como puntos 3D diferentes porque los optimizo como diferentes
            // [DUDA] Primero asumire que todos son diferentes, deberia fusionar una vez hecha la optimización si quedan muy juntos??

            for (size_t i = 0; i < v1MPs.size(); i++) {
                MapPoint_ pMPi1 = v1MPs[i];
                MapPoint_ pMPi2 = v2MPs[i];
                if (!pMPi1) continue;
                if (!pMPi2) continue;

                MapPoint_ firstPointToOptimize = pMPi1;
                MapPoint_ secondPointToOptimize = pMPi2;

                //Check if this MapPoint has been already added to the optimization
                if (mMapPointId.count(firstPointToOptimize) == 0) {
                    //EdgeSE3ProjectXYZ* vPoint = new EdgeSE3ProjectXYZ();
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
                    //EdgeSE3ProjectXYZ* vPoint = new EdgeSE3ProjectXYZ();
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

                //Set fisrt projection edge
                cv::Point2f uv = pKF1->getKeyPoint(i).pt;
                int octave = pKF1->getKeyPoint(i).octave;
                Eigen::Matrix<double,2,1> obs;
                obs << uv.x, uv.y;

                EdgeSE3ProjectXYZKeyFrame* eKF1 = new EdgeSE3ProjectXYZKeyFrame();

                eKF1->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(mMapPointId[firstPointToOptimize])));
                eKF1->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(mKeyFrameId[pKF1])));
                eKF1->setMeasurement(obs);
                eKF1->setInformation(Eigen::Matrix2d::Identity() * pKF1->getInvSigma2(octave));

                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                eKF1->setRobustKernel(rk);
                rk->setDelta(thHuber2D);

                eKF1->pCamera = pKF1->getCalibration();

                optimizer.addEdge(eKF1);

                //Set second projection edge
                uv = pKF2->getKeyPoint(i).pt;
                octave = pKF2->getKeyPoint(i).octave;
                obs << uv.x, uv.y;

                EdgeSE3ProjectXYZKeyFrame* eKF2 = new EdgeSE3ProjectXYZKeyFrame();

                eKF2->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(mMapPointId[secondPointToOptimize])));
                eKF2->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(mKeyFrameId[pKF2])));
                eKF2->setMeasurement(obs);
                eKF2->setInformation(Eigen::Matrix2d::Identity() * pKF2->getInvSigma2(octave));

                rk = new g2o::RobustKernelHuber;
                eKF2->setRobustKernel(rk);
                rk->setDelta(thHuber2D);

                eKF2->pCamera = pKF2->getCalibration();

                optimizer.addEdge(eKF2);

                for (size_t j = 0; j < v1MPs.size(); j++) { // [DUDA] Posible error si el segundo mapa es de otro size
                    MapPoint_ pMPj1 = v1MPs[j];
                    MapPoint_ pMPj2 = v2MPs[j];
                    if (!pMPj1) continue;
                    if (!pMPj2) continue;
                    if (pMPi1 == pMPj1 || pMPi2 == pMPj2) continue;

                    
                    MapPoint_ firstPointMeasurement = pMPj1;
                    MapPoint_ secondPointMeasurement = pMPj2;

                    //Set ARAP edge
                    EdgeARAP* eArap = new EdgeARAP();
                    eArap->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(mMapPointId[firstPointToOptimize])));
                    eArap->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(mMapPointId[secondPointToOptimize])));
                    eArap->Xj1world = firstPointMeasurement->getWorldPosition().cast<double>();
                    eArap->Xj2world = secondPointMeasurement->getWorldPosition().cast<double>();

                    // coger todas los errores en distancias por cada punto
                    //sacar la desv tipica en x, y, z
                    // invertirlo y usarlo
                    eArap->setInformation(Eigen::Matrix3d::Identity());// * pKF2->getInvSigma2(octave));// * pKF1->getARAPWeight(pMP1, pMP2));
                    eArap->setMeasurement(Eigen::Vector3d(0, 0, 0));


                    optimizer.addEdge(eArap);
                }
            }
        }
    }

    //Run optimization
    optimizer.initializeOptimization();

    std::cout << "Starting... \n";
    optimizer.optimize(5);
    // tras unas iteraciones con las soluciones o modificando lo que tienes

    std::cout << "Optimizado \n";

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
