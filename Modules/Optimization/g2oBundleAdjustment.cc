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
#include "open3d/Open3D.h"
#include "open3d/geometry/Qhull.h"
#include "open3d/geometry/TetraMesh.h"

#include <random> 


using namespace std;

typedef shared_ptr<MapPoint> MapPoint_;
typedef shared_ptr<KeyFrame> KeyFrame_;
typedef shared_ptr<Eigen::Matrix3d> RotationMatrix_;
typedef shared_ptr<Eigen::Vector3d> TranslationVector_;

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

void arapOptimization(Map* pMap, float repBalanceWeight, float arapBalanceWeight, int nOptIterations){
    unordered_map<KeyFrame_,size_t> mKeyFrameId;
    unordered_map<MapPoint_,size_t> mMapPointId;
    unordered_map<RotationMatrix_,size_t> mRotId;
    unordered_map<RotationMatrix_,size_t> mRotGlobalId;
    unordered_map<TranslationVector_,size_t> mTransId;

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
            KeyFrame_ pKF1 = k2->second;
            KeyFrame_ pKF2 = k1->second;
            // std::cout << "\nPair: (" << k1->first << ", " << k2->first<< ")\n";

            vector<MapPoint_>& v1MPs = pKF1->getMapPoints();
            vector<MapPoint_>& v2MPs = pKF2->getMapPoints(); // [DUDA] Deben darse como puntos 3D diferentes porque los optimizo como diferentes
            // [DUDA] Primero asumire que todos son diferentes, deberia fusionar una vez hecha la optimización si quedan muy juntos??
            

            // MESH CREATION
            std::vector<Eigen::Vector3d> v1Positions = extractPositions(v1MPs);
            std::vector<Eigen::Vector3d> v2Positions = extractPositions(v2MPs);

            // OPEN3D LIBRARY
            std::shared_ptr<open3d::geometry::TriangleMesh> mesh1;

            std::vector<Eigen::Matrix3d> Rs(v1Positions.size(), Eigen::Matrix3d::Identity());
            Eigen::Matrix3d Rs_global = Eigen::Matrix3d::Identity();
            Eigen::Vector3d Ts = Eigen::Vector3d::Zero();
            std::pair<Eigen::Matrix3d, Eigen::Vector3d> transformation = pMap->getGlobalKeyFramesTransformation(k2->first, k1->first);

            Rs_global = transformation.first;
            Ts = transformation.second;

            // Perform Delaunay Reconstruction
            mesh1 = ComputeDelaunayTriangulation3D(v1Positions);
            // mesh2 = ComputeDelaunayTriangulation3D(v2Positions);

            auto posIndexes1 = createVectorMap(mesh1->vertices_, v1Positions);
            // auto indexMap2 = createVectorMap(mesh2->vertices_, v2Positions);
            // std::cout << "point mesh: (" << mesh1->vertices_[67] << ")\n";
            // std::cout << "point: (" << v1Positions[posIndexes1[67]] << ")\n";
            // std::cout << "index: (" << posIndexes1[67] << ")\n";
            // std::cout << "mesh size 1: (" << mesh1->vertices_.size() << ")\n";

            std::map<size_t, size_t> invertedPosIndexes1;
            // std::map<size_t, size_t> invertedIndexMap2;
            for (const auto& pair : posIndexes1) {
                invertedPosIndexes1[pair.second] = pair.first;
            } 

            // Visualize OPEN3D the meshes
            // open3d::visualization::Visualizer visualizer;
            // visualizer.CreateVisualizerWindow("Mesh Visualization");
            // Eigen::Vector3d red(1.0, 0.0, 0.0);
            // mesh1->vertex_colors_.resize(mesh1->vertices_.size(), red);
            // visualizer.AddGeometry(mesh1);
            // visualizer.Run();
            // visualizer.DestroyVisualizerWindow();

            mesh1->ComputeAdjacencyList();
            auto edge_weights_1 = ComputeEdgeWeightsCot(mesh1, 0);

            RotationMatrix_ Rot_global = std::make_shared<Eigen::Matrix<double, 3, 3>>(Rs_global);
            TranslationVector_ T = std::make_shared<Eigen::Matrix<double, 3, 1>>(Ts);

            VertexTranslationVector* tVertex = new VertexTranslationVector();
            tVertex->setId(currId); // unique ID
            tVertex->setEstimate(Ts);
            optimizer.addVertex(tVertex);
            mTransId[T] = currId;
            currId++;

            VertexRotationMatrix* rotVertexGlobal = new VertexRotationMatrix();
            rotVertexGlobal->setId(currId); // unique ID
            rotVertexGlobal->setEstimate(Rs_global);
            optimizer.addVertex(rotVertexGlobal);
            mRotGlobalId[Rot_global] = currId;
            currId++;

            for (size_t i = 0; i < v1MPs.size(); i++) {
                MapPoint_ pMPi1 = v1MPs[i];
                MapPoint_ pMPi2 = v2MPs[i];
                if (!pMPi1) continue;
                if (!pMPi2) continue;

                MapPoint_ firstPointToOptimize = pMPi1;
                RotationMatrix_ Rot = std::make_shared<Eigen::Matrix<double, 3, 3>>(Rs[i]);
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

                    VertexRotationMatrix* rotVertex = new VertexRotationMatrix();
                    rotVertex->setId(currId); // unique ID
                    rotVertex->setEstimate(Rs[i]);
                    optimizer.addVertex(rotVertex);
                    mRotId[Rot] = currId;
                    currId++;
                }

                //Set fisrt projection edge
                cv::Point2f uv = pKF1->getKeyPoint(i).pt;
                int octave = pKF1->getKeyPoint(i).octave;
                Eigen::Matrix<double,2,1> obs;
                // obs << uv.x, uv.y;

                EdgeSE3ProjectXYZPerKeyFrameOnlyPoints* eKF1 = new EdgeSE3ProjectXYZPerKeyFrameOnlyPoints();

                eKF1->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(mMapPointId[firstPointToOptimize])));
                eKF1->setMeasurement(obs);
                eKF1->setInformation(Eigen::Matrix2d::Identity() * pKF1->getInvSigma2(octave) * repBalanceWeight);

                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                eKF1->setRobustKernel(rk);
                rk->setDelta(thHuber2D);

                eKF1->pCamera = pKF1->getCalibration();
                Sophus::SE3f kfPose = pKF1->getPose();
                eKF1->cameraPose = g2o::SE3Quat(kfPose.unit_quaternion().cast<double>(),kfPose.translation().cast<double>());

                optimizer.addEdge(eKF1);

                //Set second projection edge
                uv = pKF2->getKeyPoint(i).pt;
                octave = pKF2->getKeyPoint(i).octave;
                obs << uv.x, uv.y;

                EdgeSE3ProjectXYZPerKeyFrameOnlyPoints* eKF2 = new EdgeSE3ProjectXYZPerKeyFrameOnlyPoints();

                eKF2->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(mMapPointId[secondPointToOptimize])));
                eKF2->setMeasurement(obs);
                eKF2->setInformation(Eigen::Matrix2d::Identity() * pKF2->getInvSigma2(octave) * repBalanceWeight);

                rk = new g2o::RobustKernelHuber;
                eKF2->setRobustKernel(rk);
                rk->setDelta(thHuber2D);

                eKF2->pCamera = pKF2->getCalibration();
                kfPose = pKF2->getPose();
                eKF2->cameraPose = g2o::SE3Quat(kfPose.unit_quaternion().cast<double>(),kfPose.translation().cast<double>());

                optimizer.addEdge(eKF2);

                auto it1 = invertedPosIndexes1.find(i);
                size_t meshIndex1 = 0;
                if (it1 != invertedPosIndexes1.end()) {
                    meshIndex1 = it1->second;
                    // std::cout << "v1Position: (" << v1Positions[i][0] << ", " << v1Positions[i][1] << ", " << v1Positions[i][2] << ")\n";
                    // std::cout << "v2Position: (" << v2Positions[i][0] << ", " << v2Positions[i][1] << ", " << v2Positions[i][2] << ")\n";
                    // std::cout << "point mesh: (" << mesh1->vertices_[meshIndex1][0] << ", " << mesh1->vertices_[meshIndex1][1] << ", " << mesh1->vertices_[meshIndex1][2] << ")\n";
                    // std::cout << "index: (" << i << ")\n";
                } else {
                    continue;
                }

                                
                if (mesh1->adjacency_list_[meshIndex1].empty()) continue;

                // std::cout << "adjacency_list_ 1: (" << mesh1->adjacency_list_[meshIndex1].size() << ")\n";

                Eigen::Vector3d distancesInvTipDesv;
                // distancesInvTipDesv = getInvUncertainty(*firstPointToOptimize, *secondPointToOptimize, *pKF1, *pKF2);
                std::unordered_set<int> jIndexes = mesh1->adjacency_list_[meshIndex1];
                distancesInvTipDesv = getInvUncertainty(i, jIndexes, posIndexes1, v1Positions, v2Positions);
                //distancesInvTipDesv << 1.0 / 10.0, 1.0 / 10.0, 1.0 / 10.0;

                // Eigen::Matrix3d informationMatrix = distancesInvTipDesv.asDiagonal() * arapBalanceWeight;
                
                double scalarInformation = distancesInvTipDesv.mean(); // or use another method to combine the values
                Eigen::Matrix<double, 1, 1> informationMatrix;
                informationMatrix(0, 0) = scalarInformation * arapBalanceWeight;


                for (int j : mesh1->adjacency_list_[meshIndex1]) {
                    // std::cout << "adjacency_list_ 1: (" << j << ")\n";
                    //Set ARAP edge
                    EdgeARAP* eArap = new EdgeARAP();
                    // NORMAL
                    // eArap->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(mMapPointId[firstPointToOptimize])));
                    // eArap->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(mMapPointId[secondPointToOptimize])));
                    // eArap->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(mRotId[Rot])));
                    // eArap->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(mTransId[T])));
                    // eArap->Xj1world = mesh1->vertices_[j];
                    // eArap->Xj2world = v2Positions[posIndexes1[j]];
                    // eArap->weight = edge_weights_1[GetOrderedEdge(meshIndex1, j)];

                    // ONLY ONE POINT AND R
                    eArap->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(mMapPointId[firstPointToOptimize])));
                    eArap->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(mMapPointId[secondPointToOptimize])));
                    eArap->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(mRotId[Rot])));
                    eArap->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(mTransId[T])));
                    eArap->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(mRotGlobalId[Rot_global])));
                    //eArap->Xi1world = mesh1->vertices_[meshIndex1]; // or v1Positions[i] or v1Positions[posIndexes1[meshIndex1]]
                    eArap->Xj1world = mesh1->vertices_[j];
                    eArap->Xj2world = v2Positions[posIndexes1[j]];
                    eArap->weight = edge_weights_1[GetOrderedEdge(meshIndex1, j)];

                    eArap->setInformation(informationMatrix);
                    // eArap->setMeasurement(Eigen::Vector3d(0, 0, 0));
                    double measurement = 0.0;
                    eArap->setMeasurement(measurement);

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

    for(pair<RotationMatrix_,ID> pairRotationMatrixId : mRotId){
        RotationMatrix_ Rots = pairRotationMatrixId.first;
        VertexRotationMatrix* mRot = static_cast<VertexRotationMatrix*>(optimizer.vertex(pairRotationMatrixId.second));
        Eigen::Matrix3d Rotation = mRot->estimate();
        // std::cout << "Rotation matrix:\n" << Rotation << std::endl;
    }

    Eigen::Matrix3d RotGlobal = Eigen::Matrix3d::Identity();
    Eigen::Vector3d TGlobal = Eigen::Vector3d::Zero();

    for(pair<RotationMatrix_,ID> pairRotationGlobalMatrixId : mRotGlobalId){
        RotationMatrix_ Rots = pairRotationGlobalMatrixId.first;
        VertexRotationMatrix* mRot = static_cast<VertexRotationMatrix*>(optimizer.vertex(pairRotationGlobalMatrixId.second));
        RotGlobal = mRot->estimate();
        //std::cout << "Global Rotation matrix:\n" << Rotation << std::endl;
    }

    for(pair<TranslationVector_,ID> pairTranslationVectorId : mTransId){
        TranslationVector_ Ts = pairTranslationVectorId.first;
        VertexTranslationVector* mT = static_cast<VertexTranslationVector*>(optimizer.vertex(pairTranslationVectorId.second));
        TGlobal = mT->estimate();
        //std::cout << "Global translation vector:\n" << t << std::endl;
    }
    pMap->insertGlobalKeyFramesTransformation(0, 1, RotGlobal, TGlobal);
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

            std::shared_ptr<open3d::geometry::TriangleMesh> mesh1;

            // Perform Delaunay Reconstruction
            mesh1 = ComputeDelaunayTriangulation3D(v1Positions);
            mesh1->ComputeAdjacencyList();

            auto posIndexes1 = createVectorMap(mesh1->vertices_, v1Positions);
            std::cout << "mesh size 1: (" << mesh1->vertices_.size() << ")\n";

            std::map<size_t, size_t> invertedPosIndexes1;
            for (const auto& pair : posIndexes1) {
                invertedPosIndexes1[pair.second] = pair.first;
            } 

            std::vector<int> constraint_vertex_indices(10);  // Initialize vector with 200 elements

            // for (int i = 0; i < 10; ++i) {
            //     //constraint_vertex_indices[i] = distrib(gen);  // Assign values from 0 to 199
            //     constraint_vertex_indices[i] = i;
            // }

            std::shared_ptr<open3d::geometry::TriangleMesh> deformed_mesh = mesh1->DeformAsRigidAsPossible(
                constraint_vertex_indices, v2Positions, 500,  // max_iter = 50
                open3d::geometry::TriangleMesh::DeformAsRigidAsPossibleEnergy::Spokes, 0.01
            );
            // std::shared_ptr<open3d::geometry::TriangleMesh> deformed_mesh = mesh1->DeformAsRigidAsPossible();

            for (size_t i = 0; i < v1MPs.size(); i++) {
                MapPoint_ pMPi1 = v1MPs[i];
                MapPoint_ pMPi2 = v2MPs[i];

                auto it1 = invertedPosIndexes1.find(i);
                size_t meshIndex1 = 0;
                if (it1 != invertedPosIndexes1.end()) {
                    meshIndex1 = it1->second;
                    // std::cout << "v1Position: (" << v1Positions[i][0] << ", " << v1Positions[i][1] << ", " << v1Positions[i][2] << ")\n";
                    // std::cout << "point mesh: (" << mesh1->vertices_[meshIndex1][0] << ", " << mesh1->vertices_[meshIndex1][1] << ", " << mesh1->vertices_[meshIndex1][2] << ")\n";
                    // std::cout << "index: (" << i << ")\n";
                } else {
                    continue;
                }

                Eigen::Vector3f p3D1 = mesh1->vertices_[meshIndex1].cast<float>();
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
            mesh1 = ComputeDelaunayTriangulation3D(v1Positions);
            mesh1->ComputeAdjacencyList();

            posIndexes1 = createVectorMap(mesh1->vertices_, v1Positions);

            for (const auto& pair : posIndexes1) {
                invertedPosIndexes1[pair.second] = pair.first;
            } 
        }
    }
}

Eigen::Vector3d getInvUncertainty(int i, std::unordered_set<int> adjacencyList, std::map<size_t, size_t> posIndexes, std::vector<Eigen::Vector3d> v1Positions, std::vector<Eigen::Vector3d> v2Positions){

    std::vector<Eigen::Vector3d> errors;
    Eigen::Vector3d meanError = Eigen::Vector3d::Zero();
    size_t validPairs = 0;

    for (int j : adjacencyList) {
        Eigen::Vector3d pi1 = v1Positions[i];
        Eigen::Vector3d pj1 = v1Positions[posIndexes[j]];
        Eigen::Vector3d pi2 = v2Positions[i];
        Eigen::Vector3d pj2 = v2Positions[posIndexes[j]];

        if (!pi1.allFinite() || !pj1.allFinite() || !pi2.allFinite() || !pj2.allFinite()) continue;

        Eigen::Vector3d d1 = pi1 - pj1;
        Eigen::Vector3d d2 = pi2 - pj2;

        Eigen::Vector3d diff = d1 - d2;
        
        errors.push_back(diff);      
        meanError += diff;
        validPairs++;
    }

    if (validPairs > 1) {
        meanError /= static_cast<double>(validPairs);

        // Calculate the sum of the differences squared
        Eigen::Vector3d sumSquaredDiffs = Eigen::Vector3d::Zero();
        for (const Eigen::Vector3d& error : errors) {
            Eigen::Vector3d diffFromMean = error - meanError;
            sumSquaredDiffs += diffFromMean.cwiseProduct(diffFromMean); // (e_i - e_mean)^2
        }

        Eigen::Vector3d variance = sumSquaredDiffs / static_cast<double>(validPairs - 1);

        Eigen::Vector3d stdDev = variance.cwiseSqrt();
        
        return stdDev.cwiseInverse();
    } else {
        return Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
    }
        size_t meshIndex1 = 0;
}