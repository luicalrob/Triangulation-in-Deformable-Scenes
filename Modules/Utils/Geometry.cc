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
#include "libqhullcpp/PointCoordinates.h"
#include "libqhullcpp/Qhull.h"
#include "libqhullcpp/QhullFacet.h"
#include "libqhullcpp/QhullFacetList.h"
#include "libqhullcpp/QhullVertexSet.h"

#include <random> 

using namespace std;

float cosRayParallax(const Eigen::Vector3f& a, const Eigen::Vector3f& b){
    return a.dot(b)/(a.norm()*b.norm());
}

void triangulate(const Eigen::Vector3f &xn1, const Eigen::Vector3f &xn2,
                 const Sophus::SE3f &T1w, const Sophus::SE3f &T2w, Eigen::Vector3f &x3D){
    Sophus::SE3f T21 = T2w * T1w.inverse();
    Eigen::Vector3f m0 = T21.rotationMatrix() * xn1;
    Eigen::Vector3f m1 = xn2;

    Eigen::Vector3f t = T21.translation().normalized();
    Eigen::Matrix<float,3,2> M;
    M.col(0) = m0.normalized();
    M.col(1) = m1.normalized();

    Eigen::Matrix<float,2,3> A = M.transpose() * (Eigen::Matrix3f::Identity() - t*t.transpose());
    Eigen::JacobiSVD<Eigen::Matrix<float,2,3>> svd(A, Eigen::ComputeFullV);
    Eigen::Vector3f n = svd.matrixV().col(1);

    Eigen::Vector3f m0_ = m0 - (m0.dot(n)) * n;
    Eigen::Vector3f m1_ = m1 - (m1.dot(n)) * n;

    Eigen::Vector3f z = m1_.cross(m0_);
    float lambda0 = z.dot(T21.translation().cross(m1_))/(z.squaredNorm());
    Eigen::Vector3f p3D1 = T21.translation() + lambda0*m0_;

    float lambda1 = z.dot(T21.translation().cross(m0_))/(z.squaredNorm());
    Eigen::Vector3f test = lambda1 * m1_;

    x3D = T2w.inverse() * p3D1;
}

void triangulateInRays(const Eigen::Vector3f &xn1, const Eigen::Vector3f &xn2,
                 const Sophus::SE3f &T1w, const Sophus::SE3f &T2w, Eigen::Vector3f &x3D_1, Eigen::Vector3f &x3D_2){
    Sophus::SE3f T21 = T2w * T1w.inverse();
    Eigen::Vector3f m0 = T21.rotationMatrix() * xn1;
    Eigen::Vector3f m1 = xn2;

    Eigen::Vector3f t = T21.translation().normalized();
    Eigen::Matrix<float,3,2> M;
    M.col(0) = m0.normalized();
    M.col(1) = m1.normalized();

    Eigen::Matrix<float,2,3> A = M.transpose() * (Eigen::Matrix3f::Identity() - t*t.transpose());
    Eigen::JacobiSVD<Eigen::Matrix<float,2,3>> svd(A, Eigen::ComputeFullV);
    // std::cout << "A:" << A << "\n";
    Eigen::Vector3f n = svd.matrixV().col(1);

    Eigen::Vector3f m0_ = m0 - (m0.dot(n)) * n;
    Eigen::Vector3f m1_ = m1 - (m1.dot(n)) * n;

    Eigen::Vector3f z = m1_.cross(m0_);
    float lambda0 = z.dot(T21.translation().cross(m1_))/(z.squaredNorm());
    Eigen::Vector3f p3D1 = T21.translation() + lambda0*m0;

    float lambda1 = z.dot(T21.translation().cross(m0_))/(z.squaredNorm());
    Eigen::Vector3f p3D2 = lambda1 * m1;

    Eigen::Vector3f x3D_w = T2w.inverse() * p3D1;
    // std::cout << "x3D_1: x:" << x3D_w.x() << " y: " << x3D_w.y() << " z: " << x3D_w.z() << "\n";
    Eigen::Vector3f x3D_w_test = T2w.inverse() * p3D2;
    // std::cout << "x3D_2: x:" << x3D_w_test.x() << " y: " << x3D_w_test.y() << " z: " << x3D_w_test.z() << "\n";

    // point, rayOrigin, rayDir
    x3D_1 = x3D_w;
    x3D_2 = x3D_w_test;
    // x3D_1 = findClosestPointOnRay(x3D_w, T1w.translation(), T1w.rotationMatrix() * xn1);
    // x3D_2 = findClosestPointOnRay(x3D_w_test, T2w.translation(), T2w.rotationMatrix() * xn2);
}

void triangulateInRaysNearPrevSolution(const Eigen::Vector3f &xn1, const Eigen::Vector3f &xn2, const Sophus::SE3f &T1w, 
                        const Sophus::SE3f &T2w, Eigen::Vector3f &x3D_1, Eigen::Vector3f &x3D_2, Eigen::Vector3f &x3D_prev){
    Sophus::SE3f T21 = T2w * T1w.inverse();
    // Step 1: Transform previous 3D point into the local frames of the two cameras.
    Eigen::Vector3f x3D_prev_cam1 = T1w * x3D_prev; // Previous solution in camera 1 frame
    Eigen::Vector3f x3D_prev_cam2 = T2w * x3D_prev; // Previous solution in camera 2 frame

    // Step 2: Find the closest point on the first ray to the previous solution in camera 1 frame.
    float lambda1 = xn1.dot(x3D_prev_cam1); // Projection of x3D_prev_cam1 onto the direction of xn1
    x3D_1 = T1w.inverse() * (lambda1 * xn1);          // Closest point on ray 1 in world frame

    // Step 3: Find the closest point on the second ray to x3D_1.
    Eigen::Vector3f x3D_1_cam2 = T2w * x3D_1;  // Transform x3D_1 to camera 2 frame
    float lambda2 = xn2.dot(x3D_prev_cam2);                 // Projection of x3D_1_cam2 onto the direction of xn2
    x3D_2 = T2w.inverse() * (lambda2 * xn2);                       // Closest point on ray 2 in world frame
}

void triangulateTwoPoints(const Eigen::Vector3f &xn1, const Eigen::Vector3f &xn2,
                 const Sophus::SE3f &T1w, const Sophus::SE3f &T2w, Eigen::Vector3f &x3D_1, Eigen::Vector3f &x3D_2){
    Sophus::SE3f T21 = T2w * T1w.inverse();
    Eigen::Vector3f m0 = T21.rotationMatrix() * xn1;
    Eigen::Vector3f m1 = xn2;

    Eigen::Vector3f t = T21.translation().normalized();
    Eigen::Matrix<float,3,2> M;
    M.col(0) = m0.normalized();
    M.col(1) = m1.normalized();

    Eigen::Matrix<float,2,3> A = M.transpose() * (Eigen::Matrix3f::Identity() - t*t.transpose());
    Eigen::JacobiSVD<Eigen::Matrix<float,2,3>> svd(A, Eigen::ComputeFullV);
    //std::cout << "A:" << A << "\n";

    Eigen::Vector3f n = svd.matrixV().col(1);

    Eigen::Vector3f m0_ = m0 - (m0.dot(n)) * n;
    Eigen::Vector3f m1_ = m1 - (m1.dot(n)) * n;

    Eigen::Vector3f z = m1_.cross(m0_);
    float lambda0 = z.dot(T21.translation().cross(m1_))/(z.squaredNorm());
    Eigen::Vector3f p3D1 = T21.translation() + lambda0*m0_;

    float lambda1 = z.dot(T21.translation().cross(m0_))/(z.squaredNorm());
    Eigen::Vector3f p3D2 = lambda1 * m1_;

    x3D_1 = T2w.inverse() * p3D1;
    //std::cout << "x3D_1: x:" << x3D_1.x() << " y: " << x3D_1.y() << " z: " << x3D_1.z() << "\n";
    x3D_2 = T2w.inverse() * p3D2; // same 3D point
    //std::cout << "x3D_2: x:" << x3D_2.x() << " y: " << x3D_2.y() << " z: " << x3D_2.z() << "\n";
}

void triangulateBerkeley(const Eigen::Vector3f &xn1, const Eigen::Vector3f &xn2,
                        Frame &F1, Frame &F2,
                        Eigen::Vector3f& point1, Eigen::Vector3f& point2) {
    Eigen::Matrix3f K1 = F1.getCalibration()->getCalibrationMatrix();
    Eigen::Matrix3f K2 = F2.getCalibration()->getCalibrationMatrix();

    Sophus::SE3f T1w = F1.getPose();
    Sophus::SE3f T2w = F2.getPose();

    // Compute the projection matrices P1 and P2
    Eigen::Matrix3f R1w = T1w.rotationMatrix();
    Eigen::Vector3f t1w = T1w.translation();

    Eigen::Matrix3f R2w = T2w.rotationMatrix();
    Eigen::Vector3f t2w = T2w.translation();

    // Convert rotation matrices and translation vectors to cv::Mat
    cv::Mat Rcw1(3, 3, CV_32F, R1w.data());
    cv::Mat tcw1(3, 1, CV_32F, t1w.data());
    cv::Mat Tcw1(3, 4, CV_32F);
    Rcw1.copyTo(Tcw1.colRange(0, 3));
    tcw1.copyTo(Tcw1.col(3));

    cv::Mat Rcw2(3, 3, CV_32F, R2w.data());
    cv::Mat tcw2(3, 1, CV_32F, t2w.data());
    cv::Mat Tcw2(3, 4, CV_32F);
    Rcw2.copyTo(Tcw2.colRange(0, 3));
    tcw2.copyTo(Tcw2.col(3));

    cv::Mat xn1_cv(3, 1, CV_32F, const_cast<float*>(xn1.data()));
    cv::Mat xn2_cv(3, 1, CV_32F, const_cast<float*>(xn2.data()));

    cv::Mat A(4,4,CV_32F);
    A.row(0) = xn1_cv.at<float>(0)*Tcw1.row(2)-Tcw1.row(0);
    A.row(1) = xn1_cv.at<float>(1)*Tcw1.row(2)-Tcw1.row(1);
    A.row(2) = xn2_cv.at<float>(0)*Tcw2.row(2)-Tcw2.row(0);
    A.row(3) = xn2_cv.at<float>(1)*Tcw2.row(2)-Tcw2.row(1);

    // std::cout << "xn2:" << xn2 << "\n";
    // std::cout << "Rcw2:" << Rcw2 << "\n";
    // std::cout << "tcw2:" << tcw2 << "\n";
    // std::cout << "Tcw2:" << Tcw2 << "\n";

    // Compute the SVD of A
    cv::Mat x3D;
    cv::Mat w,u,vt;
    cv::SVD::compute(A,w,u,vt,cv::SVD::FULL_UV);

    x3D = vt.row(3).t();
    if(x3D.at<float>(3)==0)
    return;

    // Euclidean coordinates
    x3D = x3D.rowRange(0,3)/x3D.at<float>(3);
    // std::cout << "x3D:" << x3D << "\n";

    point1[0] = x3D.at<float>(0, 0); // x-coordinate
    point1[1] = x3D.at<float>(1, 0); // y-coordinate
    point1[2] = x3D.at<float>(2, 0); // z-coordinate
    point2 = point1;
}

float squaredReprojectionError(cv::Point2f &p1, cv::Point2f &p2){
    float errx = p1.x - p2.x;
    float erry = p1.y - p2.y;

    return errx * errx + erry *erry;
}

Eigen::Matrix<float,3,3> computeEssentialMatrixFromPose(Sophus::SE3f& T12){
    Eigen::Matrix<float,3,3> E;

    /*
     * Your code for Lab 4 - Task 2 here!
     */
    Eigen::Matrix<float,3,3> R = T12.rotationMatrix();
    Eigen::Matrix<float,3,1> t = T12.translation();

    Eigen::Matrix3f tx;
    tx << 0, -t[2], t[1],
        t[2], 0, -t[0],
        -t[1], t[0], 0;
    
    E = tx * R;

    return E;
}

std::vector<Eigen::Vector3d> extractPositions(const std::vector<std::shared_ptr<MapPoint>>& mapPoints) {
    std::vector<Eigen::Vector3d> positions;
    positions.reserve(mapPoints.size());

    for (const auto& mp : mapPoints) {
        if (!mp) continue;
        Eigen::Vector3d p3D = mp->getWorldPosition().cast<double>();
        positions.push_back(p3D);
    }

    return positions;
}

std::shared_ptr<open3d::geometry::PointCloud> convertToOpen3DPointCloud(const std::vector<Eigen::Vector3d>& positions) {
    auto pointCloud = std::make_shared<open3d::geometry::PointCloud>();
    
    for (const Eigen::Vector3d& pos : positions) {
        if (pos.isZero()) continue;
        pointCloud->points_.push_back(pos);
    }

    return pointCloud;
}

double cotangent(const Eigen::Vector3d &v0, const Eigen::Vector3d &v1, const Eigen::Vector3d &v2) {
    Eigen::Vector3d e0 = v1 - v0;
    Eigen::Vector3d e1 = v2 - v1;
    Eigen::Vector3d e2 = v0 - v2;

    double cosTheta = -e2.dot(e0) / (e2.norm() * e0.norm());
    return cosTheta / sqrt(1.0 - cosTheta * cosTheta);
}

std::unordered_map<Eigen::Vector2i, double, open3d::utility::hash_eigen<Eigen::Vector2i>> 
ComputeEdgeWeightsCot(
        std::shared_ptr<open3d::geometry::TriangleMesh> mesh,
        double min_weight) {
    std::unordered_map<Eigen::Vector2i, double,open3d::utility::hash_eigen<Eigen::Vector2i>> weights;
    auto edges_to_vertices = mesh->GetEdgeToVerticesMap();
    auto vertices = mesh->vertices_;
    for (const auto &edge_v2s : edges_to_vertices) {
        Eigen::Vector2i edge = edge_v2s.first;
        double weight_sum = 0;
        int N = 0;
        for (int v2 : edge_v2s.second) {
            Eigen::Vector3d a = vertices[edge(0)] - vertices[v2];
            Eigen::Vector3d b = vertices[edge(1)] - vertices[v2];
            double weight = a.dot(b) / (a.cross(b)).norm();
            weight_sum += weight;
            N++;
        }
        double weight = N > 0 ? weight_sum / N : 0;
        if (weight < min_weight) {
            weights[edge] = min_weight;
        } else {
            weights[edge] = weight;
        }
    }
    return weights;
}

std::map<size_t, size_t> createVectorMap(const std::vector<Eigen::Vector3d>& vertices, const std::vector<Eigen::Vector3d>& positions, double precision) {
    std::map<size_t, size_t> indexMap;

    for (size_t vertexIdx = 0; vertexIdx < vertices.size(); ++vertexIdx) {
        const Eigen::Vector3d& vertex = vertices[vertexIdx];
        for (size_t positionIdx = 0; positionIdx < positions.size(); ++positionIdx) {
            const Eigen::Vector3d& position = positions[positionIdx];
            if (vertex.isApprox(position, precision)) {
                indexMap[vertexIdx] = positionIdx;
                break;
            }
        }
    }

    return indexMap;
}

std::shared_ptr<open3d::geometry::TriangleMesh> ComputeDelaunayTriangulation3D(
        const std::vector<Eigen::Vector3d> &points) {
    auto triangle_mesh = std::make_shared<open3d::geometry::TriangleMesh>();

    if (points.size() < 3) {
        open3d::utility::LogError("Not enough points to create a triangular mesh.");
        return nullptr;
    }

    std::vector<double> qhull_points_data(points.size() * 2);
    for (size_t pidx = 0; pidx < points.size(); ++pidx) {
        const auto &pt = points[pidx];
        qhull_points_data[pidx * 2 + 0] = pt(0);
        qhull_points_data[pidx * 2 + 1] = pt(1);
    }

    orgQhull::PointCoordinates qhull_points(2, "");
    qhull_points.append(qhull_points_data);

    orgQhull::Qhull qhull;
    qhull.runQhull(qhull_points.comment().c_str(), qhull_points.dimension(),
                   qhull_points.count(), qhull_points.coordinates(),
                   "d Qbb Qt");

    orgQhull::QhullFacetList facets = qhull.facetList();
    triangle_mesh->vertices_ = points; // Keep the original 3D points
    triangle_mesh->triangles_.resize(facets.count());

    int tidx = 0;
    for (orgQhull::QhullFacetList::iterator it = facets.begin();
         it != facets.end(); ++it) {
        if (!(*it).isGood()) continue;

        orgQhull::QhullFacet f = *it;
        orgQhull::QhullVertexSet vSet = f.vertices();
        std::array<int, 3> triangle_indices;
        int tri_subscript = 0;
        for (orgQhull::QhullVertexSet::iterator vIt = vSet.begin();
             vIt != vSet.end(); ++vIt) {
            orgQhull::QhullVertex v = *vIt;
            orgQhull::QhullPoint p = v.point();

            int vidx = p.id();
            triangle_indices[tri_subscript] = vidx;
            tri_subscript++;
        }
        triangle_mesh->triangles_[tidx] = Eigen::Vector3i(triangle_indices[0], triangle_indices[1], triangle_indices[2]);
        tidx++;
    }

    return triangle_mesh;
}

Eigen::Vector3f findClosestPointOnRay(const Eigen::Vector3f &point, const Eigen::Vector3f &rayOrigin, const Eigen::Vector3f &rayDir) {
    Eigen::Vector3f originToPoint = point - rayOrigin;
    
    // Calculate the projection of the originToPoint vector onto the ray direction vector
    float t = rayDir.dot(originToPoint) / rayDir.dot(rayDir);
    
    // Ensure that t >= 0, since we're only interested in points on the ray (not behind the origin)
    if (t < 0.0f) {
        return rayOrigin;
    }
    
    Eigen::Vector3f closestPoint = rayOrigin + t * rayDir;
    
    return closestPoint;
}


double calculatePixelsStandDev(std::shared_ptr<Map> Map, cameraSelection cameraSelection){
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

    std::unordered_map<ID,KeyFrame_>&  mKeyFrames = Map.get()->getKeyFrames();

    //std::cout << "\nKEYFRAMES k AND k+1 MEASUREMENTS: \n";
    for (auto k1 = mKeyFrames.begin(); k1 != mKeyFrames.end(); ++k1) {
        for (auto k2 = std::next(k1); k2 != mKeyFrames.end(); ++k2) {
            KeyFrame_ pKF1 = k2->second;
            KeyFrame_ pKF2 = k1->second;
            //std::cout << "Pair: (" << k1->first << ", " << k2->first<< ")\n";

            vector<MapPoint_>& v1MPs = pKF1->getMapPoints();
            vector<MapPoint_>& v2MPs = pKF2->getMapPoints();

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
                meanRepErrorUVC1 += pixelsError.cwiseAbs();
                meanRepErrorUV += pixelsError.cwiseAbs();

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

                nMatches++;
            }

            if (nMatches > 0) {
                meanRepErrorUVC1 /= static_cast<double>(nMatches);
                meanRepErrorUVC2 /= static_cast<double>(nMatches);
                meanRepErrorUV /= static_cast<double>(2 * nMatches);

                meanRepErrorC1 = (meanRepErrorUVC1[0] + meanRepErrorUVC1[1]) / 2.0;
                meanRepErrorC2 = (meanRepErrorUVC2[0] + meanRepErrorUVC2[1]) / 2.0;
                meanRepError = (meanRepErrorUV[0] + meanRepErrorUV[1]) / 2.0;

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
                } else {
                std::cout << "No points to compare." << std::endl;
            }
        }
    }

    // std::cout << "desvRepErrorC1: " << desvRepErrorC1 << "\n";
    // std::cout << "desvRepErrorC2: " << desvRepErrorC2 << "\n";
    double error = 0.0;
    switch (cameraSelection) {
        case cameraSelection::C1:
            error = desvRepErrorC1;
            break;
        case cameraSelection::C2:
            error = desvRepErrorC2;
            break;
        case cameraSelection::Combined:
            error = desvRepError;
            break;
    }

    return error;
}