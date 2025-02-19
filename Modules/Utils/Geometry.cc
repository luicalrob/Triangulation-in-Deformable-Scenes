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
#include "Utils/Conversions.h"
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

void triangulateClassic(const Eigen::Vector3f &xn1, const Eigen::Vector3f &xn2,
                 const Sophus::SE3f &T1w, const Sophus::SE3f &T2w, Eigen::Vector3f &x3D_1, 
                 Eigen::Vector3f &x3D_2, std::string location){
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
    float lambda1 = z.dot(T21.translation().cross(m0_))/(z.squaredNorm());

    // point, rayOrigin, rayDir
    Eigen::Vector3f p3D1, p3D2;

    if(location == "TwoPoints"){
        p3D1 = T21.translation() + lambda0*m0_;
        p3D2 = T21.translation() + lambda0*m0_;
        //p3D2 = lambda1 * m1_;
    } else {
        p3D1 = T21.translation() + lambda0*m0;
        p3D2 = lambda1 * m1;
    }

    // point, rayOrigin, rayDir
    x3D_1 = T2w.inverse() * p3D1;
    x3D_2 = T2w.inverse() * p3D2;
}

void triangulateNRSLAM(const Eigen::Vector3f& xn1, const Eigen::Vector3f& xn2,
                 const Sophus::SE3f& T1w, const Sophus::SE3f& T2w, Eigen::Vector3f& x3D_1, 
                 Eigen::Vector3f& x3D_2, std::string location) {
    Eigen::Vector3f f0 = xn1;
    Eigen::Vector3f f1 = xn2;

    Eigen::Vector3f f0_hat = xn1.normalized();
    Eigen::Vector3f f1_hat = xn2.normalized();

    Sophus::SE3f T21 = T2w * T1w.inverse();
    Eigen::Vector3f t = T21.translation();
    Eigen::Matrix3f R = T21.rotationMatrix();

    Eigen::Vector3f p = (R * f0_hat).cross(f1_hat);
    Eigen::Vector3f q = (R * f0_hat).cross(t);
    Eigen::Vector3f r = f1_hat.cross(t);

    float lambda0 = r.norm() / p.norm();
    float lambda1 = q.norm() / p.norm();

    // Adequacy test.
    Eigen::Vector3f point0 = lambda0 * R * f0_hat;
    Eigen::Vector3f point1 = lambda1 * f1_hat;

    float v1 = (t + point0 + point1).squaredNorm();
    float v2 = (t - point0 - point1).squaredNorm();
    float v3 = (t - point0 + point1).squaredNorm();

    float minv = fmin(v1,fmin(v2,v3));

    // Inverse Depth Weighted MidPoint.
    Eigen::Vector3f x1 = q.norm() / (q.norm() + r.norm()) * (t + r.norm() / p.norm() * (R * f0_hat + f1_hat));

    Eigen::Vector3f p3D1, p3D2;

    if(location == "TwoPoints"){
        p3D1 = x1;
        p3D2 = x1;
    } else if(location == "FarPoints") {
        point0 = (t + point0);

        p3D1 = point0 + (point0 - x1);
        p3D2 = point1 + (point1 - x1);
    } else {
        p3D1 = (t + point0);
        p3D2 = point1;
    }
    
    x3D_1 = T2w.inverse() * p3D1;
    x3D_2 = T2w.inverse() * p3D2;
}

void triangulateORBSLAM(const Eigen::Vector3f& xn1, const Eigen::Vector3f& xn2,
                        const Sophus::SE3f& Tcw1, const Sophus::SE3f& Tcw2,
                        Eigen::Vector3f& x3D_1, Eigen::Vector3f& x3D_2, std::string location) {
    cv::Mat Tcw1_cv = convertSE3fToMat(Tcw1);
    cv::Mat Tcw2_cv = convertSE3fToMat(Tcw2);
    cv::Mat xn1_cv = convertVector3fToMat(xn1);
    cv::Mat xn2_cv = convertVector3fToMat(xn2);

    cv::Mat A(4, 4, CV_32F);
    
    A.row(0) = xn1_cv.at<float>(0) * Tcw1_cv.row(2) - Tcw1_cv.row(0);
    A.row(1) = xn1_cv.at<float>(1) * Tcw1_cv.row(2) - Tcw1_cv.row(1);
    A.row(2) = xn2_cv.at<float>(0) * Tcw2_cv.row(2) - Tcw2_cv.row(0);
    A.row(3) = xn2_cv.at<float>(1) * Tcw2_cv.row(2) - Tcw2_cv.row(1);

    cv::Mat w,u,vt;
    cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);

    cv::Mat x3D = vt.row(3).t();
    
    Eigen::Vector3f point1, point2;

    if (x3D.at<float>(3) != 0) {
        cv::Mat x3D_dehomog = x3D.rowRange(0, 3) / x3D.at<float>(3);

        point1 = convertMatToVector3f(x3D_dehomog);
        point2 = convertMatToVector3f(x3D_dehomog);
    } else {
        point1.setZero();
        point2.setZero();
    }
}


void triangulateDepth(const Eigen::Vector3f& xn1, const Eigen::Vector3f& xn2, const Sophus::SE3f& T1w, 
                    const Sophus::SE3f& T2w, Eigen::Vector3f& x3D_1, Eigen::Vector3f& x3D_2, std::string location) {
    Sophus::SE3f T21 = T2w * T1w.inverse();
    Eigen::Vector3f t = T21.translation();
    Eigen::Matrix3f R = T21.rotationMatrix();

    Eigen::Vector3f point0 = T21 * xn1; 
    Eigen::Vector3f point1 = xn2;
    Eigen::Vector3f x1 = (point0 + point1) / 2.0f;

    Eigen::Vector3f p3D1, p3D2;

    if(location == "TwoPoints"){
        p3D1 = x1;
        p3D2 = x1;
    } else if(location == "FarPoints") {
        p3D1 = point0 + (point0 - x1);
        p3D2 = point1 + (point1 - x1);
    } else {
        p3D1 = point0;
        p3D2 = point1;
    }

    x3D_1 = T2w.inverse() * p3D1;
    x3D_2 = T2w.inverse() * p3D2;
}

bool useTriangulationMethod(const Eigen::Vector3f& xn1, const Eigen::Vector3f& xn2, 
                                const Sophus::SE3f& T1w, const Sophus::SE3f& T2w, 
                                Eigen::Vector3f& x3D_1, Eigen::Vector3f& x3D_2,
                                std::string TrianMethod, std::string TrianLocation) {
    if (TrianMethod == "Classic") {
        triangulateClassic(xn1, xn2, T1w, T2w, x3D_1, x3D_2, TrianLocation);
    } else if (TrianMethod == "ORBSLAM") {
        triangulateORBSLAM(xn1, xn2, T1w, T2w, x3D_1, x3D_2, TrianLocation);
    } else if (TrianMethod == "DepthMeasurement") {
        triangulateDepth(xn1, xn2, T1w, T2w, x3D_1, x3D_2, TrianLocation);
    } else {
        triangulateNRSLAM(xn1, xn2, T1w, T2w, x3D_1, x3D_2, TrianLocation);
    }
    return true;
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

    positions.shrink_to_fit();
    return positions;
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

void calculatePixelsStandDev(std::shared_ptr<Map> Map, PixelsError& pixelsErrors){
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

    std::unordered_map<ID,KeyFrame_>&  mKeyFrames = Map->getKeyFrames();

    //std::cout << "\nKEYFRAMES k AND k+1 MEASUREMENTS: \n";
    for (auto k1 = mKeyFrames.begin(); k1 != mKeyFrames.end(); ++k1) {
        for (auto k2 = std::next(k1); k2 != mKeyFrames.end(); ++k2) {
            KeyFrame_ pKF1 = k2->second;
            KeyFrame_ pKF2 = k1->second;
            int kf1ID = k2->first;
            int kf2ID = k1->first;

            vector<MapPoint_>& v1MPs = pKF1->getMapPoints();
            vector<MapPoint_>& v2MPs = pKF2->getMapPoints();

            std::shared_ptr<CameraModel> pCamera1 = pKF1->getCalibration();
            g2o::SE3Quat camera1Pose = g2o::SE3Quat(pKF1->getPose().unit_quaternion().cast<double>(),pKF1->getPose().translation().cast<double>());
            std::shared_ptr<CameraModel> pCamera2 = pKF2->getCalibration();
            g2o::SE3Quat camera2Pose = g2o::SE3Quat(pKF2->getPose().unit_quaternion().cast<double>(),pKF2->getPose().translation().cast<double>());
            Sophus::SE3f T1w = pKF1->getPose();
            Sophus::SE3f T2w = pKF2->getPose();

            // calculate the sum of squared differences from the mean for standard deviation
            Eigen::Vector2d sumSquaredDifferencesUVC1 = Eigen::Vector2d::Zero();
            Eigen::Vector2d sumSquaredDifferencesUVC2 = Eigen::Vector2d::Zero();
            Eigen::Vector2d sumSquaredDifferencesUV = Eigen::Vector2d::Zero();

            for (size_t i = 0; i < v1MPs.size(); i++) {
                MapPoint_ pMPi1 = v1MPs[i];
                MapPoint_ pMPi2 = v2MPs[i];
                if (!pMPi1 || !pMPi2) continue;

                int index_in_kf1 = Map->isMapPointInKeyFrame(pMPi1->getId(), kf1ID);
                int index_in_kf2 = Map->isMapPointInKeyFrame(pMPi2->getId(), kf2ID);

                if(index_in_kf1 < 0 || index_in_kf2 < 0) continue;

                size_t idx1 = (size_t)index_in_kf1;
                size_t idx2 = (size_t)index_in_kf2;

                cv::Point2f uv = pKF1->getKeyPoint(idx1).pt;
                
                Eigen::Vector2d obs;
                obs << uv.x, uv.y;

                Eigen::Vector3f p3Dw = pMPi1->getWorldPosition();
                Eigen::Matrix<float, 1, 4> p3Dw_h;
                p3Dw_h << p3Dw[0], p3Dw[1], p3Dw[2], 1;
                Eigen::Vector4f p3Dc_h =  T1w.matrix() * p3Dw_h.transpose();
                Eigen::Vector3f p3Dc;
                p3Dc << p3Dc_h[0], p3Dc_h[1], p3Dc_h[2];
                Eigen::Vector2f projected;
                pCamera1->project(p3Dc, projected);

                Eigen::Vector2d pixelsErrorC1 = (obs - projected.cast<double>()).cwiseAbs();

                uv = pKF2->getKeyPoint(idx2).pt;

                obs << uv.x, uv.y;
                p3Dw = pMPi2->getWorldPosition();
                p3Dw_h << p3Dw[0], p3Dw[1], p3Dw[2], 1;
                p3Dc_h =  T2w.matrix() * p3Dw_h.transpose();
                p3Dc << p3Dc_h[0], p3Dc_h[1], p3Dc_h[2];
                pCamera2->project(p3Dc, projected);

                Eigen::Vector2d pixelsErrorC2 = (obs - projected.cast<double>()).cwiseAbs();

                sumSquaredDifferencesUVC1 += (pixelsErrorC1).cwiseAbs2();
                sumSquaredDifferencesUVC2 += (pixelsErrorC2).cwiseAbs2();
                sumSquaredDifferencesUV += ((pixelsErrorC1 + pixelsErrorC2)).cwiseAbs2();

                meanRepErrorUVC1 += pixelsErrorC1;
                meanRepErrorUVC2 += pixelsErrorC2;
                meanRepErrorUV += (pixelsErrorC1 + pixelsErrorC2);

                nMatches++;
            }

            meanRepErrorUVC1 /= static_cast<double>(nMatches);
            meanRepErrorUVC2 /= static_cast<double>(nMatches);
            meanRepErrorUV /= static_cast<double>(2 * nMatches);

            meanRepErrorC1 = (meanRepErrorUVC1[0] + meanRepErrorUVC1[1]) / 2.0;
            meanRepErrorC2 = (meanRepErrorUVC2[0] + meanRepErrorUVC2[1]) / 2.0;
            meanRepError = (meanRepErrorUV[0] + meanRepErrorUV[1]) / 2.0;

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
        }
    }

    double desv = (desvRepErrorC1 + desvRepErrorC2) / 2.0;
    double mean = (meanRepErrorC1 + meanRepErrorC2) / 2.0;

    // std::cout << "meanRepErrorC1: " << meanRepErrorC1 << "\n";
    // std::cout << "meanRepError: " << meanRepError << "\n";
    std::cout << "desvRepErrorC1: " << desvRepErrorC1 << "\n";
    std::cout << "desvRepErrorC2: " << desvRepErrorC2 << "\n";
    
    pixelsErrors.avgc1 = meanRepErrorC1;
    pixelsErrors.avgc2 = meanRepErrorC2;
    pixelsErrors.avg = mean;
    pixelsErrors.desvc1 = desvRepErrorC1;
    pixelsErrors.desvc2 = desvRepErrorC2;
    pixelsErrors.desv = desv;
}


Eigen::Vector3d ComputeCentroid(const std::vector<Eigen::Vector3d>& positions) {
    Eigen::Vector3d centroid(0.0, 0.0, 0.0);
    for (const auto& pos : positions) {
        centroid += pos;
    }
    centroid /= positions.size();
    return centroid;
}

void EstimateRotationAndTranslation(const std::vector<Eigen::Vector3d>& v1Positions, 
                                    const std::vector<Eigen::Vector3d>& v2Positions, 
                                    Eigen::Matrix3d& rotation, 
                                    Eigen::Vector3d& translation) {
    // Ensure both sets have the same number of points
    assert(v1Positions.size() == v2Positions.size());

    size_t n = v1Positions.size();

    Eigen::Vector3d centroid1 = ComputeCentroid(v1Positions);
    Eigen::Vector3d centroid2 = ComputeCentroid(v2Positions);

    std::vector<Eigen::Vector3d> centeredV1(n);
    std::vector<Eigen::Vector3d> centeredV2(n);
    
    for (size_t i = 0; i < n; ++i) {
        centeredV1[i] = v1Positions[i] - centroid1;
        centeredV2[i] = v2Positions[i] - centroid2;
    }

    Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
    for (size_t i = 0; i < n; ++i) {
        H += centeredV1[i] * centeredV2[i].transpose();
    }

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    
    rotation = V * U.transpose();
    
    if (rotation.determinant() < 0) {
        U.col(2) *= -1; // Flip the sign of the last column of U
        rotation = V * U.transpose();
    }

    translation = rotation * centroid2 - centroid1;
}

void computeR(std::shared_ptr<open3d::geometry::TriangleMesh> mesh,
                        std::vector<Eigen::Vector3d>& v1Positions, 
                        std::vector<Eigen::Vector3d>& v2Positions, 
                        std::vector<Sophus::SO3d>& Rs) {
    auto posIndexes = createVectorMap(mesh->vertices_, v1Positions);
    std::map<size_t, size_t> invertedPosIndexes;
    for (const auto& pair : posIndexes) {
        invertedPosIndexes[pair.second] = pair.first;
    }

    std::unordered_map<Eigen::Vector2i,
                        double,
                        open3d::utility::hash_eigen<Eigen::Vector2i>> edge_weights = ComputeEdgeWeightsCot(mesh, 0);

    for (int posIndex = 0; posIndex < v1Positions.size(); posIndex++){
        auto it = invertedPosIndexes.find(posIndex);
        size_t i = 0;
        if (it != invertedPosIndexes.end()) {
            i = it->second;
        } else {
            continue;
        }

        std::unordered_set<int> jIndexes = mesh->adjacency_list_[i];

        Eigen::Matrix3d Si = Eigen::Matrix3d::Zero();
        
        for (int j : jIndexes) {
            double weight = edge_weights[GetOrderedEdge(i, j)];

            Eigen::Vector3d pi1 = v1Positions[posIndexes[i]];
            Eigen::Vector3d pj1 = v1Positions[posIndexes[j]];
            Eigen::Vector3d pi2 = v2Positions[posIndexes[i]];
            Eigen::Vector3d pj2 = v2Positions[posIndexes[j]];

            Eigen::Vector3d undeformed_eij = pi1 - pj1;
            Eigen::Vector3d deformed_eij = pi2 - pj2;

            Si += weight * undeformed_eij * deformed_eij.transpose();
        }

        Eigen::JacobiSVD<Eigen::Matrix3d> svd(Si, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3d U = svd.matrixU();
        Eigen::Matrix3d V = svd.matrixV();
        
        Eigen::Matrix3d Ri = V * U.transpose();
        
        if (Ri.determinant() < 0) {
            U.col(2) *= -1; // Flip the sign of the last column of U
            Ri = V * U.transpose();
        }
        
        Sophus::SO3d rotation(Ri);
        Rs[i] = Ri;
    }
}


float Interpolate(const float x, const float y, const float* mat, const int cols){
    float x_,_x,y_,_y;
    _x = modf(x,&x_);
    _y = modf(y,&y_);

    //Get interpolation weights
    float w00 = (1.f - _x)*(1.f - _y);
    float w01 = (1.f - _x)*_y;
    float w10 = _x*(1.f -_y);
    float w11 = 1.f - w00 - w01 - w10;

    return (float)(mat[(int)y_*cols+(int)x_])*w00 + (float)(mat[(int)y_*cols+(int)x_+1])*w10 +
           (float)(mat[((int)y_+1)*cols+(int)x_])*w01 + (float)(mat[((int)y_+1)*cols+(int)x_+1])*w11;
}