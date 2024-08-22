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
    std::cout << "A:" << A << "\n";

    Eigen::Vector3f n = svd.matrixV().col(1);

    Eigen::Vector3f m0_ = m0 - (m0.dot(n)) * n;
    Eigen::Vector3f m1_ = m1 - (m1.dot(n)) * n;

    Eigen::Vector3f z = m1_.cross(m0_);
    float lambda0 = z.dot(T21.translation().cross(m1_))/(z.squaredNorm());
    Eigen::Vector3f p3D1 = T21.translation() + lambda0*m0_;

    float lambda1 = z.dot(T21.translation().cross(m0_))/(z.squaredNorm());
    Eigen::Vector3f p3D2 = lambda1 * m1_;

    x3D_1 = T2w.inverse() * p3D1;
    std::cout << "x3D_1: x:" << x3D_1.x() << " y: " << x3D_1.y() << " z: " << x3D_1.z() << "\n";
    x3D_2 = T2w.inverse() * p3D2; // same 3D point
    std::cout << "x3D_2: x:" << x3D_2.x() << " y: " << x3D_2.y() << " z: " << x3D_2.z() << "\n";
}

void triangulateBerkeley(const Eigen::Vector3f &xn1, const Eigen::Vector3f &xn2,
                        Frame &F1, Frame &F2,
                        Eigen::Vector3f& point1, Eigen::Vector3f& point2) {
    // Retrieve intrinsic camera matrices for F1 and F2
    Eigen::Matrix3f K1 = F1.getCalibration()->getCalibrationMatrix();
    Eigen::Matrix3f K2 = F2.getCalibration()->getCalibrationMatrix();

    // Retrieve the poses (Tcw) of the frames F1 and F2, where Tcw is the transformation
    // from world coordinates to camera coordinates.
    Sophus::SE3f T1w = F1.getPose();
    Sophus::SE3f T2w = F2.getPose();

    // Compute the projection matrices P1 and P2
    // Extract the rotation and translation from Tcw
    Eigen::Matrix3f R1w = T1w.rotationMatrix();
    Eigen::Vector3f t1w = T1w.translation();

    Eigen::Matrix3f R2w = T2w.rotationMatrix();
    Eigen::Vector3f t2w = T2w.translation();

    // std::cout << "R1w:" << R1w << "\n";
    // std::cout << "t1w:" << t1w << "\n";
    // std::cout << "R1w:" << R2w << "\n";
    // std::cout << "t1w:" << t2w << "\n";
    // std::cout << "K1:" << K1 << "\n";
    // std::cout << "K2:" << K2 << "\n";

    // std::cout << "xn1 u:" << xn1[0] << "\n";
    // std::cout << "xn1 v:" << xn1[1] << "\n";
    // std::cout << "xn2 u:" << xn2[0] << "\n";
    // std::cout << "xn2 v:" << xn2[1] << "\n";

    // Construct the 3x4 projection matrices
    Eigen::Matrix<float, 3, 4> P1;
    P1.block<3,3>(0,0) = R1w;
    P1.block<3,1>(0,3) = t1w;
    P1 = K1 * P1;  // P1 = K1 * [R1 | t1]

    Eigen::Matrix<float, 3, 4> P2;
    P2.block<3,3>(0,0) = R2w;
    P2.block<3,1>(0,3) = t2w;
    P2 = K2 * P2;  // P2 = K2 * [R2 | t2]
    
    // Prepare matrix A for triangulation
    Eigen::MatrixXf A(4, 4);

    P1 /= P1.norm();
    P2 /= P2.norm();

    Eigen::Vector3f m0 = xn1.normalized();
    Eigen::Vector3f m1 = xn2.normalized();

    // Fill A with the rows corresponding to each view
    A.row(0) = m0[0] * P1.row(2) - P1.row(0);
    A.row(1) = m0[1] * P1.row(2) - P1.row(1);
    A.row(2) = m1[0] * P2.row(2) - P2.row(0);
    A.row(3) = m1[1] * P2.row(2) - P2.row(1);

    std::cout << "A:" << A << "\n";


    // Compute the SVD of A
    Eigen::JacobiSVD<Eigen::MatrixXf> svd;
    svd.compute(A, Eigen::ComputeThinU | Eigen::ComputeFullV);

    // Check if the SVD was successful
    if (!svd.computeV()) {
         std::cerr << "Failed to compute a singular value decomposition of A matrix.";
        return;
    }

    // The 3D point is the eigenvector corresponding to the minimum eigenvalue.
    point1 = (svd.matrixV().block(0, 3, 3, 1) / svd.matrixV()(3,3)).cast<float>();
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