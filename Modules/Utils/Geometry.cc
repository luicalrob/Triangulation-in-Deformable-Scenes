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
    Eigen::Vector3f n = svd.matrixV().col(1);

    Eigen::Vector3f m0_ = m0 - (m0.dot(n)) * n;
    Eigen::Vector3f m1_ = m1 - (m1.dot(n)) * n;

    Eigen::Vector3f z = m1_.cross(m0_);
    float lambda0 = z.dot(T21.translation().cross(m1_))/(z.squaredNorm());
    Eigen::Vector3f p3D1 = T21.translation() + lambda0*m0;

    float lambda1 = z.dot(T21.translation().cross(m0_))/(z.squaredNorm());
    Eigen::Vector3f p3D2 = lambda1 * m1;

    x3D_1 = T2w.inverse() * p3D1;
    //std::cout << "x3D_1: x:" << x3D_1.x() << " y: " << x3D_1.y() << " z: " << x3D_1.z() << "\n";
    x3D_2 = T2w.inverse() * p3D2;
    //std::cout << "x3D_2: x:" << x3D_2.x() << " y: " << x3D_2.y() << " z: " << x3D_2.z() << "\n";
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

double cotangent(const Eigen::Vector3d &v0, const Eigen::Vector3d &v1, const Eigen::Vector3d &v2) {
    Eigen::Vector3d e0 = v1 - v0;
    Eigen::Vector3d e1 = v2 - v1;
    Eigen::Vector3d e2 = v0 - v2;

    double cosTheta = -e2.dot(e0) / (e2.norm() * e0.norm());
    return cosTheta / sqrt(1.0 - cosTheta * cosTheta);
}

std::vector<MapPoint*> findNearestNeighbors(MapPoint* mainMP, const std::vector<MapPoint*>& vMPs, int x) {
    Eigen::Vector3d p3D = mainMP->getWorldPosition().cast<double>();

    std::vector<MapPoint*> nearestNeighbors;
    for (auto pMP : vMPs) {
        if (pMP == mainMP)
            continue;

        double distSq = (pMP->getWorldPosition().cast<double>() - p3D).squaredNorm();

        if (nearestNeighbors.size() < x) {
            nearestNeighbors.push_back(pMP);
        } else {
            double farthestDistSq = (nearestNeighbors.back()->getWorldPosition().cast<double>() - p3D).squaredNorm();
            if (distSq < farthestDistSq) {
                nearestNeighbors.pop_back();  // Remove the farthest neighbor
                nearestNeighbors.push_back(pMP);  // Insert the closer neighbor
            }
        }
    }

    return nearestNeighbors;
}
