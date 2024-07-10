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

/*
 * Author: Juan J. Gómez Rodríguez (jjgomez@unizar.es)
 *
 * File with some useful geometric functons
 */

#ifndef SLAM_GEOMETRY_H
#define SLAM_GEOMETRY_H

#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <opencv2/opencv.hpp>

/*
 * Computes the cosine of the parallax angle between 2 rays
 */
float cosRayParallax(const Eigen::Vector3f& a, const Eigen::Vector3f& b);

/*
 * Triangulates a 3D point form the camera poses and the normalized bearing rays.
 * Lee, Seong Hun, and Javier Civera. "Closed-form optimal two-view triangulation based on angular errors."
 * Proceedings of the IEEE/CVF International Conference on Computer Vision. 2019.
 */
void triangulate(const Eigen::Vector3f &xn1, const Eigen::Vector3f &xn2,
                 const Sophus::SE3f &T1w, const Sophus::SE3f &T2w, Eigen::Vector3f &x3D);

void triangulateInRays(const Eigen::Vector3f &xn1, const Eigen::Vector3f &xn2, const Sophus::SE3f &T1w, 
                 const Sophus::SE3f &T2w, Eigen::Vector3f &x3D_1, Eigen::Vector3f &x3D_2);
                 
void triangulateTwoPoints(const Eigen::Vector3f &xn1, const Eigen::Vector3f &xn2, const Sophus::SE3f &T1w, 
                 const Sophus::SE3f &T2w, Eigen::Vector3f &x3D_1, Eigen::Vector3f &x3D_2);

/*
 * Squared reprojection error
 */
float squaredReprojectionError(cv::Point2f &p1, cv::Point2f &p2);

/*
 * Computes an Essential matrix from a relative camera pose between 2 cameras
 */
Eigen::Matrix<float,3,3> computeEssentialMatrixFromPose(Sophus::SE3f& T12);

/*
 * Computes cotangent value givem 3 vertex 
 */
double cotangent(const Eigen::Vector3d &v0, const Eigen::Vector3d &v1, const Eigen::Vector3d &v2);

/*
 * Returns the x nearest neighbour of mainMP mapPoint given a list of mapPoints and x number of neighbours desired
 */
std::vector<MapPoint*> findNearestNeighbors(MapPoint* mainMP, const std::vector<MapPoint*>& vMPs, int x);

#endif //SLAM_GEOMETRY_H
