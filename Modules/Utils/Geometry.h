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
#include <g2o/types/sba/types_sba.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include "open3d/Open3D.h"
#include "Map/Map.h"
#include "Utils/CommonTypes.h"

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

void triangulateClassic(const Eigen::Vector3f& xn1, const Eigen::Vector3f& xn2, const Sophus::SE3f& T1w, 
                 const Sophus::SE3f& T2w, Eigen::Vector3f& x3D_1, Eigen::Vector3f& x3D_2, std::string location);

void triangulateNRSLAM(const Eigen::Vector3f& xn1, const Eigen::Vector3f& xn2,
                 const Sophus::SE3f& T1w, const Sophus::SE3f& T2w, 
                 Eigen::Vector3f& x3D_1, Eigen::Vector3f& x3D_2, std::string location);

void triangulateORBSLAM(const Eigen::Vector3f& xn1, const Eigen::Vector3f& xn2,
                        const Sophus::SE3f& Tcw1, const Sophus::SE3f& Tcw2,
                        Eigen::Vector3f& point1, Eigen::Vector3f& point2, std::string location);

void triangulateDepth(const Eigen::Vector3f& xn1, const Eigen::Vector3f& xn2, const Sophus::SE3f& T1w, 
                        const Sophus::SE3f& T2w, Eigen::Vector3f& x3D_1, Eigen::Vector3f& x3D_2, std::string location);

/*
* Triangulate points using the triangulation method selected
*/
bool useTriangulationMethod(const Eigen::Vector3f& xn1, const Eigen::Vector3f& xn2, 
                                const Sophus::SE3f& T1w, const Sophus::SE3f& T2w, 
                                Eigen::Vector3f& x3D_1, Eigen::Vector3f& x3D_2,
                                std::string TrianMethod, std::string TrianLocation);

                        
/*
 * Squared reprojection error
 */
float squaredReprojectionError(cv::Point2f &p1, cv::Point2f &p2);

/*
 * Computes an Essential matrix from a relative camera pose between 2 cameras
 */
Eigen::Matrix<float,3,3> computeEssentialMatrixFromPose(Sophus::SE3f& T12);

/*
 * Extract the 3D positions as a vector of the entire set of mapPoints
 */
std::vector<Eigen::Vector3d> extractPositions(const std::vector<std::shared_ptr<MapPoint>>& mapPoints);

/*
 * Computes edge weights cotangent values as an unordered map
 */
std::unordered_map<Eigen::Vector2i, double, open3d::utility::hash_eigen<Eigen::Vector2i>> 
ComputeEdgeWeightsCot(
        std::shared_ptr<open3d::geometry::TriangleMesh> mesh,
        double min_weight);

/*
 * Create a indexes map between two pairs of 3d vectors 
 */
std::map<size_t, size_t> createVectorMap(const std::vector<Eigen::Vector3d>& vertices, const std::vector<Eigen::Vector3d>& positions, double precision = 1e-6);

/*
 * Create a 3D mesh using 2D Delaunay algorithm and adding the z coordinate
 */
std::shared_ptr<open3d::geometry::TriangleMesh> ComputeDelaunayTriangulation3D(
        const std::vector<Eigen::Vector3d> &points);
/*
 * Get ordered indexes
 */
static inline Eigen::Vector2i GetOrderedEdge(int vidx0, int vidx1) {
        return Eigen::Vector2i(std::min(vidx0, vidx1), std::max(vidx0, vidx1));
}

/*
 * Given a map with map points, it return the standard desviation of the reprojection error in pixels
 */
void calculatePixelsStandDev(std::shared_ptr<Map> Map, PixelsError& pixelsErrors);

/*
 * Function to compute centroids of a set of points
 */
Eigen::Vector3d ComputeCentroid(const std::vector<Eigen::Vector3d>& positions);

/*
 * Function to estimate the initial global rotation and translation
 */
void EstimateRotationAndTranslation(const std::vector<Eigen::Vector3d>& v1Positions, 
                                    const std::vector<Eigen::Vector3d>& v2Positions, 
                                    Eigen::Matrix3d& rotation, 
                                    Eigen::Vector3d& translation);

/*
 * Function to compute local Rotation matrices for each vertex i in the ARAP optimization
 */
void computeR(std::shared_ptr<open3d::geometry::TriangleMesh> mesh, 
                        std::vector<Eigen::Vector3d>& v1Positions,
                        std::vector<Eigen::Vector3d>& v2Positions,
                        std::vector<Sophus::SO3d>& Rs);
/*
 * Interpolates a value in a matrix given a floating point position.
 */
float Interpolate(const float x, const float y, float* mat, const int cols);

#endif //SLAM_GEOMETRY_H
