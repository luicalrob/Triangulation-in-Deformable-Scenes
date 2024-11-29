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
 * Implementaiton of the Budle Adjustemn problem with the g2o optimization library
 */

#ifndef SLAM_G2OBUNDLEADJUSTMENT_H
#define SLAM_G2OBUNDLEADJUSTMENT_H

#include "Map/Map.h"
#include "open3d/Open3D.h"

/*
 * Performs a full Bundle Adjustment (optimizes both camera poses and 3D points)
 */
void bundleAdjustment(Map* pMap);

/*
 * Performs an only pose optimization with the given Frame. It detects outliers and removes them
 * from the Frame. Returns the number of inliers (the number of MapPoints hold after the optimization)
 */
int poseOnlyOptimization(Frame& currFrame);

/*
 * Performs a Bundle Adjustmen using the local map around the given KeyFrame
 */
void localBundleAdjustment(Map* pMap, ID currKeyFrameId);

/*
 * Performs a As-Rigid-As-Possible optimization joined with a reprojection error minimization (optimizes 3D points positions in the space)
 */
void arapOptimization(Map* pMap, double repBalanceWeight, double globalBalanceWeight, double arapBalanceWeight, double alphaWeight, 
                        double betaWeight, float DepthError, int nOptIterations, double* optimizationUpdate = nullptr);


void arapOpen3DOptimization(Map* pMap);

/*
 * Performs a As-Rigid-As-Possible optimization joined with a bundle adjustment optimization (optimizes 3D poses of the cameras and 3D points positions in the space)
 */
// void arapBundleAdjustment(Map* pMap);

/*
 * Compute the tipical desviation of the distances between two objects given two mappoints to compare
 */
double getInvUncertainty(std::shared_ptr<open3d::geometry::TriangleMesh> mesh, 
                        std::vector<Eigen::Vector3d> v1Positions, 
                        std::vector<Eigen::Vector3d> v2Positions,
                        size_t i);

#endif //SLAM_G2OBUNDLEADJUSTMENT_H