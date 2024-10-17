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
#include "open3d/geometry/Qhull.h"
#include "open3d/geometry/TetraMesh.h"

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/sba/types_sba.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

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
void arapOptimization(Map* pMap, double globalBalanceWeight, double arapBalanceWeight, int nOptIterations);


void arapOpen3DOptimization(Map* pMap);

/*
 * Performs a As-Rigid-As-Possible optimization joined with a bundle adjustment optimization (optimizes 3D poses of the cameras and 3D points positions in the space)
 */
// void arapBundleAdjustment(Map* pMap);

/*
 * Compute the tipical desviation of the distances between two objects given two mappoints to compare
 */
Eigen::Vector3d getInvUncertainty(int i, std::unordered_set<int> adjacencyList, std::map<size_t, size_t> posIndexes, std::vector<Eigen::Vector3d> v1Positions, std::vector<Eigen::Vector3d> v2Positions);

#endif //SLAM_G2OBUNDLEADJUSTMENT_H