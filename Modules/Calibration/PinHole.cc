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

#include "PinHole.h"

#define fx vParameters_[0]
#define fy vParameters_[1]
#define cx vParameters_[2]
#define cy vParameters_[3]

void PinHole::project(const Eigen::Vector3f& p3D, Eigen::Vector2f& p2D){
    /*
     * Your code for Lab 3 - Task 2 here!
     */

    p2D[0] = fx * p3D[0] / p3D[2] + cx;
    p2D[1] = fy * p3D[1] / p3D[2] + cy;
}

void PinHole::unproject(const Eigen::Vector2f& p2D, Eigen::Vector3f& p3D) {

    p3D[0] = (p2D[0] - cx) / fx;
    p3D[1] = (p2D[1] - cy) / fy;
    p3D[2] = 1.0;
}

// void PinHole::unprojectWithZ(const Eigen::Vector2f& p2D, Eigen::Vector3f& p3D, double z) {
//     p3D[0] = z * (p2D[0] - cx) / fx;
//     p3D[1] = z * (p2D[1] - cy) / fy;
//     p3D[2] = z;
// }

void PinHole::projectJac(const Eigen::Vector3f& p3D, Eigen::Matrix<float,2,3>& Jac) {
    /*
     * Your code for Lab 3 - Task 2 here!
     */

    Jac(0, 0) = fx / p3D[2];
    Jac(0, 1) = 0.f;
    Jac(0, 2) = -fx * p3D[0] / (p3D[2] * p3D[2]);

    Jac(1, 0) = 0.f;
    Jac(1, 1) = fy / p3D[2];
    Jac(1, 2) = -fy * p3D[1] / (p3D[2] * p3D[2]);
}

void PinHole::unprojectJac(const Eigen::Vector2f& p2D, Eigen::Matrix<float,3,2>& Jac) {
    Jac(0,0) = 1.0 / fx;
    Jac(0,1) = 0.f;

    Jac(1,0) = 0.f;
    Jac(1,1) = 1.0 / fy;

    Jac(2,0) = 0.f;
    Jac(2,1) = 0.f;
}

// Eigen::Matrix3f PinHole::getCalibrationMatrix() const {
//     Eigen::Matrix3f K = Eigen::Matrix3f::Identity();
//     K(0, 0) = fx;
//     K(0, 2) = cx;
//     K(1, 1) = fy;
//     K(1, 2) = cy;
//     return K;
// }