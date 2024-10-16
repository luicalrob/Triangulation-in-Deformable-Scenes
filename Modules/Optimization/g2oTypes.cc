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

#include "Optimization/g2oTypes.h"

VertexSBAPointXYZ::VertexSBAPointXYZ() : BaseVertex<3, Eigen::Vector3d>()
{
}

bool VertexSBAPointXYZ::read(std::istream& is)
{

    is >> _estimate(0 ), _estimate(1 ), _estimate(2 );
    return true;
}

bool VertexSBAPointXYZ::write(std::ostream& os) const
{
    os << _estimate;
    return true;
}

VertexRotationMatrix::VertexRotationMatrix() : BaseVertex<3, Eigen::Matrix3d>() {
    _estimate.setIdentity();
}

bool VertexRotationMatrix::read(std::istream& is) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            is >> _estimate(i, j);
        }
    }
    return true;
}

bool VertexRotationMatrix::write(std::ostream& os) const {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            os << _estimate(i, j) << " ";
        }
    }
    return true;
}


VertexTranslationVector::VertexTranslationVector() : BaseVertex<3, Eigen::Vector3d>() {
    _estimate.setZero();
}

bool VertexTranslationVector::read(std::istream& is) {
    for (int i = 0; i < 3; ++i)
        is >> _estimate[i];
    return true;
}

bool VertexTranslationVector::write(std::ostream& os) const {
    for (int i = 0; i < 3; ++i)
        os << _estimate[i] << " ";
    return os.good();
}


EdgeSE3ProjectXYZ::EdgeSE3ProjectXYZ() : BaseBinaryEdge<2, Eigen::Vector2d, VertexSBAPointXYZ, g2o::VertexSE3Expmap>() {
}

bool EdgeSE3ProjectXYZ::read(std::istream& is){
    for (int i=0; i<2; i++){
        is >> _measurement[i];
    }
    for (int i=0; i<2; i++)
        for (int j=i; j<2; j++) {
            is >> information()(i,j);
            if (i!=j)
                information()(j,i)=information()(i,j);
        }
    return true;
}

bool EdgeSE3ProjectXYZ::write(std::ostream& os) const {

    for (int i=0; i<2; i++){
        os << measurement()[i] << " ";
    }

    for (int i=0; i<2; i++)
        for (int j=i; j<2; j++){
            os << " " <<  information()(i,j);
        }
    return os.good();
}


void EdgeSE3ProjectXYZ::linearizeOplus() {
    g2o::VertexSE3Expmap * vj = static_cast<g2o::VertexSE3Expmap *>(_vertices[1]);
    g2o::SE3Quat T(vj->estimate());
    VertexSBAPointXYZ* vi = static_cast<VertexSBAPointXYZ*>(_vertices[0]);
    Eigen::Vector3d xyz = vi->estimate();
    Eigen::Vector3d xyz_trans = T.map(xyz);

    double x = xyz_trans[0];
    double y = xyz_trans[1];
    double z = xyz_trans[2];

    Eigen::Matrix<double,2,3> projectJac = -pCamera->projectJac(xyz_trans);

    _jacobianOplusXi =  projectJac * T.rotation().toRotationMatrix();

    Eigen::Matrix<double,3,6> SE3deriv;
    SE3deriv << 0.f, z,   -y, 1.f, 0.f, 0.f,
                 -z , 0.f, x, 0.f, 1.f, 0.f,
                 y ,  -x , 0.f, 0.f, 0.f, 1.f;

    _jacobianOplusXj = projectJac * SE3deriv;
}

EdgeSE3ProjectXYZOnlyPose::EdgeSE3ProjectXYZOnlyPose(){}

bool EdgeSE3ProjectXYZOnlyPose::read(std::istream& is){
    for (int i=0; i<2; i++){
        is >> _measurement[i];
    }
    for (int i=0; i<2; i++)
        for (int j=i; j<2; j++) {
            is >> information()(i,j);
            if (i!=j)
                information()(j,i)=information()(i,j);
        }
    return true;
}

bool EdgeSE3ProjectXYZOnlyPose::write(std::ostream& os) const {

    for (int i=0; i<2; i++){
        os << measurement()[i] << " ";
    }

    for (int i=0; i<2; i++)
        for (int j=i; j<2; j++){
            os << " " <<  information()(i,j);
        }
    return os.good();
}


void EdgeSE3ProjectXYZOnlyPose::linearizeOplus() {
    g2o::VertexSE3Expmap * vj = static_cast<g2o::VertexSE3Expmap *>(_vertices[0]);
    Eigen::Vector3d xyz_trans = vj->estimate().map(Xworld);

    double x = xyz_trans[0];
    double y = xyz_trans[1];
    double z = xyz_trans[2];

    Eigen::Matrix<double,2,3> projectJac = -pCamera->projectJac(xyz_trans);

    Eigen::Matrix<double,3,6> SE3deriv;
    SE3deriv << 0.f, z,   -y, 1.f, 0.f, 0.f,
            -z , 0.f, x, 0.f, 1.f, 0.f,
            y ,  -x , 0.f, 0.f, 0.f, 1.f;

    _jacobianOplusXi = projectJac * SE3deriv;
}

EdgeSE3ProjectXYZPerKeyFrame::EdgeSE3ProjectXYZPerKeyFrame(){}

bool EdgeSE3ProjectXYZPerKeyFrame::read(std::istream& is){
    for (int i=0; i<2; i++){
        is >> _measurement[i];
    }
    for (int i=0; i<2; i++)
        for (int j=i; j<2; j++) {
            is >> information()(i,j);
            if (i!=j)
                information()(j,i)=information()(i,j);
        }
    return true;
}

bool EdgeSE3ProjectXYZPerKeyFrame::write(std::ostream& os) const {

    for (int i=0; i<2; i++){
        os << measurement()[i] << " ";
    }

    for (int i=0; i<2; i++)
        for (int j=i; j<2; j++){
            os << " " <<  information()(i,j);
        }
    return os.good();
}

void EdgeSE3ProjectXYZPerKeyFrame::linearizeOplus() {
    g2o::VertexSE3Expmap * vj = static_cast<g2o::VertexSE3Expmap *>(_vertices[1]);
    g2o::SE3Quat T(vj->estimate());
    VertexSBAPointXYZ* vi = static_cast<VertexSBAPointXYZ*>(_vertices[0]);
    Eigen::Vector3d xyz = vi->estimate();
    Eigen::Vector3d xyz_trans = T.map(xyz);

    double x = xyz_trans[0];
    double y = xyz_trans[1];
    double z = xyz_trans[2];

    Eigen::Matrix<double,2,3> projectJac = -pCamera->projectJac(xyz_trans);

    _jacobianOplusXi =  projectJac * T.rotation().toRotationMatrix(); //2x3

    Eigen::Matrix<double,3,6> SE3deriv;
    SE3deriv << 0.f, z,   -y, 1.f, 0.f, 0.f,
                 -z , 0.f, x, 0.f, 1.f, 0.f,
                 y ,  -x , 0.f, 0.f, 0.f, 1.f;

    _jacobianOplusXj = projectJac * SE3deriv; //2x6
}

EdgeSE3ProjectXYZPerKeyFrameOnlyPoints::EdgeSE3ProjectXYZPerKeyFrameOnlyPoints(){}

bool EdgeSE3ProjectXYZPerKeyFrameOnlyPoints::read(std::istream& is){
    for (int i=0; i<2; i++){
        is >> _measurement[i];
    }
    for (int i=0; i<2; i++)
        for (int j=i; j<2; j++) {
            is >> information()(i,j);
            if (i!=j)
                information()(j,i)=information()(i,j);
        }
    return true;
}

bool EdgeSE3ProjectXYZPerKeyFrameOnlyPoints::write(std::ostream& os) const {

    for (int i=0; i<2; i++){
        os << measurement()[i] << " ";
    }

    for (int i=0; i<2; i++)
        for (int j=i; j<2; j++){
            os << " " <<  information()(i,j);
        }
    return os.good();
}

void EdgeSE3ProjectXYZPerKeyFrameOnlyPoints::linearizeOplus() {
    g2o::SE3Quat T = cameraPose;
    VertexSBAPointXYZ* vi = static_cast<VertexSBAPointXYZ*>(_vertices[0]);
    Eigen::Vector3d xyz = vi->estimate();
    Eigen::Vector3d xyz_trans = T.map(xyz);

    double x = xyz_trans[0];
    double y = xyz_trans[1];
    double z = xyz_trans[2];

    Eigen::Matrix<double,2,3> projectJac = -pCamera->projectJac(xyz_trans);

    _jacobianOplusXi =  projectJac * T.rotation().toRotationMatrix(); //2x3
}


EdgeARAP::EdgeARAP(){
    resize(3);
}

bool EdgeARAP::read(std::istream& is){

    for (int i=0; i<3; i++){
        is >> _measurement[i];
    }

    for (int i=0; i<3; i++)
        for (int j=i; j<3; j++) {
            is >> information()(i,j);
            if (i!=j)
                information()(j,i)=information()(i,j);
        }
    return true;
}

bool EdgeARAP::write(std::ostream& os) const {

    for (int i=0; i<3; i++){
        os << measurement()[i] << " ";
    }

    for (int i=0; i<3; i++)
        for (int j=i; j<3; j++){
            os << " " <<  information()(i,j);
        }
    return os.good();
}

// bool EdgeARAP::read(std::istream& is) {
//     is >> _measurement;

//     double info;
//     is >> info;
//     information()(0, 0) = info;

//     return true;
// }

// bool EdgeARAP::write(std::ostream& os) const {
//     os << _measurement << " ";

//     os << information()(0, 0);
//     return os.good();
// }

// void EdgeARAP::linearizeOplus() {
//     //VertexSBAPointXYZ* v1 = static_cast<VertexSBAPointXYZ*>(_vertices[0]);
//     VertexSBAPointXYZ* v2 = static_cast<VertexSBAPointXYZ*>(_vertices[0]);
//     VertexSO3* vR = static_cast<VertexSO3*>(_vertices[1]);
//     g2o::VertexSE3Expmap* vT = static_cast<g2o::VertexSE3Expmap*>(_vertices[2]);

//     Eigen::Vector3d obs(_measurement);
//     //double obs(_measurement);
//     Eigen::Vector3d arap_diff;

//     Eigen::Matrix3d R = vR->estimate().matrix();
//     g2o::SE3Quat T_global =  vT->estimate();
//     Eigen::Matrix3d Rg = T_global.rotation().toRotationMatrix();
//     Eigen::Vector3d t = T_global.translation();

//     //diff = weight*((v2->estimate() - Xj2world) - (R * (Xi1world - Xj1world))^2 + (Rg * (v2->estimate() - Xi1world) - t)^2);
//     arap_diff = (v2->estimate() - Xj2world) - (R * (Xi1world - Xj1world));
    
//     Eigen::Vector3d J_v1_3x1_FT = -2 * weight * (arap_diff.transpose() * R).transpose();
//     Eigen::Vector3d J_v2_3x1_FT = 2 * weight * arap_diff;
//     Eigen::Matrix3d J_R_mat3_FT = -2 * weight * (arap_diff * Xi1world.transpose());

//     Eigen::Vector3d global_diff = (v2->estimate() - Xi1world);
//     Eigen::Vector3d transfomrationTerm = Rg * global_diff - t;

//     Eigen::Vector3d J_v1_3x1_ST = -2 * weight * (transfomrationTerm.transpose() * Rg).transpose();
//     Eigen::Vector3d J_v2_3x1_ST = 2 * weight * (transfomrationTerm.transpose() * Rg).transpose();
//     Eigen::Matrix3d J_Rg_mat3_ST = -2 * weight * Rg * global_diff * global_diff.transpose();
//     Eigen::Vector3d J_tg_mat3_ST = -2 * weight * transfomrationTerm;

//     // Eigen::Matrix3d J_v1_3x1_FT;
//     // J_v1_3x1_FT.setZero();
//     // Eigen::Vector3d J_v2_3x1_FT;
//     // J_v2_3x1_FT.setZero();
//     // Eigen::Matrix3d J_R_mat3_FT;
//     // J_R_mat3_FT.setZero();

//     // Eigen::Matrix3d J_v1_3x1_ST;
//     // Eigen::Matrix3d J_v2_3x1_ST;
//     // Eigen::Matrix3d J_Rg_mat3_ST;
//     // Eigen::Vector3d J_tg_mat3_ST;

//     // J_v1_3x1_ST.setZero();
//     // J_v2_3x1_ST.setZero();
//     // J_Rg_mat3_ST.setZero();
//     // J_tg_mat3_ST.setZero();


//     Eigen::Matrix3d J_v1_3x1 = J_v1_3x1_FT.asDiagonal().toDenseMatrix() + J_v1_3x1_ST.asDiagonal().toDenseMatrix();
//     Eigen::Matrix3d J_v2_3x1 = J_v2_3x1_FT.asDiagonal().toDenseMatrix() + J_v2_3x1_ST.asDiagonal().toDenseMatrix();
//     Eigen::Matrix3d J_R_mat3 = J_R_mat3_FT;
//     Eigen::Matrix3d J_G_mat3 = J_Rg_mat3_ST + J_tg_mat3_ST.asDiagonal().toDenseMatrix();

//     Eigen::Matrix<double, 3, 3> J_v1_mat = J_v1_3x1;
//     Eigen::Matrix<double, 3, 3> J_v2_mat = J_v2_3x1;
//     Eigen::Matrix<double, 3, 3> J_R = J_R_mat3; 
//     Eigen::Matrix<double, 3, 6> J_global;// = J_G_mat3; 
//     J_global.block<3,3>(0,0) = J_Rg_mat3_ST;
//     J_global.block<3,3>(0,3) = J_tg_mat3_ST.asDiagonal(); 
//     //J_global.setZero(); 

//     //_jacobianOplus[0] = J_v1_mat;
//     _jacobianOplus[0] = J_v2_mat;
//     _jacobianOplus[1] = J_R;
//     _jacobianOplus[2] = J_global;
// }
