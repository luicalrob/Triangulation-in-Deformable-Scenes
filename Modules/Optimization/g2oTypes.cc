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
    resize(5);
}

bool EdgeARAP::read(std::istream& is) {
    is >> _measurement;

    double info;
    is >> info;
    information()(0, 0) = info;

    return true;
}

bool EdgeARAP::write(std::ostream& os) const {
    os << _measurement << " ";

    os << information()(0, 0);
    return os.good();
}


// void EdgeARAP::linearizeOplus() {
//     const VertexSBAPointXYZ* v1i = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
//     const VertexSBAPointXYZ* v2i = static_cast<const VertexSBAPointXYZ*>(_vertices[1]);
//     const VertexSBAPointXYZ* v1j = static_cast<const VertexSBAPointXYZ*>(_vertices[2]);
//     const VertexSBAPointXYZ* v2j = static_cast<const VertexSBAPointXYZ*>(_vertices[3]);

//     Eigen::Vector3d firstUndeformed_eij = v1i->estimate() - v1j->estimate();
//     Eigen::Vector3d firstDeformed_eij = v2i->estimate() - v2j->estimate();

//     Eigen::Vector3d firstJacobian = 4.0 * weight * (firstDeformed_eij - ((Ri.matrix()+Rj.matrix()) * firstUndeformed_eij)/2.0);
//     _jacobianOplus[0] = firstJacobian; //1x3

//     Eigen::Vector3d secondJacobian = 4.0 * weight * (firstDeformed_eij - ((Ri.matrix()+Rj.matrix()) * firstUndeformed_eij)/2.0);
//     _jacobianOplus[1] = secondJacobian; //1x3

//     Eigen::Vector3d secondUndeformed_eij = v1j->estimate() - v1i->estimate();
//     Eigen::Vector3d secondDeformed_eij = v2j->estimate() - v2i->estimate();

//     Eigen::Vector3d thirdJacobian = 4.0 * weight * (secondDeformed_eij - ((Ri.matrix()+Rj.matrix()) * secondUndeformed_eij)/2.0);
//     _jacobianOplus[2] = thirdJacobian;

//     Eigen::Vector3d fourthJacobian = 4.0 * weight * (secondDeformed_eij - ((Ri.matrix()+Rj.matrix()) * secondUndeformed_eij)/2.0);
//     _jacobianOplus[3] = fourthJacobian;
// }

EdgeOdgen::EdgeOdgen(){
    resize(4);
}

bool EdgeOdgen::read(std::istream& is) {
    is >> _measurement;

    double info;
    is >> info;
    information()(0, 0) = info;

    return true;
}

bool EdgeOdgen::write(std::ostream& os) const {
    os << _measurement << " ";

    os << information()(0, 0);
    return os.good();
}

EdgeTransformation::EdgeTransformation(){
    resize(3);
}

bool EdgeTransformation::read(std::istream& is) {
    is >> _measurement;

    double info;
    is >> info;
    information()(0, 0) = info;

    return true;
}

bool EdgeTransformation::write(std::ostream& os) const {
    os << _measurement << " ";

    os << information()(0, 0);
    return os.good();
}

// void EdgeTransformation::linearizeOplus() {
//     VertexSBAPointXYZ* v1 = static_cast<VertexSBAPointXYZ*>(_vertices[0]);
//     VertexSBAPointXYZ* v2 = static_cast<VertexSBAPointXYZ*>(_vertices[1]);
//     //const g2o::VertexSE3Expmap* vT = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);


//     Eigen::Matrix3d Rg = T.rotationMatrix().cast<double>();
//     Eigen::Vector3d t = T.translation().cast<double>();

//     Eigen::Vector3d diffGlobalT = (Rg * v2->estimate() - t) - v1->estimate();

//     _jacobianOplusXi = 2.0 * Rg * diffGlobalT; //1x3
//     _jacobianOplusXj = -2.0 * diffGlobalT; //1x3
// }