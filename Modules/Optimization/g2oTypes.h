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
 * Implementation of the types needed by g2o to perform Budnle Adjustemnt
 */

#ifndef SLAM_G2OTYPES_H
#define SLAM_G2OTYPES_H

#include "Calibration/CameraModel.h"
#include "Utils/CommonTypes.h"

#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_multi_edge.h>
#include <g2o/types/sba/types_sba.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#include <Eigen/Geometry>
#include <memory>

class  VertexSBAPointXYZ : public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexSBAPointXYZ();
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    virtual void setToOriginImpl() {
        _estimate.fill(0);
    }

    virtual void oplusImpl(const double * update)
    {
        Eigen::Map<const Eigen::Vector3d> v(update);
        _estimate += v;
    }
};

class VertexRotationMatrix : public g2o::BaseVertex<3, Eigen::Matrix3d> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexRotationMatrix();

    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    virtual void setToOriginImpl() {
        _estimate.setIdentity();
    }

    virtual void oplusImpl(const double* update) {
        Eigen::Vector3d updateVec(update);
        Eigen::AngleAxisd angleAxis(updateVec.norm(), updateVec.normalized());
        Eigen::Matrix3d updateMatrix = angleAxis.toRotationMatrix();
        _estimate = updateMatrix * _estimate;
    }
};

class VertexSO3 : public g2o::BaseVertex<3, Sophus::SO3d> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VertexSO3() {}

    virtual void setToOriginImpl() override {
        _estimate = Sophus::SO3d();  // Set to identity rotation
    }

    virtual void oplusImpl(const double *update) override {
        Eigen::Map<const Eigen::Matrix<double, 3, 1>> updateVec(update);
        _estimate = Sophus::SO3d::exp(updateVec) * _estimate;  // Apply update in SO(3)
    }

    virtual bool read(std::istream &is) override {
        double x, y, z;
        is >> x >> y >> z;
        Eigen::Vector3d v(x, y, z);
        _estimate = Sophus::SO3d::exp(v);  // Load rotation from file
        return true;
    }

    virtual bool write(std::ostream &os) const override {
        Eigen::Vector3d v = _estimate.log();  // Write the Lie algebra parameters
        os << v.x() << " " << v.y() << " " << v.z();
        return true;
    }
};

class VertexTranslationVector : public g2o::BaseVertex<3, Eigen::Vector3d> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    VertexTranslationVector();
    
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    virtual void setToOriginImpl() {
        _estimate.setZero();
    }

    virtual void oplusImpl(const double* update) {
        Eigen::Map<const Eigen::Vector3d> updateVec(update);
        _estimate += updateVec;
    }
};


/*
 * Error function for geometric (reprojection error) Bundle Adjustment with analytic derivatives. Both
 * camera pose and 3D point are optimizable parameters
 */
class EdgeSE3ProjectXYZ: public  g2o::BaseBinaryEdge<2, Eigen::Vector2d, VertexSBAPointXYZ, g2o::VertexSE3Expmap>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeSE3ProjectXYZ();

    bool read(std::istream& is);

    bool write(std::ostream& os) const;

    void computeError()  {
        const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
        const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
        Eigen::Vector2d obs(_measurement);      //Observed point in the image
        Eigen::Vector3d p3Dw = v2->estimate();  //Predicted 3D world position  of the point
        g2o::SE3Quat Tcw = v1->estimate();      //Preficted camera pose

        /*
         * Your code for Lab 3 - Task 3 here! Example:
         * _error = Eigen::Vector2d::Ones()*100;
         */
        // Project the 3D point onto the image plane using the camera pose
        Eigen::Vector3d p3Dc = Tcw.map(p3Dw);
        Eigen::Vector2f projected;
        pCamera->project(p3Dc.cast<float>(), projected);

        // Compute the reprojection error
        _error = obs - projected.cast<double>();
    }

    bool isDepthPositive() {
        const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
        const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
        return ((v1->estimate().map(v2->estimate()))(2)>0.0);
    }

    virtual void linearizeOplus();

    std::shared_ptr<CameraModel> pCamera;
};

class EdgeSE3ProjectXYZOnlyPose: public g2o::BaseUnaryEdge<2,Eigen::Vector2d,g2o::VertexSE3Expmap>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeSE3ProjectXYZOnlyPose();

    bool read(std::istream& is);

    bool write(std::ostream& os) const;

    void computeError()  {
        const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
        Eigen::Vector2d obs(_measurement);  //Observed point in the image
        g2o::SE3Quat Tcw = v1->estimate();  //Preficted camera pose

        /*
        * Your code for Lab 3 - Task 3 here! Example:
        * _error = Eigen::Vector2d::Ones()*100;
        */
        // Project the 3D point onto the image plane using the camera pose
        Eigen::Vector3d p3Dc = Tcw.map(Xworld);
        Eigen::Vector2f projected;
        pCamera->project(p3Dc.cast<float>(), projected);

        // Compute the reprojection error
        _error = obs  - projected.cast<double>();
    }

    bool isDepthPositive() {
        const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
        Eigen::Vector3d p3dCamera = v1->estimate().map(Xworld);
        return ((v1->estimate().map(Xworld))(2)>0.0);
    }

    virtual void linearizeOplus();

    Eigen::Vector3d Xworld;
    std::shared_ptr<CameraModel> pCamera;
};

class EdgeSE3ProjectXYZPerKeyFrame : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, VertexSBAPointXYZ, g2o::VertexSE3Expmap> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeSE3ProjectXYZPerKeyFrame();

    bool read(std::istream& is);

    bool write(std::ostream& os) const;

    void computeError() {
        const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
        const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
        Eigen::Vector2d obs(_measurement);      //Observed point in the image
        Eigen::Vector3d p3Dw = v2->estimate();  //Predicted 3D world position  of the point
        g2o::SE3Quat Tcw = v1->estimate();      //Preficted camera pose

        /*
         * Your code for Lab 3 - Task 3 here! Example:
         * _error = Eigen::Vector2d::Ones()*100;
         */
        // Project the 3D point onto the image plane using the camera pose
        Eigen::Vector3d p3Dc = Tcw.map(p3Dw);
        Eigen::Vector2f projected;
        pCamera->project(p3Dc.cast<float>(), projected);

        // Compute the reprojection error
        _error = obs - projected.cast<double>();
        // std::cout << "Obsevations error: " << _error << std::endl;
    }

    virtual void linearizeOplus();

    std::shared_ptr<CameraModel> pCamera;
};

class EdgeSE3ProjectXYZPerKeyFrameOnlyPoints : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, VertexSBAPointXYZ> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeSE3ProjectXYZPerKeyFrameOnlyPoints();

    bool read(std::istream& is);

    bool write(std::ostream& os) const;

    void computeError() {
        const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
        Eigen::Vector2d obs(_measurement);      //Observed point in the image
        Eigen::Vector3d p3Dw = v2->estimate();  //Predicted 3D world position  of the point
        g2o::SE3Quat Tcw = cameraPose;      //Preficted camera pose

        // Project the 3D point onto the image plane using the camera pose
        Eigen::Vector3d p3Dc = Tcw.map(p3Dw);
        Eigen::Vector2f projected;
        pCamera->project(p3Dc.cast<float>(), projected);

        // Compute the reprojection error
        _error = (obs - projected.cast<double>());
        // std::cout << "Obsevations error: (" << _error[0] << ", " << _error[1] << ")\n" << std::endl;
    }

    virtual void linearizeOplus();

    std::shared_ptr<CameraModel> pCamera;
    g2o::SE3Quat cameraPose;
    //float balance;
};

class EdgeARAP: public  g2o::BaseMultiEdge<1, double>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeARAP();

    bool read(std::istream& is);

    bool write(std::ostream& os) const;

    void computeError() {
        const VertexSBAPointXYZ* v1i = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
        const VertexSBAPointXYZ* v2i = static_cast<const VertexSBAPointXYZ*>(_vertices[1]);
        const VertexSBAPointXYZ* v1j = static_cast<const VertexSBAPointXYZ*>(_vertices[2]);
        const VertexSBAPointXYZ* v2j = static_cast<const VertexSBAPointXYZ*>(_vertices[3]);
        const g2o::VertexSE3Expmap* vT = static_cast<const g2o::VertexSE3Expmap*>(_vertices[4]);

        g2o::SE3Quat T_global =  vT->estimate();
        Eigen::Matrix3d Rg = T_global.rotation().toRotationMatrix();
        Eigen::Vector3d t = T_global.translation();

        Eigen::Vector3d diffGlobalT = (Rg * v2i->estimate() - t) - v1i->estimate();

        double energyGlobalT = diffGlobalT.squaredNorm();

        double obs(_measurement);
        double mu = alpha;
        double alp = beta;

        Eigen::Vector3d numerator = (v2i->estimate() - v2j->estimate());
        Eigen::Vector3d denominator = (v1i->estimate() - v1j->estimate());
        double lamdaij = numerator.norm() / denominator.norm();
        //double lambda_ij = std::sqrt(std::pow(numerator, 2) + std::pow(denominator, 2));

        ////tension in shear
        //double energy = (2 * mu / alp) * ((pow(lamdaij, alp)) - pow((1 / (lamdaij*lamdaij)), alp));

        ////tension in Planar
        //double energy = (2 * mu / alp) * ((pow(lamdaij, alp)) - pow((1 / lamdaij),alp));

        ////tension in Tensile
        //double energy = (2 * mu / alp) * ((pow(lamdaij, alp)) - pow((1 / sqrt(lamdaij)), alp));

        ///Deformation
        double energy = (mu / alp) * (2 * (pow(lamdaij, alp)) + pow((lamdaij),-(2*alp)) - 3);

        _error[0] = (energy + energyGlobalT) - obs;   
    }

    // virtual void linearizeOplus();

    Sophus::SO3d Ri;
    Sophus::SO3d Rj;
    double weight;
    double alpha;   // Bulk deformation parameter
    double beta;    // Shear deformation parameter
};

class EdgeOdgen: public  g2o::BaseMultiEdge<1, double>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeARAP();

    bool read(std::istream& is);

    bool write(std::ostream& os) const;

    void computeError() {
        const VertexSBAPointXYZ* v1i = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
        const VertexSBAPointXYZ* v2i = static_cast<const VertexSBAPointXYZ*>(_vertices[1]);
        const VertexSBAPointXYZ* v1j = static_cast<const VertexSBAPointXYZ*>(_vertices[2]);
        const VertexSBAPointXYZ* v2j = static_cast<const VertexSBAPointXYZ*>(_vertices[3]);

        double obs(_measurement);
        double mu = alpha;
        double alp = beta;

        Eigen::Vector3d numerator = (v2i->estimate() - v2j->estimate());
        Eigen::Vector3d denominator = (v1i->estimate() - v1j->estimate());
        double lamdaij = numerator.norm() / denominator.norm();
        //double lambda_ij = std::sqrt(std::pow(numerator, 2) + std::pow(denominator, 2));

        ////tension in shear
        //double energy = (2 * mu / alp) * ((pow(lamdaij, alp)) - pow((1 / (lamdaij*lamdaij)), alp));

        ////tension in Planar
        //double energy = (2 * mu / alp) * ((pow(lamdaij, alp)) - pow((1 / lamdaij),alp));

        ////tension in Tensile
        //double energy = (2 * mu / alp) * ((pow(lamdaij, alp)) - pow((1 / sqrt(lamdaij)), alp));

        ///Deformation
        double energy = (mu / alp) * (2 * (pow(lamdaij, alp)) + pow((lamdaij),-(2*alp)) - 3);

        _error[0] = energy - obs;   
    }

    // virtual void linearizeOplus();

    Sophus::SO3d Ri;
    Sophus::SO3d Rj;
    double weight;
    double alpha;   // Bulk deformation parameter
    double beta;    // Shear deformation parameter
};

class EdgeTransformation: public  g2o::BaseMultiEdge<1, double>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeTransformation();

    bool read(std::istream& is);

    bool write(std::ostream& os) const;

    void computeError() {
        const VertexSBAPointXYZ* v1 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
        const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[1]);
        const g2o::VertexSE3Expmap* vT = static_cast<const g2o::VertexSE3Expmap*>(_vertices[2]);

        double obs(_measurement);
        Eigen::Vector3d diffGlobalT;

        // Eigen::Matrix3d Rg = T.rotationMatrix().cast<double>();
        // Eigen::Vector3d t = T.translation().cast<double>();

        g2o::SE3Quat T_global =  vT->estimate();
        Eigen::Matrix3d Rg = T_global.rotation().toRotationMatrix();
        Eigen::Vector3d t = T_global.translation();

        diffGlobalT = (Rg * v2->estimate() - t) - v1->estimate();

        double energyGlobalT = diffGlobalT.squaredNorm();

        _error[0] = energyGlobalT - obs;   
    }

    // virtual void linearizeOplus();

    // Sophus::SE3f T;
    double weight;
};


#endif //SLAM_G2OTYPES_H
