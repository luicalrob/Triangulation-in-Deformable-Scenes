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

//class EdgeARAP : public g2o::BaseMultiEdge<1, double> {
class EdgeARAP : public g2o::BaseBinaryEdge<1, double, VertexSBAPointXYZ, VertexRotationMatrix> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeARAP();

    bool read(std::istream& is);

    bool write(std::ostream& os) const;

    void computeError() {
        // NORMAL
        // const VertexSBAPointXYZ* v1 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
        // const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[1]);
        // const VertexRotationMatrix* vR = static_cast<const VertexRotationMatrix*>(_vertices[2]);
        // const VertexTranslationVector* vT = static_cast<const VertexTranslationVector*>(_vertices[3]);

        // ONLY ONE POINT AND R
        const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
        const VertexRotationMatrix* vR = static_cast<const VertexRotationMatrix*>(_vertices[1]);

        //Eigen::Vector3d obs(_measurement);
        double obs(_measurement);
        Eigen::Vector3d diff;

        Eigen::Matrix3d R = vR->estimate();
        // Eigen::Vector3d t = vT->estimate();
        //diff = (v2->estimate() - Xj2world) - ( R * (v1->estimate() - Xj1world) + t ); // [DUDA] should i use other two vertex instead of _measurement?

        // NORMAL
        // diff = (v1->estimate() - Xj1world) - ( R * (v2->estimate() - Xj2world));

        // ONLY ONE POINT AND R
        diff = (Xi1world - Xj1world) - (R * (v2->estimate() - Xj2world));

        //Eigen::Vector3d squaredNormComponents = diff.array().square();
        double energy = diff.squaredNorm();

        // _error = weight * squaredNormComponents;
        // std::cout << "ARAP error: (" << _error[0] << ", " << _error[1] << ", " << _error[2] << ")\n" << std::endl;

        _error[0] = weight * energy;
        // std::cout << "ARAP error: " << _error[0] << "\n" << std::endl;   
        // Eigen::Vector3d error = obs - (v2->estimate() - Xj2world) - R * (v1->estimate() - Xj1world);
        // Eigen::Vector3d squaredNormComponents = error.array().square();
        // _error = weight * squaredNormComponents;     
    }

    // virtual void linearizeOplus();

    // ONLY ONE POINT AND R
    Eigen::Vector3d Xi1world;

    Eigen::Vector3d Xj1world;
    Eigen::Vector3d Xj2world;
    double weight;
};

#endif //SLAM_G2OTYPES_H
