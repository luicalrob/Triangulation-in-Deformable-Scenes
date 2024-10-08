#ifndef TRIANGLE_H
#define TRIANGLE_H

#include <Eigen/Dense>
#include <iostream>

class Triangle {
public:
    // p dependent (constant) section
    Eigen::Vector3d p[3];            // original configuration (3D points)
    Eigen::Vector3d AreaGrad[3];      // area gradient wrt vertex i=0,1,2
    double we[3];                    // edge weights of 3D Laplace
    double area;                     // area of original triangle

    // q section which needs to be updated as we go along
    Eigen::Vector3d q[3];            // current deformed configuration
    Eigen::Matrix3d R;               // optimal rotation for current q
    Eigen::Matrix3d Y;               // symmetric factor for current q
    Eigen::Matrix3d W;               // weight matrix in second part of Hessian

    // Constructor
    Triangle();

    // Member functions
    void Init();
    void ComputeR();
    void ComputeArea();
    void ComputeAreaGrad(int i);
    double ComputeEdgeWeight(int i, int j);

private:

};

#endif // TRIANGLE_H
