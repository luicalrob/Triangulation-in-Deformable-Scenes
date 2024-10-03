#ifndef TET_H
#define TET_H

#include <Eigen/Dense>
#include <iostream>


class Tet {
public:
    // p dependent (i.e., constant) section
    Eigen::Vector3d p[4];            // original configuration
    Eigen::Vector3d VolGrad[4];      // vol grad wrt vertex i=0,1,2,3
    double we[6];                    // edge weights of 3D Laplace
    double vol;                      // volume of original tet
    Eigen::Matrix3d Curl[4];         // Precomputed opposing edge outer product sums

    // q section which needs to be updated as we go along
    Eigen::Vector3d q[4];            // current deformed configuration
    Eigen::Matrix3d R;               // optimal rotation for current q
    Eigen::Matrix3d Y;               // symmetric factor for current q
    Eigen::Matrix3d W;               // weight matrix in second part of Hessian

    // Constructor
    Tet();

    // Member functions
    void Init();
    void ComputeR();
    void ComputeVolume();
    void ComputeVolGrad(int i);
    double ComputeEdgeWeight(int i, int j);
    Eigen::Vector3d LaplaceQ(int i);
    Eigen::Vector3d Grad(int i, double alpha, double beta);
    void ComputeCurlMatrices();
    void ComputeW();
    Eigen::Matrix3d HessianSecondPart(int i, int j);
    Eigen::Matrix3d HessianFirstPart(int i, int j);
    Eigen::Matrix3d HessianSecondPartABSplit(int i, int j);
    Eigen::Matrix3d TotalHessian(int i, int j, double alpha, double beta);

private:
    // Helper function to map edge (i,j) to an index in we[6]
    int e(int i, int j);
};

#endif // TET_H
