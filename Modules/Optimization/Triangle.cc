#include "Triangle.h"

// Constructor
Triangle::Triangle() {
    // Initialize matrices or vectors here
    R.setIdentity();
    Y.setIdentity();
    W.setIdentity();
}

// Initialize values that depend on p alone
void Triangle::Init() {
    R.setIdentity();
    Y.setIdentity();
}

void Triangle::ComputeR() {
    Eigen::Matrix3d F;
    // Deformed edges (compute edge vectors of the deformed triangle)
    F.col(0) = q[1] - q[0];
    F.col(1) = q[2] - q[0];

    // Perform polar decomposition via SVD to compute R
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(F, Eigen::ComputeFullU | Eigen::ComputeFullV);
    
    R = svd.matrixU() * svd.matrixV().transpose();

    // Ensure that R has a determinant of +1 (i.e., a proper rotation)
    if (R.determinant() < 0) {
        // If det(R) is -1, flip the third column of U to make det(R) +1
        Eigen::Matrix3d U = svd.matrixU();
        U.col(2) *= -1;
        R = U * svd.matrixV().transpose();
    }
}

void Triangle::ComputeArea() {
    Eigen::Vector3d v1_v0 = p[1] - p[0];
    Eigen::Vector3d v2_v0 = p[2] - p[0];

    // Compute the area using the magnitude of the cross product in 3D
    area = 0.5 * v1_v0.cross(v2_v0).norm();
}

// Compute area gradient with respect to vertex p_i
void Triangle::ComputeAreaGrad(int i) {
    int j = (i + 1) % 3, k = (i + 2) % 3;

    Eigen::Vector3d edge1 = p[j] - p[k];
    Eigen::Vector3d edge2 = p[i] - p[k];
    AreaGrad[i] = edge1.cross(edge2).normalized() / 2.0; // Normalize to unit vector
}

// Compute edge weight for the Dirichlet energy
double Triangle::ComputeEdgeWeight(int i, int j) {
    ComputeAreaGrad(i);
    ComputeAreaGrad(j);

    double weight = AreaGrad[i].dot(AreaGrad[j]) / area;

    return weight;
}
