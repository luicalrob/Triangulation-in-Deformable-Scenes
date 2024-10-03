#include "Tet.h"

// Constructor
Tet::Tet() {
    // Optionally initialize matrices or vectors here
    R.setIdentity();
    Y.setIdentity();
    W.setIdentity();
}

// Initialize values that depend on p alone
void Tet::Init() {
    // compute vol, VolGrad, we, Curl, and initialize R=Id
    ComputeCurlMatrices();
    R.setIdentity(); // Identity matrix for R
    Y.setIdentity(); // Set Y as identity initially
}

void Tet::ComputeR() {
    Eigen::Matrix3d F;
    // Deformed edges
    F.col(0) = q[1] - q[0];
    F.col(1) = q[2] - q[0];
    F.col(2) = q[3] - q[0];

    // Perform polar decomposition via SVD to compute R
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(F, Eigen::ComputeFullU | Eigen::ComputeFullV);
    R = svd.matrixU() * svd.matrixV().transpose();
}

void Tet::ComputeVolume() {
    Eigen::Vector3d v1_v0 = p[1] - p[0];
    Eigen::Vector3d v2_v0 = p[2] - p[0];
    Eigen::Vector3d v3_v0 = p[3] - p[0];

    Eigen::Vector3d cross_product = v2_v0.cross(v3_v0);

    double scalar_triple_product = v1_v0.dot(v2_v0.cross(v3_v0));

    vol = std::abs(scalar_triple_product) / 6.0;
}

// Compute volume gradient with respect to vertex p_i
void Tet::ComputeVolGrad(int i) {
    int j = (i + 1) % 4, k = (i + 2) % 4, l = (i + 3) % 4;

    Eigen::Vector3d crossProduct = (p[k] - p[l]).cross(p[j] - p[l]) / 6.0;

    VolGrad[i] = crossProduct;
}

// Compute edge weight for the Dirichlet energy
double Tet::ComputeEdgeWeight(int i, int j) {
    ComputeVolGrad(i);
    ComputeVolGrad(j);

    double weight = VolGrad[i].dot(VolGrad[j]) / vol;

    return weight;
}

// Compute Laplacian at vertex q_i
Eigen::Vector3d Tet::LaplaceQ(int i) {
    int j = (i + 1) % 4, k = (i + 2) % 4, l = (i + 3) % 4;
    return we[e(i, j)] * (q[i] - q[j]) +
           we[e(i, k)] * (q[i] - q[k]) +
           we[e(i, l)] * (q[i] - q[l]);
}

// Compute gradient of the energy with respect to vertex i
Eigen::Vector3d Tet::Grad(int i, double alpha, double beta) {
    const double a2 = alpha * alpha, b2 = beta * beta;
    return b2 * LaplaceQ(i) - (a2 - (a2 - b2) * Y.trace() / 3) * R * VolGrad[i];
}

// Precompute Curl matrices
void Tet::ComputeCurlMatrices() {
    const int i = 0, j = 1, k = 2, l = 3;
    Eigen::Vector3d pij = p[j] - p[i], plk = p[k] - p[l];
    Eigen::Vector3d pik = p[k] - p[i], pjl = p[l] - p[j];
    Eigen::Vector3d pil = p[l] - p[i], pkj = p[j] - p[k];

    Curl[i] = pij * plk.transpose() + pik * pjl.transpose() + pil * pkj.transpose();
    Curl[j] = pij * plk.transpose() + pjl * pik.transpose() + pkj * pil.transpose();
    Curl[k] = plk * pij.transpose() + pik * pjl.transpose() + pkj * pil.transpose();
    Curl[l] = plk * pij.transpose() + pjl * pik.transpose() + pil * pkj.transpose();
}

// Compute W matrix for Hessian based on current state of q
void Tet::ComputeW() {
    W = (Y.trace() * Eigen::Matrix3d::Identity() - Y).inverse();
}

// Compute second part of Hessian based on i, j
Eigen::Matrix3d Tet::HessianSecondPart(int i, int j) {
    Eigen::Matrix3d RMi = R * Curl[i], RMj = R * Curl[j];
    return RMi * W * RMj.transpose() / (36 * vol);
}

// Compute first part of Hessian based on Laplacian terms
Eigen::Matrix3d Tet::HessianFirstPart(int i, int j) {
    if (i == j) {
        return Eigen::Matrix3d::Identity() * 
               (we[e(i, (i + 1) % 4)] + we[e(i, (i + 2) % 4)] + we[e(i, (i + 3) % 4)]);
    } else {
        return -Eigen::Matrix3d::Identity() * we[e(i, j)];
    }
}

// Compute second part of Hessian's A-B split
Eigen::Matrix3d Tet::HessianSecondPartABSplit(int i, int j) {
    return (R * VolGrad[i]) * (R * VolGrad[j]).transpose() / vol;
}

// Compute the total Hessian
Eigen::Matrix3d Tet::TotalHessian(int i, int j, double alpha, double beta) {
    const double a2 = alpha * alpha, b2 = beta * beta;
    if (alpha == beta) {
        return a2 * (HessianFirstPart(i, j) - HessianSecondPart(i, j));
    } else {
        return b2 * HessianFirstPart(i, j) -
               (a2 - (a2 - b2) * Y.trace() / 3) * HessianSecondPart(i, j) +
               (a2 - b2) / 3 * HessianSecondPartABSplit(i, j);
    }
}

// Edge index mapping function
int Tet::e(int i, int j) {
    int idx = -1;
    if ((i == 0 && j == 1) || (i == 1 && j == 0)) idx = 0;
    else if ((i == 0 && j == 2) || (i == 2 && j == 0)) idx = 1;
    else if ((i == 0 && j == 3) || (i == 3 && j == 0)) idx = 2;
    else if ((i == 1 && j == 2) || (i == 2 && j == 1)) idx = 3;
    else if ((i == 1 && j == 3) || (i == 3 && j == 1)) idx = 4;
    else if ((i == 2 && j == 3) || (i == 3 && j == 2)) idx = 5;
    return idx;
}
