#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>

// Function to compute cotangent of an angle given three vertices
double cotangent(const Eigen::Vector3d &v0, const Eigen::Vector3d &v1, const Eigen::Vector3d &v2) {
    Eigen::Vector3d e0 = v1 - v0;
    Eigen::Vector3d e1 = v2 - v1;
    Eigen::Vector3d e2 = v0 - v2;

    double cosTheta = -e2.dot(e0) / (e2.norm() * e0.norm());
    return cosTheta / sqrt(1.0 - cosTheta * cosTheta);
}

// Simple k-NN function (not optimized for large datasets)
std::vector<int> kNearestNeighbors(const Eigen::MatrixXd &points, int index, int k) {
    std::vector<std::pair<double, int>> distances;
    Eigen::Vector3d p = points.row(index).transpose(); // Ensure p is treated as a column vector
    for (int i = 0; i < points.rows(); ++i) {
        if (i != index) {
            double dist = (points.row(i).transpose() - p).norm(); // Ensure points.row(i) is also treated as a column vector
            distances.push_back({dist, i});
        }
    }
    std::sort(distances.begin(), distances.end());
    std::vector<int> neighbors;
    for (int i = 0; i < k; ++i) {
        neighbors.push_back(distances[i].second);
    }
    return neighbors;
}

// Compute cotangent weights for all points in a set of 3D points
void computeCotangentWeights(
    const Eigen::MatrixXd &points, // 3D points (n x 3)
    int k, // Number of nearest neighbors
    std::vector<Eigen::Triplet<double>> &tripletList // Output cotangent weights
) {
    for (int i = 0; i < points.rows(); ++i) {
        std::vector<int> neighbors = kNearestNeighbors(points, i, k);

        // Form triangles using the point and pairs of its neighbors
        for (int j = 0; j < neighbors.size(); ++j) {
            for (int l = j + 1; l < neighbors.size(); ++l) {
                int v0 = i;
                int v1 = neighbors[j];
                int v2 = neighbors[l];

                Eigen::Vector3d p0 = points.row(v0).transpose();
                Eigen::Vector3d p1 = points.row(v1).transpose();
                Eigen::Vector3d p2 = points.row(v2).transpose();

                double cotAlpha = cotangent(p0, p1, p2);
                double cotBeta = cotangent(p1, p2, p0);
                double cotGamma = cotangent(p2, p0, p1);

                tripletList.push_back(Eigen::Triplet<double>(v0, v1, cotGamma));
                tripletList.push_back(Eigen::Triplet<double>(v1, v0, cotGamma));
                tripletList.push_back(Eigen::Triplet<double>(v1, v2, cotAlpha));
                tripletList.push_back(Eigen::Triplet<double>(v2, v1, cotAlpha));
                tripletList.push_back(Eigen::Triplet<double>(v2, v0, cotBeta));
                tripletList.push_back(Eigen::Triplet<double>(v0, v2, cotBeta));
            }
        }
    }
}

int main() {
    // Example usage with 3D points
    Eigen::MatrixXd points(5, 3);
    points << 0, 0, 0,
              1, 0, 0,
              0, 1, 0,
              0, 0, 1,
              1, 1, 1;

    int k = 3; // Number of nearest neighbors
    std::vector<Eigen::Triplet<double>> tripletList;

    computeCotangentWeights(points, k, tripletList);

    // Construct the sparse matrix from triplets
    Eigen::SparseMatrix<double> L(points.rows(), points.rows());
    L.setFromTriplets(tripletList.begin(), tripletList.end());

    // Output or further process L
    std::cout << "Cotangent Laplacian matrix L:\n" << Eigen::MatrixXd(L) << std::endl;

    return 0;
}
