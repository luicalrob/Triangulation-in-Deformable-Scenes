#include "Utils/Conversions.h"

#include <random> 

using namespace std;


#include <sophus/se3.hpp>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>

cv::Mat convertSE3fToMat(const Sophus::SE3f& se3) {
    cv::Mat mat = cv::Mat::eye(4, 4, CV_32F);

    Eigen::Matrix3f rotation = se3.rotationMatrix();
    Eigen::Vector3f translation = se3.translation();

    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            mat.at<float>(i, j) = rotation(i, j);

    for (int i = 0; i < 3; ++i)
        mat.at<float>(i, 3) = translation(i);

    return mat;
}


cv::Mat convertVector3fToMat(const Eigen::Vector3f& vec) {
    cv::Mat mat(3, 1, CV_32F);

    for (int i = 0; i < 3; ++i) {
        mat.at<float>(i, 0) = vec(i);
    }

    return mat;
}

Eigen::Vector3f convertMatToVector3f(const cv::Mat& mat) {
    assert((mat.rows == 3 && mat.cols == 1) || (mat.rows == 1 && mat.cols == 3));
    assert(mat.type() == CV_32F);

    Eigen::Vector3f vec;

    if (mat.rows == 3 && mat.cols == 1) {
        vec(0) = mat.at<float>(0, 0);
        vec(1) = mat.at<float>(1, 0);
        vec(2) = mat.at<float>(2, 0);
    }
    else if (mat.rows == 1 && mat.cols == 3) {
        vec(0) = mat.at<float>(0, 0);
        vec(1) = mat.at<float>(0, 1);
        vec(2) = mat.at<float>(0, 2);
    }

    return vec;
}

Eigen::Matrix<float, 3, 4> computeProjection(const Sophus::SE3f& T, const Eigen::Matrix3f& K_c) {
    Eigen::Matrix<float, 3, 4> P_aux;
    P_aux.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity();
    P_aux.block<3, 1>(0, 3) = Eigen::Vector3f(0, 0, 0);

    Eigen::Matrix<float, 3, 4> P = K_c * P_aux * T.matrix();

    return P;
}

double roundToDecimals(double value, int decimals) {
    double factor = std::pow(10.0, decimals);
    return std::round(value * factor) / factor;
}
