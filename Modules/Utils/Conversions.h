#ifndef SLAM_CONVERSIONS_H
#define SLAM_CONVERSIONS_H

#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <opencv2/opencv.hpp>



cv::Mat convertSE3fToMat(const Sophus::SE3f& se3);


cv::Mat convertVector3fToMat(const Eigen::Vector3f& vec);


Eigen::Vector3f convertMatToVector3f(const cv::Mat& mat); 


Eigen::Matrix<float, 3, 4> computeProjection(const Sophus::SE3f& T, const Eigen::Matrix3f& K_c);


double roundToDecimals(double value, int decimals);

#endif //SLAM_CONVERSIONS_H