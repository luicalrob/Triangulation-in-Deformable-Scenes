#ifndef SLAM_NLOPTOPTIMIZATION_H
#define SLAM_NLOPTOPTIMIZATION_H

#include "System/Settings.h"

#include "Visualization/FrameVisualizer.h"
#include "Visualization/MapVisualizer.h"

#include "Tracking/Frame.h"
#include "Map/KeyFrame.h"
#include "Map/Map.h"

#include "sophus/se3.hpp"

#include <opencv2/opencv.hpp>
#include <nlopt.hpp>

struct OptimizationData {
    Map* pMap;
    std::vector<Eigen::Vector3f> originalPoints;
    std::vector<Eigen::Vector3f> movedPoints;
    std::vector<int> insertedIndexes;
    int nOptIterations;
    float repErrorStanDesv;
};

double outerObjective(const std::vector<double>& x, std::vector<double>& grad, void* data);

double calculateTotalError(Map* Map, std::vector<Eigen::Vector3f> originalPoints, std::vector<Eigen::Vector3f> movedPoints, std::vector<int> insertedIndexes);

#endif //SLAM_NLOPTOPTIMIZATION_H