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
    std::shared_ptr<Map> pMap;
    int nOptIterations;
    float repErrorStanDesv;
};

//double outerObjective(const std::vector<double>& x, std::vector<double>& grad, void* data);
double outerObjective(unsigned int n, const double* x, double* grad, void* data);

#endif //SLAM_NLOPTOPTIMIZATION_H