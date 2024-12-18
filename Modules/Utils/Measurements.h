#ifndef SLAM_MEASUREMENTS_H
#define SLAM_MEASUREMENTS_H

#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <opencv2/opencv.hpp>

#include "Map/Map.h"
#include "Utils/CommonTypes.h"
#include "Visualization/FrameVisualizer.h"
#include <Visualization/MapVisualizer.h>


/*
* Measure the 3D error of the mapPoints in simulated dataset Experiments
*/
void measureSimAbsoluteMapErrors(const std::shared_ptr<Map> pMap, 
                                const std::vector<Eigen::Vector3f> originalPoints, const std::vector<Eigen::Vector3f> movedPoints, 
                                const std::string filePath);

/*
* Measure the 3D error of the mapPoints in real dataset Experiments
*/
void measureRealAbsoluteMapErrors(const std::shared_ptr<Map> pMap, const std::string filePath);

/*
/*
* Measure the relative distances errors of the mapPoints
*/
void measureRelativeMapErrors(std::shared_ptr<Map> pMap, std::string filePath);

#endif //SLAM_MEASUREMENTS_H