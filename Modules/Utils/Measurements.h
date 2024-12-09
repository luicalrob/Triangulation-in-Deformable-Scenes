#ifndef SLAM_MEASUREMENTS_H
#define SLAM_MEASUREMENTS_H

#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <opencv2/opencv.hpp>
#include <g2o/types/sba/types_sba.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include "open3d/Open3D.h"

#include "Map/Map.h"
#include "Utils/CommonTypes.h"
#include "Visualization/FrameVisualizer.h"
#include <Visualization/MapVisualizer.h>


/*
* Measure the pos & or errors of the poses and the 3D error of the mapPoints
*/
void measureAbsoluteMapErrors(std::shared_ptr<Map> pMap, std::vector<Eigen::Vector3f> originalPoints, std::vector<Eigen::Vector3f> movedPoints, std::string filePath);

/*
* Measure the relative distances errors of the mapPoints
*/
void measureRelativeMapErrors(std::shared_ptr<Map> pMap, std::string filePath);

#endif //SLAM_MEASUREMENTS_H