#ifndef SLAM_UTILS_H
#define SLAM_UTILS_H

#include "Map/Map.h"
#include "Utils/CommonTypes.h"
#include "Visualization/FrameVisualizer.h"
#include <Visualization/MapVisualizer.h>


/*
* Stop the execution
*/
void stopExecution(const std::shared_ptr<MapVisualizer>& mapVisualizer, Sophus::SE3f Tcw, bool drawRaysSelection);

/*
* The function used to stop using configuration parameters
*/
void stopWithMeasurements(const std::shared_ptr<Map>& pMap, Sophus::SE3f Tcw,
                        const std::shared_ptr<MapVisualizer> mapVisualizer, const std::string filePath, bool drawRaysSelection, bool stop,
                        const std::vector<Eigen::Vector3f> originalPoints = {}, const std::vector<Eigen::Vector3f> movedPoints ={});

#endif //SLAM_UTILS_H