#ifndef SLAM_UTILS_H
#define SLAM_UTILS_H

#include "Map/Map.h"
#include "Utils/CommonTypes.h"
#include "Visualization/FrameVisualizer.h"
#include <Visualization/MapVisualizer.h>


/*
* Stop the execution
*/
void stopExecution(std::shared_ptr<MapVisualizer>& mapVisualizer, Sophus::SE3f Tcw, bool drawRaysSelection);

/*
* The function used to stop using configuration parameters
*/
void stopWithMeasurements(std::shared_ptr<Map>& pMap, Sophus::SE3f Tcw, std::shared_ptr<MapVisualizer> mapVisualizer, 
                         std::string filePath, bool drawRaysSelection, bool stop, bool showScene);

#endif //SLAM_UTILS_H