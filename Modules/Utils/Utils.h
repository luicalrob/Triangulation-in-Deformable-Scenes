#ifndef SLAM_UTILS_H
#define SLAM_UTILS_H

#include "Map/Map.h"
#include "Utils/CommonTypes.h"
#include "Visualization/FrameVisualizer.h"
#include <Visualization/MapVisualizer.h>


/*
* Stop the execution
*/
void stopExecution(std::shared_ptr<MapVisualizer> mapVisualizer, bool drawRaysSelection);

#endif //SLAM_UTILS_H