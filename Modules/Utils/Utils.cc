#include "Utils/Geometry.h"
#include "Utils/Measurements.h"

#include <random> 

using namespace std;

void stopExecution(std::shared_ptr<MapVisualizer>& mapVisualizer, Sophus::SE3f Tcw, bool drawRaysSelection) {
    std::cout << "Press esc to continue... " << std::endl;
    while ((cv::waitKey(10) & 0xEFFFFF) != 27) {
        mapVisualizer->update(drawRaysSelection);
        mapVisualizer->updateCurrentPose(Tcw);
    }
}

void stopWithMeasurements(std::shared_ptr<Map>& pMap, Sophus::SE3f Tcw, std::shared_ptr<MapVisualizer> mapVisualizer, 
                         std::string filePath, bool drawRaysSelection, bool stop, bool showScene){
    measureRelativeMapErrors(pMap, filePath);
    //measureAbsoluteMapErrors(pMap_, originalPoints_, movedPoints_, filePath_);

    if (stop) {
        stopExecution(mapVisualizer, Tcw, drawRaysSelection);
    } else {
        if(showScene) {
            mapVisualizer->update(drawRaysSelection);
            mapVisualizer->updateCurrentPose(Tcw);
        }
    }
}