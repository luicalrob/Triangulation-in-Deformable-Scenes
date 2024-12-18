#include "Utils/Geometry.h"
#include "Utils/Measurements.h"

#include <random> 

using namespace std;

void stopExecution(const std::shared_ptr<MapVisualizer>& mapVisualizer, Sophus::SE3f Tcw, bool drawRaysSelection) {
    std::cout << "Press esc to continue... " << std::endl;
    while ((cv::waitKey(10) & 0xEFFFFF) != 27) {
        mapVisualizer->update(drawRaysSelection);
        mapVisualizer->updateCurrentPose(Tcw);
    }
}

void stopWithMeasurements(const std::shared_ptr<Map>& pMap, Sophus::SE3f Tcw,
                        const std::shared_ptr<MapVisualizer> mapVisualizer, const std::string filePath, bool drawRaysSelection, bool stop, bool showScene,
                        const std::vector<Eigen::Vector3f> originalPoints, const std::vector<Eigen::Vector3f> movedPoints){
    measureRelativeMapErrors(pMap, filePath);
    if(originalPoints.empty() || movedPoints.empty()) {
        measureRealAbsoluteMapErrors(pMap, filePath);
    } else {
        measureSimAbsoluteMapErrors(pMap, originalPoints, movedPoints, filePath);
    }

    if (stop) {
        stopExecution(mapVisualizer, Tcw, drawRaysSelection);
    } else {
        if(showScene) {
            mapVisualizer->update(drawRaysSelection);
            mapVisualizer->updateCurrentPose(Tcw);
        }
    }
}