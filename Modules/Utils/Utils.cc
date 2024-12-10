#include "Utils/Geometry.h"
#include "Utils/Measurements.h"

#include <random> 

using namespace std;

void stopExecution(
    std::shared_ptr<MapVisualizer> mapVisualizer,
    bool drawRaysSelection) 
{
    std::cout << "Press esc to continue... " << std::endl;
    while ((cv::waitKey(10) & 0xEFFFFF) != 27) {
        mapVisualizer->update(drawRaysSelection);
    }
}