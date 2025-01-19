#include "System/SLAM.h"

#include <opencv2/opencv.hpp>

using namespace std;

int main(){
    SLAM SLAM("Data/Simulation.yaml");

    Eigen::Vector3f firstCamera = SLAM.getFirstCameraPos();
    Eigen::Vector3f secondCamera = SLAM.getSecondCameraPos();

    SLAM.loadPoints("Data/original_points.csv", "Data/moved_points.csv");

    SLAM.setCameraPoses(firstCamera, secondCamera);

    SLAM.getSimulatedDepthMeasurements();

    SLAM.createKeyPoints();

    clock_t timer;
    int nMPs = 0;

    // To visualize solution
    if(SLAM.showSolution_) {
        SLAM.viusualizeSolution();
    } else {
        timer = clock();

        SLAM.processSimulatedImage(nMPs, timer);

        timer = clock() - timer;
    }

    SLAM.stop();

    cout << "[END] Seconds: " << fixed << setprecision(4) << ((float)timer)/CLOCKS_PER_SEC << endl;
    cout << "[END] Number of MapPoints: " << nMPs << endl;

    return 0;
}