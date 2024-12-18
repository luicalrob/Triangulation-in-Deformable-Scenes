#include "System/SLAM.h"
#include "Utils/Measurements.h"
#include "DatasetLoader/DrunkardLoader.h"
#include "Utils/CommonTypes.h"

#include <opencv2/opencv.hpp>

using namespace std;

int main(int argc, char **argv){
    //Check program parameters are good
    if(argc != 2){
        cerr << "[Error]: you need to invoke the program with 1 parameter: " << argc << endl;
        cerr << "\t./Drunkard <dataset_path>" << endl;
        cerr << "Finishing execution..." << endl;
        return -1;
    }
    
    //Load dataset sequence
    string datasetPath = argv[1];
    DrunkardLoader sequence(datasetPath, datasetPath + "/pose.txt");

    SLAM SLAM("Data/Drunkard.yaml");

    // Eigen::Vector3f firstCamera = SLAM.getFirstCameraPos();
    // Eigen::Vector3f secondCamera = SLAM.getSecondCameraPos();

    // SLAM.loadPoints("Data/original_points.csv", "Data/moved_points.csv");

    // SLAM.setCameraPoses(firstCamera, secondCamera);

    // SLAM.getSimulatedDepthMeasurements();

    // SLAM.createKeyPoints();

    clock_t timer;
    timer = clock();
    int nMPs = 0;
    int nKF = 0;

    cv::Mat currIm;
    double currTs;
    PoseData currPose;
    //for(int i = 0; i < sequence.getLenght(); i++){
    for(int i = 0; i < 30; i++){
        sequence.getRGBImage(i,currIm);
        sequence.getTimeStamp(i,currTs);
        sequence.getPoseData(i,currPose);

        Sophus::SE3f Tcw;
        Eigen::Vector3f translation(currPose.tx, currPose.ty, currPose.tz);
        Eigen::Quaternionf quaternion(currPose.qw, currPose.qx, currPose.qy, currPose.qz);
        quaternion.normalize();
        Tcw = Sophus::SE3f(quaternion, translation);

        cout << "[" << i <<"] TimeStamp: " << currTs << endl;
        cout << "Translation: " << translation[0]  << " " << translation[1]  << " "  << translation[2] << " " << endl;
        cout << "Rotation: "
                    << "[w: " << quaternion.w()
                    << ", x: " << quaternion.x()
                    << ", y: " << quaternion.y()
                    << ", z: " << quaternion.z() << "]" << endl;
        SLAM.processImage(currIm, Tcw, nKF, nMPs, timer);
    }
    
    timer = clock() - timer;

    SLAM.stop();
	cout << "[END] Seconds: " << fixed << setprecision(4) << ((float)timer)/CLOCKS_PER_SEC << endl;
    cout << "[END] Number of Keyframes: " << nKF << endl;
    cout << "[END] Number of MapPoints: " << nMPs << endl;

    return 0;
}