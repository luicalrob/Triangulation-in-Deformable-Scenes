#include "System/SLAM.h"
#include "Utils/Measurements.h"
#include "DatasetLoader/RealcolonLoader.h"
#include "Utils/CommonTypes.h"
#include <cstdlib>

#include <opencv2/opencv.hpp>

using namespace std;

int main(int argc, char **argv){
    //Check program parameters are good
    if(argc < 2 && argc > 5){
        cerr << "[Error]: you need to invoke the program with 1 parameter: " << argc << endl;
        cerr << "\t./Realcolon <dataset_path>" << endl;
        cerr << "Or with 4 parameter: " << argc << endl;
        cerr << "\t./Realcolon <dataset_path> (int)Starting frame (int)Frames step (int)Ending frame" << endl;
        cerr << "Finishing execution..." << endl;
        return -1;
    }
    
    //Load dataset sequence
    string datasetPath = argv[1];
    RealcolonLoader sequence(datasetPath, datasetPath + "/trajectory.txt");

    int startingFrame, framesStep, endingFrame;
    if (argc >= 3) {
        try {
            startingFrame = std::stoi(argv[2]);
        } catch (const std::exception&) {
            cerr << "[Warning]: Invalid starting frame. Using default value: 0." << endl;
            startingFrame = 0;
        }

        try {
            framesStep = std::stoi(argv[3]);
        } catch (const std::exception&) {
            cerr << "[Warning]: Invalid frames step. Using default value: 1." << endl;
            framesStep = 1;
        }

        try {
            endingFrame = std::stoi(argv[4]);
        } catch (const std::exception&) {
            cerr << "[Warning]: Invalid ending frame. Using default value: " << sequence.getLength() << "." << endl;
            endingFrame = sequence.getLength();
        }
    } else {
        // Default values are already set.
        startingFrame = 0;
        framesStep = 1;
        endingFrame = sequence.getLength();
    }

    clock_t timer;
    timer = clock();
    int nMPs = 0;
    int nKF = 0;

    cv::Mat currIm;
    cv::Mat currDepthIm;
    // double currTs;
    PoseData currPose;
    currPose.tx = 0.0;
    currPose.ty = 0.0;
    currPose.tz = 0.0; 
    currPose.qx = 0.0;
    currPose.qy = 0.0;
    currPose.qz = 0.0;
    currPose.qw = 1.0;
    currPose.isValid = true;

    SLAM SLAM("Data/Realcolon.yaml", currPose);

    //for(int i = 0; i < sequence.getLenght(); i++){
    for(int i = startingFrame; i <= endingFrame; i+=framesStep){
        sequence.getRGBImage(i,currIm);
        sequence.getDepthImage(i,currDepthIm);
        // sequence.getTimeStamp(i,currTs);

        if (currIm.empty()) {
            std::cerr << "Error: Could not load image!" << std::endl;
            return -1;
        }
    
        cv::imshow("Image Window", currIm);
        Sophus::SE3f Tcw;

        cout << "[" << i <<"] TimeStamp " << endl;
        bool triangulated = SLAM.processImage(currIm, currDepthIm, Tcw, nKF, nMPs, timer);
        if(triangulated) break;
    }
    
    timer = clock() - timer;

    SLAM.stop();
	cout << "[END] Seconds: " << fixed << setprecision(4) << ((float)timer)/CLOCKS_PER_SEC << endl;
    cout << "[END] Number of Keyframes: " << nKF << endl;
    cout << "[END] Number of MapPoints: " << nMPs << endl;

    return 0;
}