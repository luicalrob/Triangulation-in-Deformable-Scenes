#pragma once

#include "Map/KeyFrame.h"
#include "Map/MapPoint.h"
#include "sophus/se3.hpp"

#include <memory>

using namespace std;

typedef shared_ptr<MapPoint> MapPoint_;
typedef shared_ptr<KeyFrame> KeyFrame_;
typedef shared_ptr<Sophus::SO3d> RotationMatrix_;
typedef shared_ptr<Sophus::SE3f> TransformationMatrix_;

enum class cameraSelection {
    C1,
    C2,
    Combined
};

struct PixelsError {
    double avgc1;    // Average error camera 1
    double avgc2;    // Average error camera 2
    double avg;      // Average error across both cameras
    double desvc1;   // Standard deviation camera 1
    double desvc2;   // Standard deviation camera 2
    double desv;     // Standard deviation across both camera
};