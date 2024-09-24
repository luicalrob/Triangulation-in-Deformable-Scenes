#pragma once

#include "Map/KeyFrame.h"
#include "Map/MapPoint.h"
#include "Map/Map.h"


#include <memory>

using namespace std;

typedef shared_ptr<MapPoint> MapPoint_;
typedef shared_ptr<KeyFrame> KeyFrame_;
typedef shared_ptr<Eigen::Matrix3d> RotationMatrix_;
typedef shared_ptr<Eigen::Vector3d> TranslationVector_;

enum class cameraSelection {
    C1,
    C2,
    Combined
};