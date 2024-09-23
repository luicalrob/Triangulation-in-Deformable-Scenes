#include "Optimization/g2oBundleAdjustment.h"
#include "Optimization/g2oTypes.h"

#include "Utils/Geometry.h"
#include "Optimization/nloptOptimization.h"
#include "Utils/CommonTypes.h"

double outerObjective(const std::vector<double>& x, std::vector<double>& grad, void* data) {
    double repBalanceWeight = x[0];
    double arapBalanceWeight = x[1];

    OptimizationData* pData = static_cast<OptimizationData*>(data);
    Map* Map = pData->pMap;
    std::vector<Eigen::Vector3f>& originalPoints = pData->originalPoints;
    std::vector<Eigen::Vector3f>& movedPoints = pData->movedPoints;
    std::vector<int>& insertedIndexes = pData->insertedIndexes;
    int& nOptIterations = pData->nOptIterations;
    float repErrorStanDesv = pData->repErrorStanDesv;


    arapOptimization(Map, repBalanceWeight, arapBalanceWeight, nOptIterations);

    //double error = calculateTotalError(Map, originalPoints, movedPoints, insertedIndexes);
    double stanDeviation = calculatePixelsStandDev(Map, cameraSelection::Combined);

    double error = std::pow(repErrorStanDesv - stanDeviation, 2);

    std::cout << "error: " << error << "\n";
    return error;
}

double calculateTotalError(Map* Map, std::vector<Eigen::Vector3f> originalPoints, std::vector<Eigen::Vector3f> movedPoints, std::vector<int> insertedIndexes) {
    double total_error = 0.0f;

    for(size_t i = 0, j = 0; j < insertedIndexes.size(); i += 2, j++) {
        std::shared_ptr<MapPoint> mapPoint1 = Map->getMapPoint(i);
        std::shared_ptr<MapPoint> mapPoint2 = Map->getMapPoint(i+1);
        Eigen::Vector3f opt_original_position = mapPoint1->getWorldPosition();
        Eigen::Vector3f opt_moved_position = mapPoint2->getWorldPosition();

        Eigen::Vector3f original_position = originalPoints[insertedIndexes[j]];
        Eigen::Vector3f moved_position = movedPoints[insertedIndexes[j]];

        Eigen::Vector3f original_error = opt_original_position - original_position;
        Eigen::Vector3f moved_error = opt_moved_position - moved_position;
        double error_magnitude_original = static_cast<double>(original_error.norm());
        double error_magnitude_moved = static_cast<double>(moved_error.norm());
        total_error += error_magnitude_moved + error_magnitude_original;
    }

    // std::cout << "point_count: " << point_count << std::endl;
    return total_error;
}
