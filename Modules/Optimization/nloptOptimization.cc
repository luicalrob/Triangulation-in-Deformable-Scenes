#include "Optimization/g2oBundleAdjustment.h"
#include "Optimization/g2oTypes.h"

#include "Utils/Geometry.h"
#include "Optimization/nloptOptimization.h"
#include "Utils/CommonTypes.h"
#include <Eigen/Dense> 
#include "Map/Map.h"

double outerObjective(const std::vector<double>& x, std::vector<double>& grad, void* data) {
    double repBalanceWeight = x[0];
    double arapBalanceWeight = x[1];

    OptimizationData* pData = static_cast<OptimizationData*>(data);
    std::shared_ptr<Map> pMapCopy = pData->pMap->clone();
    Map* pMap = new Map(*pMapCopy);
    int& nOptIterations = pData->nOptIterations;
    float repErrorStanDesv = pData->repErrorStanDesv;

    arapOptimization(pMap, repBalanceWeight, arapBalanceWeight, nOptIterations);

    //double error = calculateTotalError(Map, originalPoints, movedPoints, insertedIndexes);
    double stanDeviation = calculatePixelsStandDev(pMap);

    double error = std::pow(repErrorStanDesv - stanDeviation, 2);

    std::cout << "error: " << error << "\n";
    return error;
}
