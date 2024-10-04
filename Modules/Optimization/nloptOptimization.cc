#include "Optimization/g2oBundleAdjustment.h"
#include "Optimization/g2oTypes.h"

#include "Utils/Geometry.h"
#include "Optimization/nloptOptimization.h"
#include "Utils/CommonTypes.h"

double outerObjective(const std::vector<double>& x, std::vector<double>& grad, void* data) {
    double repBalanceWeight = x[0];
    double arapBalanceWeight = x[1];

    std::cout << "Current x values: " << x[0] << ", " << x[1] << std::endl;

    OptimizationData* pData = static_cast<OptimizationData*>(data);
    std::shared_ptr<Map> pMapCopy = pData->pMap->clone();
    int& nOptIterations = pData->nOptIterations;
    float repErrorStanDesv = pData->repErrorStanDesv;

    arapOptimization(pMapCopy.get(), repBalanceWeight, arapBalanceWeight, nOptIterations);

    double stanDeviation = calculatePixelsStandDev(pMapCopy, cameraSelection::Combined);

    double error = std::pow(std::log(repErrorStanDesv + 1) - std::log(stanDeviation + 1), 2);
    // The +1 inside the logarithm prevents it from becoming undefined when the values are close to zero.

    std::cout << "stanDeviation: " << stanDeviation << "\n";

    std::cout << "error: " << error << "\n";
    return error;
}