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

    PixelsError pixelsErrors;
    calculatePixelsStandDev(pMapCopy, pixelsErrors);

    // double objetiveError1 = repErrorStanDesv - pixelsErrors.desvc1;
    // double objetiveError2 = repErrorStanDesv - pixelsErrors.desvc2;

    //assuming that repErrorStanDesv is 1 pixel
    double errorC1 = std::pow(std::log(pixelsErrors.desvc1), 2);
    double errorC2 = std::pow(std::log(pixelsErrors.desvc2), 2);
    double error = errorC1 + errorC2;
    // The +1 inside the logarithm prevents it from becoming undefined when the values are close to zero.

    std::cout << "stanDeviation: " << pixelsErrors.desv << "\n";

    std::cout << "error: " << error << "\n";
    return error;
}