#include "Optimization/g2oBundleAdjustment.h"
#include "Utils/Geometry.h"
#include "Optimization/nloptOptimization.h"

double outerObjective(unsigned int n, const double* x, double* grad, void* data){
    double repBalanceWeight = x[0];
    double globalBalanceWeight = x[1];
    double arapBalanceWeight = x[2];

    std::cout << "Current x values. Reprojection: " << x[0] << ", Global T: "<< x[1] << ", ARAP: "<< x[2] << std::endl;

    OptimizationData* pData = static_cast<OptimizationData*>(data);
    std::shared_ptr<Map> pMapCopy = pData->pMap->clone();
    int& nOptIterations = pData->nOptIterations;
    float repErrorStanDesv = pData->repErrorStanDesv;
    double alpha = pData->alpha;
    double beta = pData->beta;

    arapOptimization(pMapCopy.get(), repBalanceWeight, globalBalanceWeight, arapBalanceWeight, alpha, beta, nOptIterations);

    PixelsError pixelsErrors;
    calculatePixelsStandDev(pMapCopy, pixelsErrors);

    // double objetiveError1 = repErrorStanDesv - pixelsErrors.desvc1;
    // double objetiveError2 = repErrorStanDesv - pixelsErrors.desvc2;

    //assuming that repErrorStanDesv is 1 pixel
    double errorC1 = std::pow(std::log(pixelsErrors.desvc1), 2);
    double errorC2 = std::pow(std::log(pixelsErrors.desvc2), 2);
    double error = errorC1 + errorC2;
    // The +1 inside the logarithm prevents it from becoming undefined when the values are close to zero.

    // std::cout << "stanDeviation: " << pixelsErrors.desv << "\n";

    std::cout << "error: " << error << "\n";
    return error;
}