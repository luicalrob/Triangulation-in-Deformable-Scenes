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

    std::unordered_map<ID,KeyFrame_>&  mKeyFrames = pMapCopy->getKeyFrames();

    for (auto k1 = mKeyFrames.begin(); k1 != mKeyFrames.end(); ++k1) {
        for (auto k2 = std::next(k1); k2 != mKeyFrames.end(); ++k2) {

        KeyFrame_ pKF1 = k2->second;
        KeyFrame_ pKF2 = k1->second;

        vector<MapPoint_>& v1MPs = pKF1->getMapPoints();
        vector<MapPoint_>& v2MPs = pKF2->getMapPoints();
                
        std::vector<Eigen::Vector3d> v1Positions = extractPositions(v1MPs);
        std::vector<Eigen::Vector3d> v2Positions = extractPositions(v2MPs);

        std::cout << "v1Position: (" << v1Positions[2][0] << ", " << v1Positions[2][1] << ", " << v1Positions[2][2] << ")\n";
        std::cout << "v2Position: (" << v2Positions[2][0] << ", " << v2Positions[2][1] << ", " << v2Positions[2][2] << ")\n";
        }
    }

    arapOptimization(pMapCopy.get(), repBalanceWeight, arapBalanceWeight, nOptIterations);

    double stanDeviation = calculatePixelsStandDev(pMapCopy, cameraSelection::Combined);

    double error = std::pow(repErrorStanDesv - stanDeviation, 2);

    std::cout << "stanDeviation: " << stanDeviation << "\n";

    std::cout << "error: " << error << "\n";
    return error;
}