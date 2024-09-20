#include "Optimization/EigenOptimization.h"
#include "Utils/Geometry.h"
#include "Optimization/nloptOptimization.h"
#include "Utils/CommonTypes.h"

#include <cassert>

EigenOptimizationFunctor::EigenOptimizationFunctor(Map* map, int nOptIterations, double repStanDesv)
    : m_inputs(2), m_values(1), pMap(map), nIterations(nOptIterations), repErrorStanDesv(repStanDesv) {}

int EigenOptimizationFunctor::operator()(const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const {
    assert(x.size() == m_inputs);
    assert(fvec.size() == m_values);
    std::cout << "Operator called with x: " << x.transpose() << "\n";
    fvec[0] = calculateResidual(x[0], x[1]);
    return 0; // Return 0 on success
}

double EigenOptimizationFunctor::calculateResidual(double repWeight, double arapWeight) const {

    // Assuming these functions are defined elsewhere
    arapOptimization(pMap, repWeight, arapWeight, nIterations);
    
    double stanDeviation = calculatePixelsStandDev(pMap);

    double error = std::pow(repErrorStanDesv - stanDeviation, 2);

    std::cout << "error: " << error << "\n";
    return error;
}

int EigenOptimizationFunctor::inputs() const { return m_inputs; }

int EigenOptimizationFunctor::values() const { return m_values; }

