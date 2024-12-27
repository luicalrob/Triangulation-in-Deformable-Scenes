#ifndef SLAM_NLOPTOPTIMIZATION_H
#define SLAM_NLOPTOPTIMIZATION_H

#include "Map/Map.h"
#include <nlopt.hpp>

struct OptimizationData {
    std::shared_ptr<Map> pMap;
    int nOptIterations;
    float repErrorStanDesv;
    double alpha;
    double beta;
    float depthUncertainty;
    std::vector<int> vMatches;
};

//double outerObjective(const std::vector<double>& x, std::vector<double>& grad, void* data);
double outerObjective(unsigned int n, const double* x, double* grad, void* data);

#endif //SLAM_NLOPTOPTIMIZATION_H