#ifndef EIGEN_OPTIMIZATION_H
#define EIGEN_OPTIMIZATION_H

#include <Eigen/Dense>
#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>
#include "Optimization/EigenOptimization.h"
#include "Utils/Geometry.h"
#include "Optimization/nloptOptimization.h"
#include "Utils/CommonTypes.h"

template<typename _Scalar, int NX = Eigen::Dynamic, int NY = Eigen::Dynamic>
struct Functor
{
typedef _Scalar Scalar;
enum {
    InputsAtCompileTime = NX,
    ValuesAtCompileTime = NY
};
typedef Eigen::Matrix<Scalar,InputsAtCompileTime,1> InputType;
typedef Eigen::Matrix<Scalar,ValuesAtCompileTime,1> ValueType;
typedef Eigen::Matrix<Scalar,ValuesAtCompileTime,InputsAtCompileTime> JacobianType;

int m_inputs, m_values;

Functor() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime) {}
Functor(int inputs, int values) : m_inputs(inputs), m_values(values) {}

int inputs() const { return m_inputs; }
int values() const { return m_values; }

};

struct EigenOptimizationFunctor : Functor<double>
{
EigenOptimizationFunctor(Map* map, int iterations, double error): Functor<double>(2,2), pMap(map), nIterations(iterations), repErrorStanDesv(error) {}

Map* pMap;           // Pointer to a Map object
int nIterations;     // Number of iterations
double repErrorStanDesv; // Standard deviation of error

int operator()(const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const
{   
    arapOptimization(pMap, x(0), x(1), nIterations);
    
    double stanDeviation = calculatePixelsStandDev(pMap);

    double error = std::pow(repErrorStanDesv - stanDeviation, 2);

    std::cout << "error: " << error << "\n";

    fvec(0) = error;
    fvec(1) = 0;

    return 0;
}
};

#endif // EIGEN_OPTIMIZATION_H