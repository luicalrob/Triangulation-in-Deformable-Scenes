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
struct Functor {
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

struct EigenOptimizationFunctor : Functor<double> {
    EigenOptimizationFunctor(std::shared_ptr<Map> map, int iterations, double error): Functor<double>(2,2), pMap(map), nIterations(iterations), repErrorStanDesv(error) {}

    std::shared_ptr<Map> pMap;           // Pointer to a Map object
    int nIterations;     // Number of iterations
    double repErrorStanDesv; // Standard deviation of error

    int operator()(const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const {   
        std::shared_ptr<Map> pMapCopy = pMap->clone();

        std::cout << "Current x values: " << x[0] << ", " << x[1] << std::endl;

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


        arapOptimization(pMapCopy.get(), x(0), x(1), nIterations);
        
        double stanDeviation1 = calculatePixelsStandDev(pMapCopy, cameraSelection::C1);
        double stanDeviation2 = calculatePixelsStandDev(pMapCopy, cameraSelection::C2);
        std::cout << "stanDeviation1: " << stanDeviation1 << "\n";
        std::cout << "stanDeviation2: " << stanDeviation2 << "\n";

        double errorC1 = std::pow(repErrorStanDesv - stanDeviation1, 2);
        double errorC2 = std::pow(repErrorStanDesv - stanDeviation2, 2);

        std::cout << "errorC1: " << errorC1 << "\n";
        std::cout << "errorC2: " << errorC2 << "\n";

        fvec(0) = errorC1;
        fvec(1) = errorC2;

        return 0;
    }
};

#endif // EIGEN_OPTIMIZATION_H