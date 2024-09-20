#ifndef EIGEN_OPTIMIZATION_H
#define EIGEN_OPTIMIZATION_H

#include <Eigen/Core>
#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>
#include "Map/Map.h"

// template<typename _Scalar, int NX = Eigen::Dynamic, int NY = Eigen::Dynamic>
// struct Functor
// {
//     typedef _Scalar Scalar;
//     enum {
//         InputsAtCompileTime = NX,
//         ValuesAtCompileTime = NY
//     };
//     typedef Eigen::Matrix<Scalar,InputsAtCompileTime,1> InputType;
//     typedef Eigen::Matrix<Scalar,ValuesAtCompileTime,1> ValueType;
//     typedef Eigen::Matrix<Scalar,ValuesAtCompileTime,InputsAtCompileTime> JacobianType;

//     int m_inputs, m_values;

//     Functor() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime) {}
//     Functor(int inputs, int values) : m_inputs(inputs), m_values(values) {}

//     int inputs() const { return m_inputs; }
//     int values() const { return m_values; }

// };

struct EigenOptimizationFunctor {
    // Typedefs required by Eigen::NumericalDiff
    typedef double Scalar; // The scalar type (e.g., double)
    typedef Eigen::VectorXd InputType; // The type of the input (parameters vector)
    typedef Eigen::VectorXd ValueType; // The type of the output (residuals vector)

    enum {
        InputsAtCompileTime = Eigen::Dynamic,
        ValuesAtCompileTime = Eigen::Dynamic
    };

    int m_inputs; // Number of inputs
    int m_values; // Number of residuals

    EigenOptimizationFunctor(Map* map, int nOptIterations, double repStanDesv);

    // Method required by Eigen::LevenbergMarquardt
    int operator()(const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const;

    // Jacobian computation
    int df(const Eigen::VectorXd &x, Eigen::MatrixXd &jacobian) const;

    // Custom method
    double calculateResidual(double repWeight, double arapWeight) const;

    Map* pMap;
    int nIterations;
    double repErrorStanDesv;

    typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VectorType;
    typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> JacobianType;
    typedef Eigen::JacobiSVD<JacobianType> QRSolver;

    int inputs() const;
    int values() const;
};

#endif // EIGEN_OPTIMIZATION_H

