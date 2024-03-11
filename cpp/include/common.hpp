#ifndef COMMON
#define COMMON

#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include "Eigen/Dense"

namespace backend {

using real = double;        // float or double.
using integer = int32_t;    // int32_t or int64_t.

// Eigen matrices and vectors. 
using Matrix2r = Eigen::Matrix<real, 2, 2>;
using Matrix3r = Eigen::Matrix<real, 3, 3>;
using Matrix2Xr = Eigen::Matrix<real, 2, Eigen::Dynamic>;
using Matrix3Xr = Eigen::Matrix<real, 3, Eigen::Dynamic>;
using MatrixXr = Eigen::Matrix<real, Eigen::Dynamic, Eigen::Dynamic>;


using VectorXr = Eigen::Matrix<real, Eigen::Dynamic, 1>;
using Vector1r = Eigen::Matrix<real, 1, 1>;
using Vector2r = Eigen::Matrix<real, 2, 1>;
using Vector3r = Eigen::Matrix<real, 3, 1>;

void Assert(const bool condition, const std::string& location, const std::string& message);

template<integer dim>
const VectorXr FromSkewSymmetricMatrix(const Eigen::Matrix<real, dim, dim>& A);
template<integer dim>
const Eigen::Matrix<real, dim, dim> ToSkewSymmetricMatrix(const VectorXr& a);
template<integer dim>
const VectorXr Cross(const Eigen::Matrix<real, dim, 1>& a, const Eigen::Matrix<real, dim, 1>& b);

}

#endif