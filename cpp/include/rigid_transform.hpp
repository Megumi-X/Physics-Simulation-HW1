#ifndef RIGID_TRANSFORM
#define RIGID_TRANSFORM

#include "include/common.hpp"

namespace backend {

template<integer dim>
class RigidTransform {
public:
    RigidTransform();
    RigidTransform(const Eigen::Matrix<real, dim, dim>& rotation, const Eigen::Matrix<real, dim, 1>& translation);

    const Eigen::Matrix<real, dim, dim>& rotation() const { return rotation_; }
    const Eigen::Matrix<real, dim, 1>& translation() const { return translation_; }

    const RigidTransform<dim> Inverse() const;

private:
    Eigen::Matrix<real, dim, dim> rotation_;
    Eigen::Matrix<real, dim, 1> translation_;
};

template<integer dim>
const RigidTransform<dim> operator*(const RigidTransform<dim>& t1, const RigidTransform<dim>& t2);

template<integer dim>
std::ostream& operator<<(std::ostream& out, const RigidTransform<dim>& t);

}

#endif