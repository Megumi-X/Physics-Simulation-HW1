#include "include/translational_joint.hpp"

namespace backend {

template<integer dim>
const RigidTransform<dim> TranslationalJoint<dim>::ComputeChildJointToParentJointTransform(const VectorXr& dofs) const {
    return RigidTransform<dim>(Eigen::Matrix<real, dim, dim>::Identity(), dofs);
}

template<integer dim>
const Eigen::Matrix<real, dim, Eigen::Dynamic> TranslationalJoint<dim>::ComputeChildJointToParentJointTranslationDerivative(const VectorXr& dofs) const {
    return Eigen::Matrix<real, dim, dim>::Identity();
}

}