#include "include/prismatic_joint.hpp"

namespace backend {

template<integer dim>
const RigidTransform<dim> PrismaticJoint<dim>::ComputeChildJointToParentJointTransform(const VectorXr& dofs) const {
    return RigidTransform<dim>(Eigen::Matrix<real, dim, dim>::Identity(), Joint<dim>::joint_axis_[0] * dofs(0));
}

template<integer dim>
const Eigen::Matrix<real, dim, Eigen::Dynamic> PrismaticJoint<dim>::ComputeChildJointToParentJointTranslationDerivative(const VectorXr& dofs) const {
    // TODO.
    // return Joint<dim>::ComputeChildJointToParentJointTranslationDerivative(dofs);
    ////////////////////////////////////////////
    return Joint<dim>::joint_axis_[0];
}

template class PrismaticJoint<2>;
template class PrismaticJoint<3>;

}
