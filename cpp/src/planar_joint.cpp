#include "include/planar_joint.hpp"

namespace backend {

const RigidTransform<3> PlanarJoint::ComputeChildJointToParentJointTransform(const VectorXr& dofs) const {
    
    return RigidTransform<3>(Matrix3r::Identity(), dofs(0) * joint_axis_[0] + dofs(1) * joint_axis_[1]);
}


const Matrix3Xr PlanarJoint::ComputeChildJointToParentJointTranslationDerivative(const VectorXr& dofs) const {
    // TODO.
    // return Joint<3>::ComputeChildJointToParentJointTranslationDerivative(dofs);
    ////////////////////////////////////////////
    Eigen::Matrix<real, 3, 2> ret;
    ret.col(0) = joint_axis_[0].normalized(); 
    ret.col(1) = joint_axis_[1].normalized();
    return ret;
}

}