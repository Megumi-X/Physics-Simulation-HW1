#ifndef UNIVERSAL_JOINT
#define UNIVERSAL_JOINT

#include "include/joint.hpp"

namespace backend {

class UniversalJoint : public Joint<3> {
public:
    UniversalJoint() : Joint<3>(2) {}

    const RigidTransform<3> ComputeChildJointToParentJointTransform(const VectorXr& dofs) const override;
    const std::vector<Matrix3r> ComputeChildJointToParentJointRotationDerivative(const VectorXr& dofs) const override;
    const std::vector<std::vector<Matrix3r>> ComputeChildJointToParentJointRotationSecondDerivative(const VectorXr& dofs) const override;
    
    // normalized_axis_parent_ and normalized_axis_child_ are rigidly attached to the parent_joint frame.
    // First, rotate along normalized_axis_child_ by an angle dofs(0);
    // Second, rotate along normalized_axis_parent_ by an angle dofs(1).
    const Vector3r& normalized_axis_child() const { return joint_axis_[1]; }
    const Vector3r& normalized_axis_parent() const { return joint_axis_[0]; }
};

TEST_JOINT3D(Universal)

}

#endif