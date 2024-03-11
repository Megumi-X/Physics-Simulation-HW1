#ifndef HINGE_JOINT
#define HINGE_JOINT

#include "include/joint.hpp"

namespace backend {

template<integer dim>
class HingeJoint;

template<>
class HingeJoint<2> : public Joint<2> {
public:
    HingeJoint() : Joint<2>(1) {}

    const RigidTransform<2> ComputeChildJointToParentJointTransform(const VectorXr& dofs) const override;
    const std::vector<Matrix2r> ComputeChildJointToParentJointRotationDerivative(const VectorXr& dofs) const override;
    const std::vector<std::vector<Matrix2r>> ComputeChildJointToParentJointRotationSecondDerivative(const VectorXr& dofs) const override;
};

template<>
class HingeJoint<3> : public Joint<3> {
public:
    HingeJoint() : Joint<3>(1) {}

    const RigidTransform<3> ComputeChildJointToParentJointTransform(const VectorXr& dofs) const override;
    const std::vector<Matrix3r> ComputeChildJointToParentJointRotationDerivative(const VectorXr& dofs) const override;
    const std::vector<std::vector<Matrix3r>> ComputeChildJointToParentJointRotationSecondDerivative(const VectorXr& dofs) const override;
    const Vector3r& normalized_axis() const { return joint_axis_[0]; }
};

TEST_JOINT(Hinge)

}

#endif