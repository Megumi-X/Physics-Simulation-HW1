#ifndef BALL_JOINT
#define BALL_JOINT

#include "include/joint.hpp"

namespace backend {

class BallJoint : public Joint<3> {
public:
    BallJoint() : Joint<3>(3) {}

    const RigidTransform<3> ComputeChildJointToParentJointTransform(const VectorXr& dofs) const override;
    const std::vector<Matrix3r> ComputeChildJointToParentJointRotationDerivative(const VectorXr& dofs) const override;
    const std::vector<std::vector<Matrix3r>> ComputeChildJointToParentJointRotationSecondDerivative(const VectorXr& dofs) const override;
};

TEST_JOINT3D(Ball)

}

#endif