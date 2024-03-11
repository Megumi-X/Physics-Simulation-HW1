#ifndef PLANAR_JOINT
#define PLANAR_JOINT

#include "include/joint.hpp"

namespace backend {

class PlanarJoint : public Joint<3> {
public:
    PlanarJoint() : Joint<3>(2) {}

    const RigidTransform<3> ComputeChildJointToParentJointTransform(const VectorXr& dofs) const override;
    const Matrix3Xr ComputeChildJointToParentJointTranslationDerivative(const VectorXr& dofs) const override;
};

TEST_JOINT3D(Planar)

}

#endif