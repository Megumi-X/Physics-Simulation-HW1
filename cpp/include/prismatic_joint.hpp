#ifndef PRISMATIC_JOINT
#define PRISMATIC_JOINT

#include "include/joint.hpp"

namespace backend {

template<integer dim>
class PrismaticJoint : public Joint<dim> {
public:
    PrismaticJoint() : Joint<dim>(1) {}

    const RigidTransform<dim> ComputeChildJointToParentJointTransform(const VectorXr& dofs) const override;
    const Eigen::Matrix<real, dim, Eigen::Dynamic> ComputeChildJointToParentJointTranslationDerivative(const VectorXr& dofs) const override;
};

TEST_JOINT(Prismatic)

}

#endif