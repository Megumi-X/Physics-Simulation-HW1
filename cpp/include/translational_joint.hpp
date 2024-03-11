#ifndef TRANSLATIONAL_JOINT
#define TRANSLATIONAL_JOINT

#include "include/joint.hpp"

namespace backend {

template<integer dim>
class TranslationalJoint : public Joint<dim> {
public:
    TranslationalJoint() : Joint<dim>(dim) {}

    const RigidTransform<dim> ComputeChildJointToParentJointTransform(const VectorXr& dofs) const override;
    const Eigen::Matrix<real, dim, Eigen::Dynamic> ComputeChildJointToParentJointTranslationDerivative(const VectorXr& dofs) const override;
};

template class TranslationalJoint<2>;
template class TranslationalJoint<3>;

TEST_JOINT(Translational)

}

#endif