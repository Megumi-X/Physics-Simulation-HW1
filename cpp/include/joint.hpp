#ifndef JOINT
#define JOINT

#include "include/common.hpp"
#include "include/rigid_transform.hpp"

namespace backend {

template<integer dim>
class Joint {
public:
    Joint(const integer dof_num) : dof_num_(dof_num) {}

    virtual const RigidTransform<dim> ComputeChildJointToParentJointTransform(const VectorXr& dofs) const = 0;
    virtual const std::vector<Eigen::Matrix<real, dim, dim>> ComputeChildJointToParentJointRotationDerivative(const VectorXr& dofs) const;
    virtual const std::vector<std::vector<Eigen::Matrix<real, dim, dim>>> ComputeChildJointToParentJointRotationSecondDerivative(const VectorXr& dofs) const;
    virtual const Eigen::Matrix<real, dim, Eigen::Dynamic> ComputeChildJointToParentJointTranslationDerivative(const VectorXr& dofs) const;
    virtual const std::vector<Eigen::Matrix<real, dim, Eigen::Dynamic>> ComputeChildJointToParentJointTranslationSecondDerivative(const VectorXr& dofs) const;

    // Important: see how the *JointToJoint* APIs transfer to *LinkToLink* API.
    const RigidTransform<dim> ComputeChildLinkToParentLinkTransform(const VectorXr& dofs) const;
    const MatrixXr ComputeChildLinkToParentLinkTransformJacobian(const VectorXr& dofs) const;
    const MatrixXr ComputeChildLinkToParentLinkTransformJacobianTimeDerivative(const VectorXr& dofs, const VectorXr& dofs_dot) const;

    const integer dof_num() const { return dof_num_; }
    const RigidTransform<dim>& parent_joint_to_parent_link_transform() const { return parent_joint_to_parent_link_transform_; }
    const RigidTransform<dim>& child_link_to_child_joint_transform() const { return child_link_to_child_joint_transform_; }

    void set_parent_joint_to_parent_link_transform(const RigidTransform<dim>& parent_joint_to_parent_link_transform)
        { parent_joint_to_parent_link_transform_ = parent_joint_to_parent_link_transform; }
    void set_child_link_to_child_joint_transform(const RigidTransform<dim>& child_link_to_child_joint_transform)
        { child_link_to_child_joint_transform_ = child_link_to_child_joint_transform; }
    void set_joint_axis(const std::vector<Eigen::Matrix<real, dim, 1>>& joint_axis) {
        for (auto& v: joint_axis) Assert(0.99 < v.norm() && v.norm() < 1.01, "Joint::set_joint_axis", "Requires normalized vectors.");
        joint_axis_ = joint_axis; }

protected:
    std::vector<Eigen::Matrix<real, dim, 1>> joint_axis_;

private:
    const integer dof_num_;

    // These are FIXED transforms after initialization.
    RigidTransform<dim> parent_joint_to_parent_link_transform_;
    RigidTransform<dim> child_link_to_child_joint_transform_;
};

// The following functions are for testing.
inline bool CheckClose(const VectorXr& a, const VectorXr& b, const real abs_tol, const real rel_tol) {
    return a.size() == b.size() && ((a - b).cwiseAbs().array() <= b.cwiseAbs().array() * rel_tol + abs_tol).all();
}

template<typename T, typename M>
void TestJoint(const std::string& name) {
    T joint;
    joint.set_joint_axis({ (M::Random() * 99).normalized(), (M::Random() * 99).normalized() });
    const integer dof_num = joint.dof_num();
    const VectorXr q = VectorXr::Random(dof_num);
    const auto dRdq = joint.ComputeChildJointToParentJointRotationDerivative(q);
    const auto dtdq = joint.ComputeChildJointToParentJointTranslationDerivative(q);
    const auto ddRdqq = joint.ComputeChildJointToParentJointRotationSecondDerivative(q);
    const auto ddtdqq = joint.ComputeChildJointToParentJointTranslationSecondDerivative(q);
    const real eps(1e-3), abs_tol(1e-5), rel_tol(1e-3);
    for (integer i = 0; i < dof_num; ++i) {
        VectorXr q_pos = q, q_neg = q; q_pos(i) += eps; q_neg(i) -= eps;
        const auto R_pos = joint.ComputeChildJointToParentJointTransform(q_pos).rotation();
        const VectorXr x_pos = joint.ComputeChildJointToParentJointTransform(q_pos).translation();
        const auto R_neg = joint.ComputeChildJointToParentJointTransform(q_neg).rotation();
        const VectorXr x_neg = joint.ComputeChildJointToParentJointTransform(q_neg).translation();
        const auto dR = (R_pos - R_neg) / (2 * eps);
        const VectorXr dx = (x_pos - x_neg) / (2 * eps);
        Assert(CheckClose(dR.reshaped(), dRdq[i].reshaped(), abs_tol, rel_tol), name, "Incorrect rotation derivative.");
        Assert(CheckClose(dx, dtdq.col(i), abs_tol, rel_tol), name, "Incorrect translation derivative.");
        const auto dRdq_pos = joint.ComputeChildJointToParentJointRotationDerivative(q_pos);
        const auto dRdq_neg = joint.ComputeChildJointToParentJointRotationDerivative(q_neg);
        const auto dxdq_pos = joint.ComputeChildJointToParentJointTranslationDerivative(q_pos);
        const auto dxdq_neg = joint.ComputeChildJointToParentJointTranslationDerivative(q_neg);
        for (integer j = 0; j < dof_num; ++j) {
            const auto ddR = (dRdq_pos[j] - dRdq_neg[j]) / (2 * eps);
            const VectorXr ddx = (dxdq_pos.col(j) - dxdq_neg.col(j)) / (2 * eps);
            Assert(CheckClose(ddR.reshaped(), ddRdqq[i][j].reshaped(), abs_tol, rel_tol), name, "Incorrect rotation second derivative.");
            Assert(CheckClose(ddx, ddtdqq[i].col(j), abs_tol, rel_tol), name, "Incorrect translation second derivative.");
        }
    }
}

#define DECLARE_TEST_JOINT_TEMPLATE(name, dim) inline void Test##name##Joint##dim##d() { TestJoint<name##Joint<dim>, Vector##dim##r>(#name "Joint" #dim "d"); }
#define TEST_JOINT3D(name) inline void Test##name##Joint() { TestJoint<name##Joint, Vector3r>(#name "Joint"); }
#define TEST_JOINT(name) DECLARE_TEST_JOINT_TEMPLATE(name, 2) DECLARE_TEST_JOINT_TEMPLATE(name, 3)

}

#endif