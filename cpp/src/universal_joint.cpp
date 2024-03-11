#include "include/universal_joint.hpp"
#include <vector>

namespace backend {

const RigidTransform<3> UniversalJoint::ComputeChildJointToParentJointTransform(const VectorXr& dofs) const {
    

    const Eigen::AngleAxis<real> R_child(dofs(0), normalized_axis_child());
    const Eigen::AngleAxis<real> R_parent(dofs(1), normalized_axis_parent());
    const Matrix3r R = (R_parent * R_child).matrix();
    return RigidTransform<3>(R, Vector3r::Zero());
}

const std::vector<Matrix3r> UniversalJoint::ComputeChildJointToParentJointRotationDerivative(const VectorXr& dofs) const {
    // TODO.
    // return Joint<3>::ComputeChildJointToParentJointRotationDerivative(dofs);
    ////////////////////////////////////////////
    Matrix3r rot_n_child, rot_n_parent;
    rot_n_child << 0, -normalized_axis_child()(2), normalized_axis_child()(1),
            normalized_axis_child()(2), 0, -normalized_axis_child()(0),
            -normalized_axis_child()(1), normalized_axis_child()(0), 0;
    rot_n_parent << 0, -normalized_axis_parent()(2), normalized_axis_parent()(1),
            normalized_axis_parent()(2), 0, -normalized_axis_parent()(0),
            -normalized_axis_parent()(1), normalized_axis_parent()(0), 0;
    const real theta_child = dofs(0);
    const Matrix3r dR_child = std::cos(theta_child) * rot_n_child + std::sin(theta_child) * rot_n_child * rot_n_child;
    const real theta_parent = dofs(1);
    const Matrix3r dR_parent = std::cos(theta_parent) * rot_n_parent + std::sin(theta_parent) * rot_n_parent * rot_n_parent;
    const Eigen::AngleAxis<real> R_child(dofs(0), normalized_axis_child());
    const Eigen::AngleAxis<real> R_parent(dofs(1), normalized_axis_parent());
    std::vector<Matrix3r> ret{R_parent.matrix() * dR_child, dR_parent * R_child.matrix()};
    return ret;

}

const std::vector<std::vector<Matrix3r>> UniversalJoint::ComputeChildJointToParentJointRotationSecondDerivative(const VectorXr& dofs) const {
    // TODO.
    // return Joint<3>::ComputeChildJointToParentJointRotationSecondDerivative(dofs);
    ////////////////////////////////////////////
    Matrix3r rot_n_child, rot_n_parent;
    rot_n_child << 0, -normalized_axis_child()(2), normalized_axis_child()(1),
            normalized_axis_child()(2), 0, -normalized_axis_child()(0),
            -normalized_axis_child()(1), normalized_axis_child()(0), 0;
    rot_n_parent << 0, -normalized_axis_parent()(2), normalized_axis_parent()(1),
            normalized_axis_parent()(2), 0, -normalized_axis_parent()(0),
            -normalized_axis_parent()(1), normalized_axis_parent()(0), 0;
    const real theta_child = dofs(0);
    const Matrix3r dRdqq_child = -std::sin(theta_child) * rot_n_child + std::cos(theta_child) * rot_n_child * rot_n_child;
    const Matrix3r dR_child = std::cos(theta_child) * rot_n_child + std::sin(theta_child) * rot_n_child * rot_n_child;
    const real theta_parent = dofs(1);
    const Matrix3r dRdqq_parent = -std::sin(theta_parent) * rot_n_parent + std::cos(theta_parent) * rot_n_parent * rot_n_parent;
    const Matrix3r dR_parent = std::cos(theta_parent) * rot_n_parent + std::sin(theta_parent) * rot_n_parent * rot_n_parent;
    const Eigen::AngleAxis<real> R_child(dofs(0), normalized_axis_child());
    const Eigen::AngleAxis<real> R_parent(dofs(1), normalized_axis_parent());
    std::vector<std::vector<Matrix3r>> ret{{R_parent.matrix() * dRdqq_child, dR_parent * dR_child}, {dR_parent * dR_child, dRdqq_parent * R_child.matrix()}};
    return ret;
}

}