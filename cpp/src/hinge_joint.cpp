#include "include/hinge_joint.hpp"
#include <vector>

namespace backend {

const RigidTransform<2> HingeJoint<2>::ComputeChildJointToParentJointTransform(const VectorXr& dofs) const {
    
    const real theta = dofs(0);
    const real c = std::cos(theta);
    const real s = std::sin(theta);
    Matrix2r R;
    R << c, -s,
        s, c;
    Vector2r t(0, 0);
    return RigidTransform<2>(R, t);
}

const std::vector<Matrix2r> HingeJoint<2>::ComputeChildJointToParentJointRotationDerivative(const VectorXr& dofs) const {
    // TODO.
    // return Joint<2>::ComputeChildJointToParentJointRotationDerivative(dofs);
    ////////////////////////////////////////////
    Matrix2r R;
    const real theta = dofs(0);
    const real c = std::cos(theta);
    const real s = std::sin(theta);
    R << -s, -c,
        c, -s;
    std::vector<Matrix2r> ret{R};
    return ret;
}

const std::vector<std::vector<Matrix2r>> HingeJoint<2>::ComputeChildJointToParentJointRotationSecondDerivative(const VectorXr& dofs) const {
    // TODO.
    // return Joint<2>::ComputeChildJointToParentJointRotationSecondDerivative(dofs);
    ////////////////////////////////////////////
    Matrix2r R;
    const real theta = dofs(0);
    const real c = std::cos(theta);
    const real s = std::sin(theta);
    R << -c, s,
        -s, -c;
    std::vector<std::vector<Matrix2r>> ret{{R}};
    return ret;    
}

const RigidTransform<3> HingeJoint<3>::ComputeChildJointToParentJointTransform(const VectorXr& dofs) const {
    const real theta = dofs(0);

    const Matrix3r R = Eigen::AngleAxis<real>(theta, normalized_axis()).toRotationMatrix();
    const Vector3r t(0, 0, 0);

    return RigidTransform<3>(R, t);
}

const std::vector<Matrix3r> HingeJoint<3>::ComputeChildJointToParentJointRotationDerivative(const VectorXr& dofs) const {
    // TODO.
    // return Joint<3>::ComputeChildJointToParentJointRotationDerivative(dofs);
    ////////////////////////////////////////////
    Matrix3r rot_n;
    rot_n << 0, -normalized_axis()(2), normalized_axis()(1),
            normalized_axis()(2), 0, -normalized_axis()(0),
            -normalized_axis()(1), normalized_axis()(0), 0;
    const real theta = dofs(0);
    const Matrix3r R = std::cos(theta) * rot_n + std::sin(theta) * rot_n * rot_n;
    return std::vector<Matrix3r>(1, R);

}

const std::vector<std::vector<Matrix3r>> HingeJoint<3>::ComputeChildJointToParentJointRotationSecondDerivative(const VectorXr& dofs) const {
    // TODO.
    // return Joint<3>::ComputeChildJointToParentJointRotationSecondDerivative(dofs);
    ////////////////////////////////////////////
    Matrix3r rot_n;
    rot_n << 0, -normalized_axis()(2), normalized_axis()(1),
            normalized_axis()(2), 0, -normalized_axis()(0),
            -normalized_axis()(1), normalized_axis()(0), 0;
    const real theta = dofs(0);
    const Matrix3r R = -std::sin(theta) * rot_n + std::cos(theta) * rot_n * rot_n;
    return std::vector<std::vector<Matrix3r>>(1, std::vector<Matrix3r>(1, R));
}

}