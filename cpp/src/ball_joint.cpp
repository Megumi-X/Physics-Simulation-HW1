#include "include/ball_joint.hpp"

namespace backend {

const RigidTransform<3> BallJoint::ComputeChildJointToParentJointTransform(const VectorXr& dofs) const {
    
    const real theta = dofs.norm();
    const Vector3r normalized_axis = dofs.normalized();
    const Matrix3r R = Eigen::AngleAxis<real>(theta, normalized_axis).toRotationMatrix();
    const Vector3r t = Vector3r::Zero();

    return RigidTransform<3>(R, t);
}

const std::vector<Matrix3r> BallJoint::ComputeChildJointToParentJointRotationDerivative(const VectorXr& dofs) const {

    const real theta = dofs.norm();
    const real theta_sqr = theta * theta;
    real a1, a2, b1, b2;
    if (theta > static_cast<real>(2e-3)) {
        const real a0 = std::cos(theta);
        a1 = std::sin(theta) / theta;
        a2 = (1 - a0) / theta_sqr;
        b1 = (a0 - a1) / theta_sqr;
        b2 = (a1 - 2 * a2) / theta_sqr;
    } else {
        a1 = 1 - theta_sqr / 6;
        a2 = static_cast<real>(0.5) - theta_sqr / 24;
        b1 = static_cast<real>(-1. / 3.) + theta_sqr / 30;
        b2 = static_cast<real>(-1. / 12.) + theta_sqr / 180;
    }
    const Matrix3r A = ToSkewSymmetricMatrix<3>(dofs);

    std::vector<Matrix3r> ret(3, Matrix3r::Zero());
    for (integer i = 0; i < 3; ++i) {
        const Matrix3r dA = ToSkewSymmetricMatrix<3>(Vector3r::Unit(i));
        ret[i] = b1 * dofs(i) * A + a1 * dA + b2 * dofs(i) * A * A + a2 * (A * dA + dA * A); 
    }
    return ret;
}

const std::vector<std::vector<Matrix3r>> BallJoint::ComputeChildJointToParentJointRotationSecondDerivative(const VectorXr& dofs) const {

    const real theta = dofs.norm();
    const real theta_sqr = theta * theta;

    const Matrix3r A = ToSkewSymmetricMatrix<3>(dofs);
    const Matrix3r A_sqr = A * A;
    std::array<Matrix3r, 3> dA{ ToSkewSymmetricMatrix<3>(Vector3r::Unit(0)), ToSkewSymmetricMatrix<3>(Vector3r::Unit(1)), ToSkewSymmetricMatrix<3>(Vector3r::Unit(2)) };

    real a1, a2, b1, b2, c1, c2;
    if (theta > static_cast<real>(0.02)) {
        const real a0 = std::cos(theta);
        a1 = std::sin(theta) / theta;
        a2 = (1 - a0) / theta_sqr;
        const real b0 = -a1;
        b1 = (a0 - a1) / theta_sqr;
        b2 = (a1 - 2 * a2) / theta_sqr;
        c1 = (b0 - 3 * b1) / theta_sqr;
        c2 = (b1 - 4 * b2) / theta_sqr;
    } else {
        a1 = 1 - theta_sqr / 6;
        a2 = static_cast<real>(0.5) - theta_sqr / 24;
        b1 = static_cast<real>(-1. / 3.) + theta_sqr / 30;
        b2 = static_cast<real>(-1. / 12.) + theta_sqr / 180;
        c1 = static_cast<real>(1. / 15.) - theta_sqr / 210;
        c2 = static_cast<real>(1. / 90.) - theta_sqr / 1680;
    }

    std::vector<std::vector<Matrix3r>> ret(3, std::vector<Matrix3r>(3, Matrix3r::Zero()));
    for (integer i = 0; i < 3; ++i) {
        const real qi = dofs(i);
        for (integer j = 0; j < 3; ++j) {
            if (i == j) {
                ret[i][i] = (b1 + c1 * qi * qi) * A + 2 * b1 * qi * dA[i]
                          + (b2 + c2 * qi * qi) * A_sqr
                          + 2 * b2 * qi * A * dA[i] + 2 * b2 * qi * dA[i] * A
                          + 2 * a2 * dA[i] * dA[i];
            } else {
                const real qj = dofs(j);
                ret[i][j] = c1 * qi * qj * A + b1 * (qi * dA[j] + qj * dA[i])
                          + c2 * qi * qj * A_sqr
                          + b2 * qi * (A * dA[j] + dA[j] * A)
                          + b2 * qj * (A * dA[i] + dA[i] * A)
                          + a2 * (dA[j] * dA[i] + dA[i] * dA[j]);
            }
        }
    }
    return ret;
}

}