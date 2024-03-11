#include "include/joint.hpp"

namespace backend {

template<integer dim>
const std::vector<Eigen::Matrix<real, dim, dim>> Joint<dim>::ComputeChildJointToParentJointRotationDerivative(const VectorXr& dofs) const {
    return std::vector<Eigen::Matrix<real, dim, dim>>(dof_num_, Eigen::Matrix<real, dim, dim>::Zero());
}

template<integer dim>
const std::vector<std::vector<Eigen::Matrix<real, dim, dim>>> Joint<dim>::ComputeChildJointToParentJointRotationSecondDerivative(const VectorXr& dofs) const {
    return std::vector<std::vector<Eigen::Matrix<real, dim, dim>>>(dof_num_, std::vector<Eigen::Matrix<real, dim, dim>>(dof_num_, Eigen::Matrix<real, dim, dim>::Zero()));
}

template<integer dim>
const Eigen::Matrix<real, dim, Eigen::Dynamic> Joint<dim>::ComputeChildJointToParentJointTranslationDerivative(const VectorXr& dofs) const {
    return Eigen::Matrix<real, dim, Eigen::Dynamic>::Zero(dim, dof_num_);
}

template<integer dim>
const std::vector<Eigen::Matrix<real, dim, Eigen::Dynamic>> Joint<dim>::ComputeChildJointToParentJointTranslationSecondDerivative(const VectorXr& dofs) const {
    return std::vector<Eigen::Matrix<real, dim, Eigen::Dynamic>>(dof_num_, Eigen::Matrix<real, dim, Eigen::Dynamic>::Zero(dim, dof_num_));
}

template<integer dim>
const RigidTransform<dim> Joint<dim>::ComputeChildLinkToParentLinkTransform(const VectorXr& dofs) const {
    return parent_joint_to_parent_link_transform_ * ComputeChildJointToParentJointTransform(dofs) * child_link_to_child_joint_transform_;
}

template<integer dim>
const MatrixXr Joint<dim>::ComputeChildLinkToParentLinkTransformJacobian(const VectorXr& dofs) const {
    const Eigen::Matrix<real, dim, dim>& Rp = parent_joint_to_parent_link_transform().rotation();
    const Eigen::Matrix<real, dim, dim> R = ComputeChildJointToParentJointTransform(dofs).rotation();
    const Eigen::Matrix<real, dim, 1>& tc = child_link_to_child_joint_transform().translation();

    const std::vector<Eigen::Matrix<real, dim, dim>> dR = ComputeChildJointToParentJointRotationDerivative(dofs);
    const Eigen::Matrix<real, dim, Eigen::Dynamic> dt = ComputeChildJointToParentJointTranslationDerivative(dofs);

    // Obtain the angular dim.
    const integer w_dim = (dim == 2) ? 1 : 3;
    MatrixXr J = MatrixXr::Zero(dim + w_dim, dof_num_);
    for (integer i = 0; i < dof_num_; ++i) {
        J.col(i).head(dim) = Rp * (dR[i] * tc + dt.col(i));
        const Eigen::Matrix<real, dim, dim> w_mat = Rp * dR[i] * R.transpose() * Rp.transpose();
        J.col(i).tail(w_dim) = FromSkewSymmetricMatrix<dim>(w_mat);
    }

    return J;
}

template<integer dim>
const MatrixXr Joint<dim>::ComputeChildLinkToParentLinkTransformJacobianTimeDerivative(const VectorXr& dofs, const VectorXr& dofs_dot) const {
    const Eigen::Matrix<real, dim, dim>& Rp = parent_joint_to_parent_link_transform().rotation();
    const Eigen::Matrix<real, dim, dim> R = ComputeChildJointToParentJointTransform(dofs).rotation();
    const Eigen::Matrix<real, dim, 1>& tc = child_link_to_child_joint_transform().translation();

    const std::vector<Eigen::Matrix<real, dim, dim>> dR = ComputeChildJointToParentJointRotationDerivative(dofs);
    const Eigen::Matrix<real, dim, Eigen::Dynamic> dt = ComputeChildJointToParentJointTranslationDerivative(dofs);
    const std::vector<std::vector<Eigen::Matrix<real, dim, dim>>> ddR = ComputeChildJointToParentJointRotationSecondDerivative(dofs);
    const std::vector<Eigen::Matrix<real, dim, Eigen::Dynamic>> ddt = ComputeChildJointToParentJointTranslationSecondDerivative(dofs);

    const integer w_dim = (dim == 2) ? 1 : 3;

    std::vector<MatrixXr> dJ(dof_num_, MatrixXr::Zero(dim + w_dim, dof_num_));
    for (integer j = 0; j < dof_num_; ++j)
        for (integer i = 0; i < dof_num_; ++i) {
            const Eigen::Matrix<real, dim, 1> dJvi_dqj = Rp * (ddR[j][i] * tc + ddt[j].col(i));
            dJ[j].col(i).head(dim) += dJvi_dqj;
            dJ[j].col(i).tail(w_dim) += FromSkewSymmetricMatrix<dim>(
                Rp * (ddR[j][i] * R.transpose() + dR[i] * dR[j].transpose()) * Rp.transpose()
            );
        }

    MatrixXr J_dot = MatrixXr::Zero(dim + w_dim, dof_num_);
    for (integer i = 0; i < dof_num_; ++i)
        J_dot += dJ[i] * dofs_dot(i);

    return J_dot;
}

template class Joint<2>;
template class Joint<3>;

}