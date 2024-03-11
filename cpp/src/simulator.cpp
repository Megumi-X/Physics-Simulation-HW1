#include "include/simulator.hpp"

namespace backend {

template<integer dim>
Simulator<dim>::Simulator()
    : link_num_(0),
    dof_num_(0),
    generalized_position_(VectorXr::Zero(0)),
    generalized_velocity_(VectorXr::Zero(0)),
    external_force_and_torque_(VectorXr::Zero(0)),
    is_link_pose_outdated_(true),
    mass_matrix_(MatrixXr::Zero(0, 0)),
    is_mass_matrix_initialized_(false),
    is_mass_matrix_outdated_(true),
    jacobian_(MatrixXr::Zero(0, 0)),
    is_jacobian_outdated_(true),
    jacobian_time_derivative_(MatrixXr::Zero(0, 0)),
    is_jacobian_time_derivative_outdated_(true) {}

template<integer dim>
void Simulator<dim>::AddLink(const real mass, const Eigen::Matrix<real, dim * (dim - 1) / 2, dim * (dim - 1) / 2>& inertia,
    const Eigen::Matrix<real, dim, 1>& center_of_mass, const integer parent_index, const std::string& joint_type, 
    const Eigen::Matrix<real, dim, 1>& joint_location, const VectorXr& q, const std::vector<Eigen::Matrix<real, dim, 1>>& joint_axis) {
    // Configure the joint.
    const integer index = link_num_;
    const integer dof_begin_index = dof_num_;
    std::shared_ptr<Joint<dim>> joint = CreateJoint(joint_type);
    Eigen::Matrix<real, dim, 1> parent_center_of_mass = Eigen::Matrix<real, dim, 1>::Zero();
    if (parent_index != -1) {
        Assert(0 <= parent_index && parent_index < link_num_, "Simulator::AddLink", "Out-of-bound parent index.");
        parent_center_of_mass = links_[parent_index].initial_center_of_mass;
    }
    joint->set_joint_axis(joint_axis);
    joint->set_parent_joint_to_parent_link_transform(RigidTransform<dim>(Eigen::Matrix<real, dim, dim>::Identity(),
        joint_location - parent_center_of_mass) * joint->ComputeChildJointToParentJointTransform(q).Inverse());
    joint->set_child_link_to_child_joint_transform(RigidTransform<dim>(Eigen::Matrix<real, dim, dim>::Identity(),
        center_of_mass - joint_location));
    links_.push_back({ mass, inertia, center_of_mass, parent_index, dof_begin_index, joint });
    ++link_num_;
    dof_num_ += joint->dof_num();
    
    // Reset the buffer.
    const VectorXr old_q = generalized_position_;
    generalized_position_ = VectorXr::Zero(dof_num_);
    generalized_position_ << old_q, q;

    const VectorXr old_q_dot = generalized_velocity_;
    generalized_velocity_ = VectorXr::Zero(dof_num_);
    generalized_velocity_.head(dof_begin_index) = old_q_dot;

    external_force_and_torque_ = VectorXr::Zero(dim * link_num_ + w_dim * link_num_);
    jacobian_ = MatrixXr::Zero(dim * link_num_ + w_dim * link_num_, dof_num_);
    jacobian_time_derivative_ = MatrixXr::Zero(dim * link_num_ + w_dim * link_num_, dof_num_);

    // Invalidate flags.
    is_link_pose_outdated_ = true;
    is_mass_matrix_outdated_ = true;
    is_jacobian_outdated_ = true;
    is_jacobian_time_derivative_outdated_ = true;
}

template<integer dim>
const VectorXr Simulator<dim>::SliceDofs(const integer index, const VectorXr& dofs) const {
    const std::string error_location = "Simulator::SliceDofs";
    Assert(0 <= index && index < link_num_, error_location, "Out-of-bound index.");
    Assert(static_cast<integer>(dofs.size()) == dof_num_, error_location, "Invalid dof size.");
    return dofs.segment(links_[index].dof_begin_index, links_[index].joint->dof_num());
}

// Set q and q_dot.
template<integer dim>
void Simulator<dim>::set_generalized_position(const VectorXr& generalized_position) {
    Assert(static_cast<integer>(generalized_position.size()) == dof_num_,
        "Simulator::set_generalized_position", "Incompatible q size.");
    // Skip if there are no updates.
    if ((generalized_position.array() ==  generalized_position_.array()).all()) return;

    generalized_position_ = generalized_position;

    is_link_pose_outdated_ = true;
    is_mass_matrix_outdated_ = true;
    is_jacobian_outdated_ = true;
    is_jacobian_time_derivative_outdated_ = true;
}

template<integer dim>
void Simulator<dim>::set_generalized_velocity(const VectorXr& generalized_velocity) {
    Assert(static_cast<integer>(generalized_velocity.size()) == dof_num_,
        "Simulator::set_generalized_velocity", "Incompatible q_dot size.");
    // Skip if there are no updates.
    if ((generalized_velocity.array() ==  generalized_velocity_.array()).all()) return;

    generalized_velocity_ = generalized_velocity;
    is_jacobian_time_derivative_outdated_ = true;
}

// Apply force and torque.
template<integer dim>
void Simulator<dim>::ApplyForce(const integer link_index, const Eigen::Matrix<real, dim, 1>& world_force,
    const Eigen::Matrix<real, dim, 1>& world_location) {
    Assert(0 <= link_index && link_index < link_num_, "Simulator::ApplyForce", "Out-of-bound index.");

    external_force_and_torque_.segment(dim * link_index, dim) += world_force;

    external_force_and_torque_.segment(dim * link_num_ + w_dim * link_index, w_dim) += Cross<dim>(
        world_location - link_pose(link_index).translation(), world_force);
}

template<integer dim>
void Simulator<dim>::ApplyTorque(const integer link_index, const VectorXr& world_torque) {
    Assert(0 <= link_index && link_index < link_num_, "Simulator::Torque", "Out-of-bound index.");
    Assert(static_cast<integer>(world_torque.size()) == w_dim, "Simulator::ApplyTorque", "Inconsistent torque dim.");
    external_force_and_torque_.segment(dim * link_num_ + w_dim * link_index, w_dim) += world_torque;
}

template<integer dim>
void Simulator<dim>::ResetExternalForceAndTorque() {
    external_force_and_torque_ = VectorXr::Zero(dim * link_num_ + w_dim * link_num_);
}

template<integer dim>
const RigidTransform<dim>& Simulator<dim>::link_pose(const integer index) const {
    if (is_link_pose_outdated_) {
        // Recompute.
        link_pose_.clear();
        for (integer i = 0; i < link_num_; ++i) {
            const integer parent_index = links_[i].parent_index;
            const RigidTransform<dim>& child_link_to_parent_link_transform = links_[i].joint->ComputeChildLinkToParentLinkTransform(SliceDofs(i, q()));
            link_pose_.push_back(
                (parent_index == -1) ? child_link_to_parent_link_transform : link_pose_[parent_index] * child_link_to_parent_link_transform
            );
        }
        is_link_pose_outdated_ = false;
    }
    return link_pose_[index];
}

template<>
const MatrixXr& Simulator<2>::mass_matrix() const {
    const integer dim = 2;
    if (!is_mass_matrix_initialized_) {
        mass_matrix_ = MatrixXr::Identity(dim * link_num_ + w_dim * link_num_, dim * link_num_ + w_dim * link_num_);
        // TODO.

        //////////////////////////////////
        is_mass_matrix_initialized_ = true;
    }
    return mass_matrix_;
}

template<>
const MatrixXr& Simulator<3>::mass_matrix() const {
    const integer dim = 3;
    if (!is_mass_matrix_initialized_) {
        mass_matrix_ = MatrixXr::Zero(dim * link_num_ + w_dim * link_num_, dim * link_num_ + w_dim * link_num_);
        for (integer i = 0; i < link_num_; ++i)
            for (integer d = 0; d < dim; ++d)
                mass_matrix_(dim * i + d, dim * i + d) = links_[i].mass;
        is_mass_matrix_initialized_ = true;
    }
    if (is_mass_matrix_outdated_) {
        for (integer i = 0; i < link_num_; ++i) {
            const Matrix3r R = link_pose(i).rotation();
            mass_matrix_.block(dim * link_num_ + w_dim * i, dim * link_num_ + w_dim * i, w_dim, w_dim) =
                R * links_[i].inertia * R.transpose();
        }
        is_mass_matrix_outdated_ = false;
    }
    return mass_matrix_;
}

template<>
const MatrixXr Simulator<2>::ComputeAngularJacobian() const {
    const integer dim = 2;
    MatrixXr Jw = MatrixXr::Zero(w_dim * link_num_, dof_num_);

    for (integer i = 0; i < link_num_; ++i) {
        integer parent_index = i;
        while (parent_index != -1) {
            const MatrixXr joint_Jw = links_[parent_index].joint->ComputeChildLinkToParentLinkTransformJacobian(SliceDofs(parent_index, q())).bottomRows(w_dim);
            Jw.block(w_dim * i, links_[parent_index].dof_begin_index, w_dim, links_[parent_index].joint->dof_num()) += joint_Jw;
            parent_index = links_[parent_index].parent_index;
        }
    }

    return Jw;
}

template<>
const MatrixXr Simulator<3>::ComputeAngularJacobian() const {
    const integer dim = 3;
    MatrixXr Jw = MatrixXr::Zero(w_dim * link_num_, dof_num_);
    // TODO.

    //////////////////////////////////
    return Jw;
}

template<>
const MatrixXr Simulator<2>::ComputeLinearJacobian(const MatrixXr& Jw) const {
    const integer dim = 2;
    Assert(static_cast<integer>(Jw.rows()) == w_dim * link_num_ && static_cast<integer>(Jw.cols()) == dof_num_,
        "Simulator::ComputeLinearJacobian", "Incompatible Jw size.");

    MatrixXr Jv = MatrixXr::Zero(dim * link_num_, dof_num_);
    // TODO.

    //////////////////////////////////
    return Jv;
}

template<>
const MatrixXr Simulator<3>::ComputeLinearJacobian(const MatrixXr& Jw) const {
    const integer dim = 3;
    Assert(static_cast<integer>(Jw.rows()) == w_dim * link_num_ && static_cast<integer>(Jw.cols()) == dof_num_,
        "Simulator::ComputeLinearJacobian", "Incompatible Jw size.");

    MatrixXr Jv = MatrixXr::Zero(dim * link_num_, dof_num_);
    // TODO.

    //////////////////////////////////
    return Jv;
}

template<>
const MatrixXr Simulator<2>::ComputeAngularJacobianTimeDerivative(const MatrixXr& Jw) const {
    const integer dim = 2;
    MatrixXr Jw_dot = MatrixXr::Zero(w_dim * link_num_, dof_num_);

    for (integer i = 0; i < link_num_; ++i) {
        integer parent_index = i;
        while (parent_index != -1) {
            const MatrixXr joint_Jw_dot = links_[parent_index].joint->ComputeChildLinkToParentLinkTransformJacobianTimeDerivative(SliceDofs(parent_index, q()), SliceDofs(parent_index, q_dot())).bottomRows(w_dim);
            Jw_dot.block(w_dim * i, links_[parent_index].dof_begin_index, w_dim, links_[parent_index].joint->dof_num()) += joint_Jw_dot;
            parent_index = links_[parent_index].parent_index;
        }
    }

    return Jw_dot;
}

template<>
const MatrixXr Simulator<3>::ComputeAngularJacobianTimeDerivative(const MatrixXr& Jw) const {
    const integer dim = 3;
    
    Assert(static_cast<integer>(Jw.rows()) == w_dim * link_num_ && static_cast<integer>(Jw.cols()) == dof_num_,
        "Simulator::ComputeAngularJacobianTimeDerivative", "Incompatible Jw size.");

    MatrixXr Jw_dot = MatrixXr::Zero(w_dim * link_num_, dof_num_);
    // TODO.

    //////////////////////////////////
    return Jw_dot;
}

template<>
const MatrixXr Simulator<2>::ComputeLinearJacobianTimeDerivative(const MatrixXr& Jw, const MatrixXr& Jw_dot) const {
    const integer dim = 2;
    Assert(static_cast<integer>(Jw.rows()) == w_dim * link_num_ && static_cast<integer>(Jw.cols()) == dof_num_,
        "Simulator::ComputeLinearJacobian", "Incompatible Jw size.");
    Assert(static_cast<integer>(Jw_dot.rows()) == w_dim * link_num_ && static_cast<integer>(Jw_dot.cols()) == dof_num_,
        "Simulator::ComputeLinearJacobian", "Incompatible Jw_dot size.");

    MatrixXr Jv_dot = MatrixXr::Zero(dim * link_num_, dof_num_);
    // TODO.

    //////////////////////////////////
    return Jv_dot;
}

template<>
const MatrixXr Simulator<3>::ComputeLinearJacobianTimeDerivative(const MatrixXr& Jw, const MatrixXr& Jw_dot) const {
    const integer dim = 3;
    Assert(static_cast<integer>(Jw.rows()) == w_dim * link_num_ && static_cast<integer>(Jw.cols()) == dof_num_,
        "Simulator::ComputeLinearJacobian", "Incompatible Jw size.");
    Assert(static_cast<integer>(Jw_dot.rows()) == w_dim * link_num_ && static_cast<integer>(Jw_dot.cols()) == dof_num_,
        "Simulator::ComputeLinearJacobian", "Incompatible Jw_dot size.");

    MatrixXr Jv_dot = MatrixXr::Zero(dim * link_num_, dof_num_);
    // TODO.

    //////////////////////////////////
    return Jv_dot;
}

template<integer dim>
const MatrixXr& Simulator<dim>::jacobian() const {
    if (is_jacobian_outdated_) {
        const MatrixXr Jw = ComputeAngularJacobian();
        const MatrixXr Jv = ComputeLinearJacobian(Jw);

        jacobian_.topRows(dim * link_num_) = Jv;
        jacobian_.bottomRows(w_dim * link_num_) = Jw;
        is_jacobian_outdated_ = false;
    }
    return jacobian_;
}

template<integer dim>
const MatrixXr& Simulator<dim>::jacobian_time_derivative() const {
    if (is_jacobian_time_derivative_outdated_) {
        const MatrixXr& J = jacobian();
        const MatrixXr Jw_dot = ComputeAngularJacobianTimeDerivative(J.bottomRows(w_dim * link_num_));
        const MatrixXr Jv_dot = ComputeLinearJacobianTimeDerivative(J.bottomRows(w_dim * link_num_), Jw_dot);

        jacobian_time_derivative_.topRows(dim * link_num_) = Jv_dot;
        jacobian_time_derivative_.bottomRows(w_dim * link_num_) = Jw_dot;
        is_jacobian_time_derivative_outdated_ = false;
    }
    return jacobian_time_derivative_;
}

template<integer dim>
const MatrixXr Simulator<dim>::ComputeLhs() const {
    const MatrixXr& J = jacobian();
    return J.transpose() * mass_matrix() * J;
}

template<>
const VectorXr Simulator<2>::ComputeRhs() const {
    const MatrixXr& J = jacobian();
    return J.transpose() * external_force_and_torque_  - J.transpose() * (mass_matrix() * (jacobian_time_derivative() * q_dot()));
}

template<>
const VectorXr Simulator<3>::ComputeRhs() const {
    const integer dim = 3;
    const MatrixXr& M = mass_matrix();
    const MatrixXr& J = jacobian();
    const MatrixXr& J_dot = jacobian_time_derivative();

    // Compute [w_tilde].
    MatrixXr w_tilde = MatrixXr::Zero(dim * link_num_ + w_dim * link_num_, dim * link_num_ + w_dim * link_num_);
    const MatrixXr w = (J * q_dot()).tail(w_dim * link_num_).reshaped(w_dim, link_num_);
    for (integer i = 0; i < link_num_; ++i)
        w_tilde.block(dim * link_num_ + w_dim * i, dim * link_num_ + w_dim * i, w_dim, w_dim) = ToSkewSymmetricMatrix<dim>(w.col(i));

    return J.transpose() * external_force_and_torque_ - J.transpose() * (M * (J_dot * q_dot())) - J.transpose() * (w_tilde * (M * (J * q_dot())));
}

template<integer dim>
void Simulator<dim>::Forward(const real time_step) {
    set_q_dot(generalized_velocity_ + ComputeLhs().fullPivHouseholderQr().solve(ComputeRhs()) * time_step);
    set_q(generalized_position_ + generalized_velocity_ * time_step);
}

template<>
const std::shared_ptr<Joint<2>> Simulator<2>::CreateJoint(const std::string& joint_type) const {
    std::shared_ptr<Joint<2>> joint = nullptr;
    if (joint_type == "hinge") {
        joint = std::make_shared<HingeJoint<2>>();
    } else if (joint_type == "prismatic") {
        joint = std::make_shared<PrismaticJoint<2>>();
    } else if (joint_type == "translational") {
        joint = std::make_shared<TranslationalJoint<2>>();
    } else {
        Assert(false, "Simulator::CreateJoint", "Unsupported joint type " + joint_type + ".");
    }
    return joint;
}

template<>
const std::shared_ptr<Joint<3>> Simulator<3>::CreateJoint(const std::string& joint_type) const {
    std::shared_ptr<Joint<3>> joint = nullptr;
    if (joint_type == "hinge") {
        joint = std::make_shared<HingeJoint<3>>();
    } else if (joint_type == "universal") {
        joint = std::make_shared<UniversalJoint>();
    } else if (joint_type == "ball") {
        joint = std::make_shared<BallJoint>();
    } else if (joint_type == "prismatic") {
        joint = std::make_shared<PrismaticJoint<3>>();
    } else if (joint_type == "planar") {
        joint = std::make_shared<PlanarJoint>();
    } else if (joint_type == "translational") {
        joint = std::make_shared<TranslationalJoint<3>>();
    } else {
        Assert(false, "Simulator::CreateJoint", "Unsupported joint type " + joint_type + ".");
    }
    return joint;
}

template class Simulator<2>;
template class Simulator<3>;

}