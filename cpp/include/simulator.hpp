#ifndef SIMULATOR
#define SIMULATOR

#include "include/common.hpp"
#include "include/joint.hpp"
#include "include/hinge_joint.hpp"
#include "include/prismatic_joint.hpp"
#include "include/planar_joint.hpp"
#include "include/translational_joint.hpp"
#include "include/universal_joint.hpp"
#include "include/ball_joint.hpp"

namespace backend {

template<integer dim>
struct Link {
    const real mass;
    const Eigen::Matrix<real, dim * (dim - 1) / 2, dim * (dim - 1) / 2> inertia;
    const Eigen::Matrix<real, dim, 1> initial_center_of_mass;
    integer parent_index;
    integer dof_begin_index;
    std::shared_ptr<Joint<dim>> joint;
};

template<integer dim>
class Simulator {
public:
    Simulator();
    ~Simulator() {}

    // - Be sure that the "inertia" is w.r.t. the center-of-mass frame.
    // - The "inertia" is 1x1 in 2D and 3x3 in 3D.
    // - The "child joint" frame is set to be at the "joint_location".
    // - The initial "q" corresponds to the initial generalized position,
    //   which would determine how the "parent joint" frame is initialized,
    //   so changing it would NOT affect how the links are placed initially.
    // - We set "parent_index" to be -1 if the parent is the "world".
    // - Some types of joints require non-empty "joint_axis".
    void AddLink(const real mass, const Eigen::Matrix<real, dim * (dim - 1) / 2, dim * (dim - 1) / 2>& inertia,
        const Eigen::Matrix<real, dim, 1>& center_of_mass, const integer parent_index, const std::string& joint_type, 
        const Eigen::Matrix<real, dim, 1>& joint_location, const VectorXr& q, const std::vector<Eigen::Matrix<real, dim, 1>>& joint_axis);

    // Query links and joints.
    const integer dof_num() const { return dof_num_; }
    const VectorXr SliceDofs(const integer joint_index, const VectorXr& dofs) const;

    // Query q and q_dot.
    const VectorXr& generalized_position() const { return generalized_position_; }
    const VectorXr& generalized_velocity() const { return generalized_velocity_; }
    void set_generalized_position(const VectorXr& generalized_position);
    void set_generalized_velocity(const VectorXr& generalized_velocity);
    // Alias.
    const VectorXr& q() const { return generalized_position_; }
    const VectorXr& q_dot() const { return generalized_velocity_; }
    void set_q(const VectorXr& q) { set_generalized_position(q); }
    void set_q_dot(const VectorXr& q_dot) { set_generalized_velocity(q_dot); }

    // Apply force and torque.
    void ApplyForce(const integer link_index, const Eigen::Matrix<real, dim, 1>& world_force,
        const Eigen::Matrix<real, dim, 1>& world_location);
    void ApplyTorque(const integer link_index, const VectorXr& world_torque);
    void ResetExternalForceAndTorque();

    // This API may be confusing: it refers to the pose relative to the initial canonical frame. 
    // To be specific: begin with the object when you are adding it to the simulator, first translate your object so that the center of mass is at the origin, 
    // then apply the transform acquired by `link_pose` should give you the correct configuration of the object. 
    const RigidTransform<dim>& link_pose(const integer link_index) const;
    
    // Cached intermediate results.
    const MatrixXr& jacobian() const;
    const MatrixXr& jacobian_time_derivative() const;
    const MatrixXr& mass_matrix() const;

    // Forward and backward simulation.
    void Forward(const real time_step);

    static const integer w_dim = dim * (dim - 1) / 2;

private:
    const std::shared_ptr<Joint<dim>> CreateJoint(const std::string& joint_type) const;

    // For private usage only.
    const MatrixXr ComputeAngularJacobian() const;                                                          // Jw.
    const MatrixXr ComputeLinearJacobian(const MatrixXr& Jw) const;                                         // Jv.
    const MatrixXr ComputeAngularJacobianTimeDerivative(const MatrixXr& Jw) const;                          // Jw_dot.
    const MatrixXr ComputeLinearJacobianTimeDerivative(const MatrixXr& Jw, const MatrixXr& Jw_dot) const;   // Jv_dot.                                                       // Q.
    const MatrixXr ComputeLhs() const;
    const VectorXr ComputeRhs() const;

    integer link_num_;
    std::vector<Link<dim>> links_;

    integer dof_num_;
    VectorXr generalized_position_;             // q.
    VectorXr generalized_velocity_;             // q_dot.
    VectorXr external_force_and_torque_;        // f.

    // Cached intermediate results.
    // Constant in 2D.
    // Depend on q in 3D.
    mutable MatrixXr mass_matrix_;              // M.
    mutable bool is_mass_matrix_initialized_;
    mutable bool is_mass_matrix_outdated_;

    // Depend on q.
    mutable MatrixXr jacobian_;                 // J.
    mutable bool is_jacobian_outdated_;

    // Depend on q and q_dot.
    mutable MatrixXr jacobian_time_derivative_; // J_dot.
    mutable bool is_jacobian_time_derivative_outdated_;

    // Depend on q.
    mutable std::vector<RigidTransform<dim>> link_pose_;
    mutable bool is_link_pose_outdated_;
};

}

#endif