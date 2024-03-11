#include "include/rigid_transform.hpp"

namespace backend {

template<integer dim>
RigidTransform<dim>::RigidTransform()
    : rotation_(Eigen::Matrix<real, dim, dim>::Identity()), translation_(Eigen::Matrix<real, dim, 1>::Zero()) {}

template<integer dim>
RigidTransform<dim>::RigidTransform(const Eigen::Matrix<real, dim, dim>& rotation, const Eigen::Matrix<real, dim, 1>& translation) {
    // Check if rotation is an orthogonal matrix.
    Assert(std::abs(rotation.determinant() - 1 < 1e-3) &&
        std::abs((rotation * rotation.transpose() - Eigen::Matrix<real, dim, dim>::Identity()).squaredNorm() / dim) < 1e-6,
        "Transform::Transform", "The rotation matrix is not orthogonal.");
    rotation_ = rotation;
    translation_ = translation;
}

template<integer dim>
const RigidTransform<dim> RigidTransform<dim>::Inverse() const {
    Eigen::Matrix<real, dim, dim> ret_rotation = rotation_.transpose();
    Eigen::Matrix<real, dim, 1> ret_translation = -ret_rotation * translation_;
    return RigidTransform<dim>(ret_rotation, ret_translation);
}

template class RigidTransform<2>;
template class RigidTransform<3>;

template<integer dim>
const RigidTransform<dim> operator*(const RigidTransform<dim>& t1, const RigidTransform<dim>& t2) {
    Eigen::Matrix<real, dim, dim> ret_rotation = t1.rotation() * t2.rotation();
    Eigen::Matrix<real, dim, 1> ret_translation = t1.rotation() * t2.translation() + t1.translation();
    return RigidTransform<dim>(ret_rotation, ret_translation);
}

template const RigidTransform<2> operator*(const RigidTransform<2>& t1, const RigidTransform<2>& t2);
template const RigidTransform<3> operator*(const RigidTransform<3>& t1, const RigidTransform<3>& t2);

template<integer dim>
std::ostream& operator<<(std::ostream& out, const RigidTransform<dim>& t) {
    out << "Rotation:\n" << t.rotation() << "\nTranslation:\n" << t.translation().transpose() << std::endl;
    return out;
}

template std::ostream& operator<<<2>(std::ostream& out, const RigidTransform<2>& t);
template std::ostream& operator<<<3>(std::ostream& out, const RigidTransform<3>& t);

}