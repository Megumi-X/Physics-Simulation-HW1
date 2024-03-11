#include "pybind11/pybind11.h"
#include "pybind11/operators.h"
#include "pybind11/eigen.h"
#include "pybind11/stl.h"
#include "include/simulator.hpp"

PYBIND11_MODULE(backend, m) {
    m.def("TestHingeJoint2d", &backend::TestHingeJoint2d);
    m.def("TestHingeJoint3d", &backend::TestHingeJoint3d);
    m.def("TestUniversalJoint", &backend::TestUniversalJoint);
    m.def("TestBallJoint", &backend::TestBallJoint);
    m.def("TestPrismaticJoint2d", &backend::TestPrismaticJoint2d);
    m.def("TestPrismaticJoint3d", &backend::TestPrismaticJoint3d);
    m.def("TestPlanarJoint", &backend::TestPlanarJoint);
    m.def("TestTranslationalJoint2d", &backend::TestTranslationalJoint2d);
    m.def("TestTranslationalJoint3d", &backend::TestTranslationalJoint3d);

    pybind11::class_<backend::RigidTransform<2>>(m, "RigidTransform2d")
        .def(pybind11::init<>())
        .def(pybind11::init<const backend::Matrix2r&, const backend::Vector2r&>())
        .def("rotation", &backend::RigidTransform<2>::rotation)
        .def("translation", &backend::RigidTransform<2>::translation)
        .def("Inverse", &backend::RigidTransform<2>::Inverse)
        .def(pybind11::self * pybind11::self);

    pybind11::class_<backend::RigidTransform<3>>(m, "RigidTransform3d")
        .def(pybind11::init<>())
        .def(pybind11::init<const backend::Matrix3r&, const backend::Vector3r&>())
        .def("rotation", &backend::RigidTransform<3>::rotation)
        .def("translation", &backend::RigidTransform<3>::translation)
        .def("Inverse", &backend::RigidTransform<3>::Inverse)
        .def(pybind11::self * pybind11::self);

    pybind11::class_<backend::Simulator<2>>(m, "Simulator2d")
        .def(pybind11::init<>())
        .def("Forward", &backend::Simulator<2>::Forward,
            pybind11::arg("time_step"))
        .def("AddLink", &backend::Simulator<2>::AddLink, pybind11::arg("mass"), pybind11::arg("inertia"),
            pybind11::arg("center_of_mass"), pybind11::arg("parent_index"), pybind11::arg("joint_type"),
            pybind11::arg("joint_location"), pybind11::arg("q"), pybind11::arg("joint_axis"))
        .def("link_pose", &backend::Simulator<2>::link_pose,
            pybind11::arg("link_index"))
        .def("set_q", &backend::Simulator<2>::set_q, pybind11::arg("q"))
        .def("set_q_dot", &backend::Simulator<2>::set_q_dot, pybind11::arg("q_dot"))
        .def("ApplyForce", &backend::Simulator<2>::ApplyForce,
           pybind11::arg("link_index"), pybind11::arg("world_force"), pybind11::arg("world_location"))
        .def("ApplyTorque", &backend::Simulator<2>::ApplyTorque,
           pybind11::arg("link_index"), pybind11::arg("world_torque"))
        .def("q", &backend::Simulator<2>::q)
        .def("q_dot", &backend::Simulator<2>::q_dot)
        .def("ResetExternalForceAndTorque", &backend::Simulator<2>::ResetExternalForceAndTorque);

    pybind11::class_<backend::Simulator<3>>(m, "Simulator3d")
        .def(pybind11::init<>())
        .def("Forward", &backend::Simulator<3>::Forward,
            pybind11::arg("time_step"))
        .def("AddLink", &backend::Simulator<3>::AddLink, pybind11::arg("mass"), pybind11::arg("inertia"),
            pybind11::arg("center_of_mass"), pybind11::arg("parent_index"), pybind11::arg("joint_type"),
            pybind11::arg("joint_location"), pybind11::arg("q"), pybind11::arg("joint_axis"))
        .def("link_pose", &backend::Simulator<3>::link_pose,
            pybind11::arg("link_index"))
        .def("set_q", &backend::Simulator<3>::set_q, pybind11::arg("q"))
        .def("set_q_dot", &backend::Simulator<3>::set_q_dot, pybind11::arg("q_dot"))
        .def("ApplyForce", &backend::Simulator<3>::ApplyForce,
           pybind11::arg("link_index"), pybind11::arg("world_force"), pybind11::arg("world_location"))
        .def("ApplyTorque", &backend::Simulator<3>::ApplyTorque,
           pybind11::arg("link_index"), pybind11::arg("world_torque"))
        .def("q", &backend::Simulator<3>::q)
        .def("q_dot", &backend::Simulator<3>::q_dot)
        .def("ResetExternalForceAndTorque", &backend::Simulator<3>::ResetExternalForceAndTorque);
}