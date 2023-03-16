//
// Created by Dongho Kang on 06.09.22.
//

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "pyloco/PySimulator.h"
#include "pylocobase/Viewer.h"
#include "pylocobase/sim/VanillaSimulator.h"

namespace py = pybind11;

namespace pyloco {

PYBIND11_MODULE(pyloco, m) {  //Defining module
    m.doc() = "Python binding of locomotion simulator.";

    py::class_<Simulator, PySimulator>(m, "Simulator")
        .def(py::init<double, double>())
        .def_readonly("max_joint_angle", &Simulator::maxJointAngle_)
        .def_readonly("min_joint_angle", &Simulator::minJointAngle_)
        .def_readonly("nominal_joint_angle", &Simulator::nominalJointAngle_)
        .def_readonly("zero_joint_angle", &Simulator::zeroJointAngle_)
        .def_readonly("nominal_base_height", &Simulator::nominalBaseHeight_)
        .def_readonly("control_timestep_size", &Simulator::controlTimeStepSize)
        .def_readonly("sim_timestep_size", &Simulator::simTimeStepSize)
        .def_readwrite("motor_kp", &Simulator::motorsKp)
        .def_readwrite("motor_kd", &Simulator::motorsKd)
        .def_readwrite("motor_max_torque", &Simulator::motorMaxTorque)
        .def("get_observation", &Simulator::getObservation, "Get observation implemented in C++ side.")
        .def("reset", py::overload_cast<>(&Simulator::reset), "Reset environment with default state.")
        .def("reset", py::overload_cast<const crl::dVector &, const crl::dVector &>(&Simulator::reset), "Reset environment with given q and qDot.")
        .def("step", &Simulator::step)
        .def("set_q_and_qdot", &Simulator::setQAndQDot)
        .def("get_time_stamp", &Simulator::getTimeStamp)
        .def("get_q", &Simulator::getQ)
        .def("get_qdot", &Simulator::getQDot)
        .def("is_robot_collapsed", &Simulator::isRobotCollapsed)
        .def("get_feet_pos", &Simulator::getFeetPosition)
        .def("get_feet_vel", &Simulator::getFeetVelocity)
        .def("is_feet_contact", &Simulator::getFeetContact)
        .def("get_motor_torques", &Simulator::getMotorTorques)
        .def("get_all_motor_torques", &Simulator::getAllLoopMotorTorques)
        .def("throw_box", &Simulator::throwBox)
        .def_readwrite("lock_selected_joints", &Simulator::lockSelectedJoints);

    py::class_<VanillaSimulator, Simulator>(m, "VanillaSimulator")
        .def(py::init<double, double>())
        .def(py::init<double, double, uint, bool>())
        .def("__repr__", [](const VanillaSimulator &) { return "<pyloco VanillaSimulator>"; })
        .def_readwrite("cmd_forward_speed", &VanillaSimulator::commandForwardSpeed);

    py::class_<Viewer>(m, "Viewer")
        .def(py::init<Simulator *>())
        .def(py::init<Simulator *, int, int>())
        .def_readonly("width", &Viewer::width)
        .def_readonly("height", &Viewer::height)
        .def_readwrite("show_plots", &Viewer::showPlots)
        .def("render", &Viewer::render)
        .def("read_pixels",
             [](Viewer &self, py::array_t<unsigned char> rgb) {
                 py::buffer_info info = rgb.request();
                 self.readPixels(static_cast<unsigned char *>(info.ptr));
             })
        .def("close", &Viewer::close)
        .def("__repr__", [](const Viewer &) { return "<pyloco Viewer>"; });
}

}  // namespace pyloco
