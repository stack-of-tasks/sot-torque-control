/*
 * Copyright 2017, A. Del Prete, T. Flayols, O. Stasse, LAAS-CNRS
 *
 * This file is part of sot-torque-control.
 * sot-torque-control is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 * sot-torque-control is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.  You should
 * have received a copy of the GNU Lesser General Public License along
 * with sot-torque-control.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <sot/torque_control/common.hh>
#include <boost/python.hpp>

using namespace boost::python;
using namespace dynamicgraph::sot::torque_control;


BOOST_PYTHON_MODULE(common_sot_py)
{
  class_<JointLimits>
    ("JointLimits",init<double,double>())
    .def_readwrite("upper",&JointLimits::upper)
    .def_readwrite("lower",&JointLimits::lower)
    ;

  class_<ForceLimits>("ForceLimits",init<const Eigen::VectorXd &,const Eigen::VectorXd &>())
    .def("display",&ForceLimits::display)
    .def_readwrite("upper",&ForceLimits::upper)
    .def_readwrite("lower",&ForceLimits::lower)
    ;

  class_<ForceUtil>("ForceUtil")
    .def("set_name_to_force_id", &ForceUtil::set_name_to_force_id)
    .def("set_force_id_to_limits", &ForceUtil::set_force_id_to_limits)
    .def("create_force_id_to_name_map",&ForceUtil::create_force_id_to_name_map)
    .def("get_id_from_name",        &ForceUtil::get_id_from_name)
    .def("get_name_from_id",        &ForceUtil::cp_get_name_from_id)
    .def("get_limits_from_id",      &ForceUtil::cp_get_limits_from_id)
    .def("get_force_id_left_hand",  &ForceUtil::get_force_id_left_hand)
    .def("set_force_id_left_hand",  &ForceUtil::set_force_id_left_hand)
    .def("get_force_id_right_hand", &ForceUtil::get_force_id_right_hand)
    .def("set_force_id_right_hand", &ForceUtil::set_force_id_right_hand)
    .def("get_force_id_left_foot",  &ForceUtil::get_force_id_left_foot)
    .def("set_force_id_left_foot",  &ForceUtil::set_force_id_left_foot)
    .def("get_force_id_right_foot", &ForceUtil::get_force_id_right_foot)
    .def("set_force_id_right_foot", &ForceUtil::set_force_id_right_foot)
    .def("display",                 &ForceUtil::display)
    ;

  class_<RobotUtil>("RobotUtil")
    .def_readwrite("m_force_util",&RobotUtil::m_force_util)
    .def_readwrite("m_foot_util",&RobotUtil::m_foot_util)
    .def_readwrite("m_urdf_to_sot",&RobotUtil::m_urdf_to_sot)
    .def("set_joint_limits_for_id",&RobotUtil::set_joint_limits_for_id)
    .def("get_joint_limits_from_id",&RobotUtil::cp_get_joint_limits_from_id)
    //.def("set_joint_limits_for_id",&RobotUtil::set_joint_limits_for_id)
    //.def("set_name_to_id", &RobotUtil::set_name_to_id)
    //.def("create_id_to_name_map",&RobotUtil::create_id_to_name_map)
    //.def("get_id_from_name",&RobotUtil::get_id_from_name)
    ;
}
