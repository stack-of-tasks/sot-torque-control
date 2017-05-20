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


#ifndef __sot_torque_control_common_H__
#define __sot_torque_control_common_H__

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (hrp2_common_EXPORTS)
#    define HRP2COMMON_EXPORT __declspec(dllexport)
#  else
#    define HRP2COMMON_EXPORT __declspec(dllimport)
#  endif
#else
#  define HRP2COMMON_EXPORT
#endif


/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include <sot/torque_control/signal-helper.hh>
#include <sot/torque_control/utils/vector-conversions.hh>
#include <map>
#include <initializer_list>
#include "boost/assign.hpp"


namespace dynamicgraph {
  namespace sot {
    namespace torque_control {
      
      struct FromURDFToSoT
      {
      protected:
	std::vector<unsigned int> m_urdf_to_sot;
	unsigned int m_nbJoints;
	
      public:
	void set_urdf_to_sot(const std::vector<unsigned int> &urdf_to_sot);
	
	bool joints_urdf_to_sot(Eigen::ConstRefVector q_urdf, Eigen::RefVector q_sot);

	bool joints_sot_to_urdf(Eigen::ConstRefVector q_sot, Eigen::RefVector q_urdf);

      }; // struct FromURDFToSoT


      bool base_se3_to_sot(Eigen::ConstRefVector pos,
                           Eigen::ConstRefMatrix R,
                           Eigen::RefVector q_sot);
      bool base_urdf_to_sot(Eigen::ConstRefVector q_urdf, Eigen::RefVector q_sot);
      bool base_sot_to_urdf(Eigen::ConstRefVector q_sot, Eigen::RefVector q_urdf);
      bool config_urdf_to_sot(Eigen::ConstRefVector q_urdf, Eigen::RefVector q_sot);
      bool config_sot_to_urdf(Eigen::ConstRefVector q_sot, Eigen::RefVector q_urdf);
      bool velocity_urdf_to_sot(Eigen::ConstRefVector v_urdf, Eigen::RefVector v_sot);
      bool velocity_sot_to_urdf(Eigen::ConstRefVector v_sot, Eigen::RefVector v_urdf);
      bool joints_urdf_to_sot(Eigen::ConstRefVector q_urdf, Eigen::RefVector q_sot);
      bool joints_sot_to_urdf(Eigen::ConstRefVector q_sot, Eigen::RefVector q_urdf);

    }    // namespace torque_control
  }      // namespace sot
}        // namespace dynamicgraph

#endif // sot_torque_control_common_h_
