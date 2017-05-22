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
#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>


namespace dynamicgraph
{
  namespace sot
  {
    namespace torque_control
    {
      namespace dg = ::dynamicgraph;
      using namespace dg;
      using namespace dg::command;

      void FromURDFToSoT::
      set_urdf_to_sot(const std::vector<unsigned int> &urdf_to_sot)
      {
	m_urdf_to_sot.resize(urdf_to_sot.size());
	m_dgv_urdf_to_sot.resize(urdf_to_sot.size());
	for(unsigned int idx=0;idx<urdf_to_sot.size();idx++)
	  {
	    m_urdf_to_sot[idx] = urdf_to_sot[idx];
	    m_dgv_urdf_to_sot[idx] = urdf_to_sot[idx];
	  }
      }
      
      void FromURDFToSoT::
      set_urdf_to_sot(const dg::Vector &urdf_to_sot)
      {
	m_urdf_to_sot.resize(urdf_to_sot.size());
	for(unsigned int idx=0;idx<urdf_to_sot.size();idx++)
	  {
	    m_urdf_to_sot[idx] = (unsigned int)urdf_to_sot[idx];
	  }
	m_dgv_urdf_to_sot = urdf_to_sot;
      }
      
      bool FromURDFToSoT::
      joints_urdf_to_sot(Eigen::ConstRefVector q_urdf, Eigen::RefVector q_sot)
      {
	assert(q_urdf.size()==N_JOINTS);
	assert(q_sot.size()==N_JOINTS);
	
	for(unsigned int idx=0;idx<m_nbJoints;idx++)
	  q_sot[m_urdf_to_sot[idx]]=q_urdf[idx];	
	return true;
      }
      
      bool FromURDFToSoT::
      joints_sot_to_urdf(Eigen::ConstRefVector q_sot, Eigen::RefVector q_urdf)
      {
	assert(q_urdf.size()==N_JOINTS);
	assert(q_sot.size()==N_JOINTS);
	
	for(unsigned int idx=0;idx<m_nbJoints;idx++)
	  q_urdf[idx]=q_sot[m_urdf_to_sot[idx]];	
	return true;
      }

	
      bool base_se3_to_sot(Eigen::ConstRefVector pos,
			   Eigen::ConstRefMatrix R,
			   Eigen::RefVector q_sot)
      {
	assert(q_sot.size()==6);
	assert(pos.size()==3);
	assert(R.rows()==3);
	assert(R.cols()==3);
	// ********* Quat to RPY *********
	double r,p,y,m;
	m = sqrt(R(2, 1) * R(2, 1) + R(2, 2) * R(2, 2));
	p = atan2(-R(2, 0), m);
	if (fabs(fabs(p) - M_PI / 2) < 0.001 )
	  {
	    r = 0.0;
	    y = -atan2(R(0, 1), R(1, 1));
	  }
	else
	  {
	    y = atan2(R(1, 0), R(0, 0)) ;
	    r = atan2(R(2, 1), R(2, 2)) ;
	  }
	// *********************************
	q_sot[0 ]=pos[0 ];
	q_sot[1 ]=pos[1 ];
	q_sot[2 ]=pos[2 ];
	q_sot[3 ]=r;
	q_sot[4 ]=p;
	q_sot[5 ]=y;
	return true;
      }
	
      bool base_urdf_to_sot(Eigen::ConstRefVector q_urdf, Eigen::RefVector q_sot)
      {
	assert(q_urdf.size()==7);
	assert(q_sot.size()==6);
	// ********* Quat to RPY *********
	const double W = q_urdf[6];
	const double X = q_urdf[3];
	const double Y = q_urdf[4];
	const double Z = q_urdf[5];
	const Eigen::Matrix3d R = Eigen::Quaterniond(W, X, Y, Z).toRotationMatrix();
	return base_se3_to_sot(q_urdf.head<3>(), R, q_sot);
      }

      bool base_sot_to_urdf(Eigen::ConstRefVector q_sot, Eigen::RefVector q_urdf)
      {
	assert(q_urdf.size()==7);
	assert(q_sot.size()==6);
	// *********  RPY to Quat *********
	const double r = q_sot[3];
	const double p = q_sot[4];
	const double y = q_sot[5];
	const Eigen::AngleAxisd  rollAngle(r, Eigen::Vector3d::UnitX());
	const Eigen::AngleAxisd pitchAngle(p, Eigen::Vector3d::UnitY());
	const Eigen::AngleAxisd   yawAngle(y, Eigen::Vector3d::UnitZ());
	const Eigen::Quaternion<double> quat = yawAngle * pitchAngle * rollAngle;
	  
	q_urdf[0 ]=q_sot[0 ]; //BASE
	q_urdf[1 ]=q_sot[1 ];
	q_urdf[2 ]=q_sot[2 ];
	q_urdf[3 ]=quat.x();
	q_urdf[4 ]=quat.y();
	q_urdf[5 ]=quat.z();
	q_urdf[6 ]=quat.w();
	  
	return true;
      }
    } // torque_control
  } // sot
} // dynamic_graph
