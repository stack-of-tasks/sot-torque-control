/*
 * Copyright 2014-2017, Andrea Del Prete, Rohan Budhiraja LAAS-CNRS
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

#ifndef __sot_torque_control_TorqueOffsetEstimator_H__
#define __sot_torque_control_TorqueOffsetEstimator_H__
/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (torque_offset_estimator_EXPORTS)
#    define TORQUEOFFSETESTIMATOR_EXPORT __declspec(dllexport)
#  else
#    define TORQUEOFFSETESTIMATOR_EXPORT __declspec(dllimport)
#  endif
#else
#  define TORQUEOFFSETESTIMATOR_EXPORT
#endif

//#define VP_DEBUG 1        /// enable debug output
//#define VP_DEBUG_MODE 20

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */


#include <boost/circular_buffer.hpp>
#include <Eigen/StdVector>

/*Motor model*/
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

/* HELPER */
#include <dynamic-graph/signal-helper.h>
#include <sot/core/matrix-geometry.hh>
#include <sot/core/robot-utils.hh>
#include <sot/core/stop-watch.hh>

/*Motor model*/
#include <sot/torque_control/motor-model.hh>


namespace dynamicgraph {
  namespace sot {
    namespace torque_control {

      class TORQUEOFFSETESTIMATOR_EXPORT TorqueOffsetEstimator
          :public ::dynamicgraph::Entity
      {
        DYNAMIC_GRAPH_ENTITY_DECL();

      public:  /* --- SIGNALS --- */
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
          typedef int dummy;
        typedef
          std::vector<Eigen::VectorXd,Eigen::aligned_allocator<Eigen::VectorXd> > stdAlignedVector;

        /** --- CONSTRUCTOR ---- */
        TorqueOffsetEstimator( const std::string & name );
        void init(const std::string &urdfFile,
                  const Eigen::Matrix4d& _m_torso_X_imu,
                  const double& gyro_epsilon,
                  const std::string& ffJointName,
                  const std::string& torsoJointName);
        void computeOffset(const int& nIterations, const double& epsilon);

        virtual void display( std::ostream& os ) const;




        DECLARE_SIGNAL_IN(base6d_encoders,          dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(accelerometer,            dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(gyroscope,                dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(jointTorques,             dynamicgraph::Vector);
        DECLARE_SIGNAL_INNER(collectSensorData,     dummy);
        DECLARE_SIGNAL_OUT(jointTorquesEstimated,   dynamicgraph::Vector);	

      protected:
        RobotUtilShrPtr       m_robot_util;
        pinocchio::Model  m_model;            /// Pinocchio robot model
        pinocchio::Data   *m_data;            /// Pinocchio robot data 
        int n_iterations;   //Number of iterations to consider
        double epsilon;
        double gyro_epsilon;

	pinocchio::JointIndex ffIndex, torsoIndex;  //Index of the free-flyer and torso frames
        Eigen::VectorXd jointTorqueOffsets;
        pinocchio::SE3 m_torso_X_imu; // Definition of the imu in the chest frame.

        // stdAlignedVector encSignals;
        // stdAlignedVector accSignals;
        // stdAlignedVector gyrSignals;
        // stdAlignedVector tauSignals;

        // stdAlignedVector stdVecJointTorqueOffsets;

        void sendMsg(const std::string& msg, MsgType t=MSG_TYPE_INFO, const char* file="", int line=0)
        {
	  Entity::sendMsg("["+name+"] "+msg, t, file, line);
        }

      private:
        enum {
          PRECOMPUTATION,
          INPROGRESS,
          COMPUTED } sensor_offset_status;

        // void calculateSensorOffsets();
        int current_progress;
      }; // class TorqueOffsetEstimator

    } // namespace torque_control
  } // namespace sot
} // namespace dynamicgraph



#endif // #ifndef __sot_torque_control_TorqueOffsetEstimator_H__
