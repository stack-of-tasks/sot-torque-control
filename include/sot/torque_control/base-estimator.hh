/*
 * Copyright 2017, Thomas Flayols, LAAS-CNRS
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

#ifndef __sot_torque_control_base_estimator_H__
#define __sot_torque_control_base_estimator_H__

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (base_estimator_EXPORTS)
#    define SOTBASEESTIMATOR_EXPORT __declspec(dllexport)
#  else
#    define SOTBASEESTIMATOR_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTBASEESTIMATOR_EXPORT
#endif


/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include <sot/torque_control/signal-helper.hh>
#include <sot/torque_control/utils/vector-conversions.hh>
#include <sot/torque_control/utils/logger.hh>
#include <sot/torque_control/hrp2-common.hh>
#include <map>
#include <initializer_list>
#include "boost/assign.hpp"
//#include <boost/random/normal_distribution.hpp>
#include <boost/math/distributions/normal.hpp> // for normal_distribution

/* Pinocchio */
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

namespace dynamicgraph {
  namespace sot {
    namespace torque_control {

      /* --------------------------------------------------------------------- */
      /* --- CLASS ----------------------------------------------------------- */
      /* --------------------------------------------------------------------- */

      /** Convert from Roll, Pitch, Yaw to transformation Matrix. */
      void rpyToMatrix(double r, double p, double y, Eigen::Matrix3d & R);

      /** Convert from Roll, Pitch, Yaw to transformation Matrix. */
      void rpyToMatrix(const Eigen::Vector3d & rpy, Eigen::Matrix3d & R);

      /**  Convert from Transformation Matrix to Roll, Pitch, Yaw */
      void matrixToRpy(const Eigen::Matrix3d & M, Eigen::Vector3d & rpy);

      class SOTBASEESTIMATOR_EXPORT BaseEstimator
          :public::dynamicgraph::Entity
      {
        typedef BaseEstimator EntityClassName;
        typedef se3::SE3 SE3;
        typedef Eigen::Vector2d Vector2;
        typedef Eigen::Vector3d Vector3;
        typedef Eigen::Vector4d Vector4;
        typedef Eigen::Vector6d Vector6;
        typedef Eigen::Matrix3d Matrix3;
        typedef boost::math::normal normal;

        DYNAMIC_GRAPH_ENTITY_DECL();
        
      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /* --- CONSTRUCTOR ---- */
        BaseEstimator( const std::string & name );

        void init(const double & dt, const std::string& urdfFile);
        void reset_foot_positions();
        void set_imu_weight(const double & w);
        void set_zmp_std_dev_right_foot(const double & std_dev);
        void set_zmp_std_dev_left_foot(const double & std_dev);
        void set_normal_force_std_dev_right_foot(const double & std_dev);
        void set_normal_force_std_dev_left_foot(const double & std_dev);
        void set_stiffness_right_foot(const dynamicgraph::Vector & k);
        void set_stiffness_left_foot(const dynamicgraph::Vector & k);

        void reset_foot_positions_impl();
        void compute_zmp(const Vector6 & w, Vector2 & zmp);
        double compute_zmp_weight(const Vector2 & zmp, const Vector4 & foot_sizes,
                                  double std_dev, double margin);
        double compute_force_weight(double fz, double std_dev, double margin);

        /* --- SIGNALS --- */
        DECLARE_SIGNAL_IN(joint_positions,            dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(joint_velocities,           dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(imu_quaternion,             dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(forceLLEG,                  dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(forceRLEG,                  dynamicgraph::Vector);
        DECLARE_SIGNAL_INNER(kinematics_computations, dynamicgraph::Vector);

        DECLARE_SIGNAL_OUT(q,                         dynamicgraph::Vector);  /// n+6 robot configuration with base6d in RPY
        DECLARE_SIGNAL_OUT(v,                         dynamicgraph::Vector);  /// n+6 robot velocities

        DECLARE_SIGNAL_OUT(q_lf,                       dynamicgraph::Vector);  /// n+6 robot configuration with base6d in RPY
        DECLARE_SIGNAL_OUT(q_rf,                       dynamicgraph::Vector);  /// n+6 robot configuration with base6d in RPY
        DECLARE_SIGNAL_OUT(q_imu,                      dynamicgraph::Vector);  /// n+6 robot configuration with base6d in RPY

        /* --- COMMANDS --- */
        /* --- ENTITY INHERITANCE --- */
        virtual void display( std::ostream& os ) const;
        virtual void commandLine(const std::string& cmdLine,
                                 std::istringstream& cmdArgs,
                                 std::ostream& os);

        void sendMsg(const std::string& msg, MsgType t=MSG_TYPE_INFO, const char* file="", int line=0)
        {
          getLogger().sendMsg("["+name+"] "+msg, t, file, line);
        }
        
      protected:
        bool              m_initSucceeded;    /// true if the entity has been successfully initialized
        bool              m_reset_foot_pos;   /// true after the command resetFootPositions is called
        double            m_dt;               /// sampling time step

        /* Estimator parameters */
        double            m_w_imu;            /// weight of IMU for sensor fusion
        double            m_zmp_std_dev_rf;   /// standard deviation of ZMP measurement errors for right foot
        double            m_zmp_std_dev_lf;   /// standard deviation of ZMP measurement errors for left foot
        double            m_fz_std_dev_rf;    /// standard deviation of normal force measurement errors for right foot
        double            m_fz_std_dev_lf;    /// standard deviation of normal force measurement errors for left foot
        Vector6           m_K_rf;             /// 6d stiffness of right foot spring
        Vector6           m_K_lf;             /// 6d stiffness of left foot spring

        se3::Model        m_model;            /// Pinocchio robot model
        se3::Data         *m_data;            /// Pinocchio robot data
        se3::SE3          m_oMff_lf;          /// world-to-base transformation obtained through left foot
        se3::SE3          m_oMff_rf;          /// world-to-base transformation obtained through right foot
        SE3               m_oMlfs;            /// transformation from world to left foot sole
        SE3               m_oMrfs;            /// transformation from world to right foot sole

        normal            m_normal;           /// Normal distribution

        SE3               m_sole_M_ftSens;    /// foot sole to F/T sensor transformation

        unsigned int      m_right_foot_id;
        unsigned int      m_left_foot_id;

        Eigen::VectorXd   m_q_pin;            /// robot configuration according to pinocchio convention
        Eigen::VectorXd   m_q_sot;            /// robot configuration according to SoT convention
        Eigen::VectorXd   m_v_pin;            /// robot velocities according to pinocchio convention
        Eigen::VectorXd   m_v_sot;            /// robot velocities according to SoT convention

      }; // class BaseEstimator
      
    }    // namespace torque_control
  }      // namespace sot
}        // namespace dynamicgraph



#endif // #ifndef __sot_torque_control_free_flyer_locator_H__
