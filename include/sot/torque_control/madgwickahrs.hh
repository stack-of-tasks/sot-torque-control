//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
// 11/05/2017   T Flayols       Make it a dynamic graph entity
//
//=====================================================================================================

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

#ifndef __sot_torque_control_madgwickahrs_H__
#define __sot_torque_control_madgwickahrs_H__

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (madgwickahrs_EXPORTS)
#    define SOTMADGWICKAHRS_EXPORT __declspec(dllexport)
#  else
#    define SOTMADGWICKAHRS_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTMADGWICKAHRS_EXPORT
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

#define betaDef		0.01f		// 2 * proportional g

namespace dynamicgraph {
  namespace sot {
    namespace torque_control {

      /* --------------------------------------------------------------------- */
      /* --- CLASS ----------------------------------------------------------- */
      /* --------------------------------------------------------------------- */

      class SOTMADGWICKAHRS_EXPORT MadgwickAHRS
          :public::dynamicgraph::Entity
      {
        typedef MadgwickAHRS EntityClassName;
        DYNAMIC_GRAPH_ENTITY_DECL();

      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /* --- CONSTRUCTOR ---- */
        MadgwickAHRS( const std::string & name );

        void init(const double& dt);
        void set_beta(const double & beta);

        /* --- SIGNALS --- */
        DECLARE_SIGNAL_IN(accelerometer,              dynamicgraph::Vector);  /// ax ay az in m.s-2
        DECLARE_SIGNAL_IN(gyroscope,                  dynamicgraph::Vector);  /// gx gy gz in rad.s-1
        DECLARE_SIGNAL(imu_quat, OUT,                 dynamicgraph::Vector);  /// Estimated orientation of IMU as a quaternion

      protected:
        DECLARE_SIGNAL_OUT_FUNCTION(imu_quat,         dynamicgraph::Vector);

        /* --- COMMANDS --- */
        /* --- ENTITY INHERITANCE --- */
        virtual void display( std::ostream& os ) const;
        virtual void commandLine(const std::string& cmdLine,
                                 std::istringstream& cmdArgs,
                                 std::ostream& os);
        /* --- METHODS --- */
        float invSqrt(float x);
        void madgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) ;
        //void madgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
        void sendMsg(const std::string& msg, MsgType t=MSG_TYPE_INFO, const char* file="", int line=0)
        {
          getLogger().sendMsg("[MadgwickAHRS-"+name+"] "+msg, t, file, line);
        }

      protected:
        bool              m_initSucceeded;                                      /// true if the entity has been successfully initialized
        volatile float    m_beta = betaDef;								        /// 2 * proportional gain (Kp)
        volatile float    m_q0 = 1.0f, m_q1 = 0.0f, m_q2 = 0.0f, m_q3 = 0.0f;	/// quaternion of sensor frame
        float             m_sampleFreq = 512.0f;		                        /// sample frequency in Hz

      }; // class MadgwickAHRS
    }    // namespace torque_control
  }      // namespace sot
}        // namespace dynamicgraph

#endif // #ifndef __sot_torque_control_madgwickahrs_H__
