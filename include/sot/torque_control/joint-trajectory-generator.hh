/*
 * Copyright 2014, Oscar E. Ramos Ponce, LAAS-CNRS
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

#ifndef __sot_torque_control_joint_trajectory_generator_H__
#define __sot_torque_control_joint_trajectory_generator_H__

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (joint_position_controller_EXPORTS)
#    define SOTJOINTTRAJECTORYGENERATOR_EXPORT __declspec(dllexport)
#  else
#    define SOTJOINTTRAJECTORYGENERATOR_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTJOINTTRAJECTORYGENERATOR_EXPORT
#endif


/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include <sot/torque_control/signal-helper.hh>
#include <sot/torque_control/utils/vector-conversions.hh>
#include <sot/torque_control/utils/logger.hh>
#include <sot/torque_control/utils/trajectory-generators.hh>
#include <map>
#include <initializer_list>
#include "boost/assign.hpp"


namespace dynamicgraph {
  namespace sot {
    namespace torque_control {

      /* --------------------------------------------------------------------- */
      /* --- CLASS ----------------------------------------------------------- */
      /* --------------------------------------------------------------------- */

      class SOTJOINTTRAJECTORYGENERATOR_EXPORT JointTrajectoryGenerator
	:public::dynamicgraph::Entity
      {
        typedef JointTrajectoryGenerator EntityClassName;
        DYNAMIC_GRAPH_ENTITY_DECL();
        
      public: 
        /* --- CONSTRUCTOR ---- */
        JointTrajectoryGenerator( const std::string & name );

        void init(const double& dt);

        /* --- SIGNALS --- */
        DECLARE_SIGNAL_IN(base6d_encoders,  ml::Vector);
        DECLARE_SIGNAL_OUT(q,               ml::Vector);
        DECLARE_SIGNAL_OUT(dq,              ml::Vector);
        DECLARE_SIGNAL_OUT(ddq,             ml::Vector);
        DECLARE_SIGNAL(fRightFoot, OUT,     ml::Vector);
        DECLARE_SIGNAL(fLeftFoot,  OUT,     ml::Vector);
        DECLARE_SIGNAL(fRightHand, OUT,     ml::Vector);
        DECLARE_SIGNAL(fLeftHand,  OUT,     ml::Vector);

      protected:
        DECLARE_SIGNAL_OUT_FUNCTION(fRightFoot, ml::Vector);
        DECLARE_SIGNAL_OUT_FUNCTION(fLeftFoot,  ml::Vector);
        DECLARE_SIGNAL_OUT_FUNCTION(fRightHand, ml::Vector);
        DECLARE_SIGNAL_OUT_FUNCTION(fLeftHand,  ml::Vector);

      public:

        /* --- COMMANDS --- */

        void playTrajectoryFile(const std::string& fileName);

        /** Print the current angle of the specified joint. */
        void getJoint(const std::string& jointName);

        /** Move a joint to a position with a minimum-jerk trajectory.
         * @param jointName The short name of the joint.
         * @param qFinal The desired final position of the joint [rad].
         * @param time The time to go from the current position to qFinal [sec].
         */
        void moveJoint(const std::string& jointName, const double& qFinal, const double& time);
        void moveForce(const std::string& forceName, const int& axis, const double& fFinal, const double& time);

        /** Start an infinite sinusoidal trajectory.
         * @param jointName The short name of the joint.
         * @param qFinal The position of the joint corresponding to the max amplitude of the sinusoid [rad].
         * @param time The time to go from the current position to qFinal [sec].
         */
        void startSinusoid(const std::string& jointName, const double& qFinal, const double& time);

        /** Start an infinite triangle trajectory.
         * @param jointName The short name of the joint.
         * @param qFinal The position of the joint corresponding to the max amplitude of the trajectory [rad].
         * @param time The time to go from the current position to qFinal [sec].
         */
        void startTriangle(const std::string& jointName, const double& qFinal, const double& time, const double& Tacc);

        /** Start an infinite trajectory with piece-wise constant acceleration.
         * @param jointName The short name of the joint.
         * @param qFinal The position of the joint corresponding to the max amplitude of the trajectory [rad].
         * @param time The time to go from the current position to qFinal [sec].
         * @param Tacc The time during witch acceleration is keept constant [sec].
         */
        void startConstAcc(const std::string& jointName, const double& qFinal, const double& time);

        /** Start an infinite sinusoidal trajectory for the specified force signal.
         * @param forceName The short name of the force signal (rh, lh, rf, lf).
         * @param fFinal The 6d force corresponding to the max amplitude of the sinusoid [N/Nm].
         * @param time The time to go from 0 to fFinal [sec].
         */
//        void startForceSinusoid(const std::string& forceName, const ml::Vector& fFinal, const double& time);
        void startForceSinusoid(const std::string& forceName, const int& axis, const double& fFinal, const double& time);

        /** Start a linear-chirp trajectory, that is a sinusoidal trajectory with frequency
         * being a linear function of time.
         * @param jointName The short name of the joint.
         * @param qFinal The position of the joint corresponding to the max amplitude of the sinusoid [rad].
         * @param f0 The initial (min) frequency of the sinusoid [Hz]
         * @param f1 The final (max) frequency of the sinusoid [Hz]
         * @param time The time to get from f0 to f1 [sec]
         */
        void startLinearChirp(const std::string& jointName, const double& qFinal, const double& f0, const double& f1, const double& time);

        void startForceLinearChirp(const std::string& forceName, const int& axis, const double& fFinal, const double& f0, const double& f1, const double& time);

        /** Stop the motion of the specified joint. If jointName is "all"
         * it stops the motion of all joints.
         * @param jointName A string identifying the joint to stop.
         * */
        void stop(const std::string& jointName);

        /** Stop the trajectory of the specified force. If forceName is "all"
         * it stops all forces.
         * @param forceName A string identifying the force to stop.
         * */
        void stopForce(const std::string& forceName);
        
        /* --- ENTITY INHERITANCE --- */
        virtual void display( std::ostream& os ) const;
        virtual void commandLine(const std::string& cmdLine,
                                 std::istringstream& cmdArgs,
                                 std::ostream& os);

        void sendMsg(const std::string& msg, MsgType t=MSG_TYPE_INFO, const char* file="", int line=0)
        {
          getLogger().sendMsg("[JointTrajectoryGenerator-"+name+"] "+msg, t, file, line);
        }
        
      protected:
        enum JTG_Status
        {
          JTG_STOP,
          JTG_SINUSOID,
          JTG_MIN_JERK,
          JTG_LIN_CHIRP,
          JTG_TRIANGLE,
          JTG_CONST_ACC,
          JTG_TEXT_FILE
        };

        bool              m_initSucceeded;    /// true if the entity has been successfully initialized
        bool              m_firstIter;        /// true if it is the first iteration, false otherwise
        double            m_dt;               /// control loop time period

        std::vector<int>  m_iterForceSignals;

        std::vector<JTG_Status> m_status;     /// status of the joints
        std::vector<AbstractTrajectoryGenerator*>    m_currentTrajGen;
        std::vector<NoTrajectoryGenerator*>          m_noTrajGen;
        std::vector<MinimumJerkTrajectoryGenerator*> m_minJerkTrajGen;
        std::vector<SinusoidTrajectoryGenerator*>    m_sinTrajGen;
        std::vector<LinearChirpTrajectoryGenerator*> m_linChirpTrajGen;
        std::vector<TriangleTrajectoryGenerator*>    m_triangleTrajGen;
        std::vector<ConstantAccelerationTrajectoryGenerator*>    m_constAccTrajGen;
        TextFileTrajectoryGenerator*                 m_textFileTrajGen;

        std::vector<JTG_Status> m_status_force;     /// status of the forces
        std::vector<AbstractTrajectoryGenerator*>    m_currentTrajGen_force;
        std::vector<NoTrajectoryGenerator*>          m_noTrajGen_force;
        std::vector<SinusoidTrajectoryGenerator*>    m_sinTrajGen_force;
        std::vector<MinimumJerkTrajectoryGenerator*> m_minJerkTrajGen_force;
        std::vector<LinearChirpTrajectoryGenerator*> m_linChirpTrajGen_force;

        bool generateReferenceForceSignal(const std::string& forceName, int fid, ml::Vector& s, int iter);

        bool convertJointNameToJointId(const std::string& name, unsigned int& id);
        bool convertForceNameToForceId(const std::string& name, unsigned int& id);
        bool isJointInRange(unsigned int id, double q);
        bool isForceInRange(unsigned int id, const Eigen::VectorXd& f);
        bool isForceInRange(unsigned int id, int axis, double f);

      }; // class JointTrajectoryGenerator
      
    }    // namespace torque_control
  }      // namespace sot
}        // namespace dynamicgraph



#endif // #ifndef __sot_torque_control_joint_position_controller_H__
