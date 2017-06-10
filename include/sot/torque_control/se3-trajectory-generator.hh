/*
 * Copyright 2017, Andrea Del Prete, LAAS-CNRS
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

#ifndef __sot_torque_control_se3_trajectory_generator_H__
#define __sot_torque_control_se3_trajectory_generator_H__

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (se3_position_controller_EXPORTS)
#    define SOTSE3TRAJECTORYGENERATOR_EXPORT __declspec(dllexport)
#  else
#    define SOTSE3TRAJECTORYGENERATOR_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTSE3TRAJECTORYGENERATOR_EXPORT
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

      class SOTSE3TRAJECTORYGENERATOR_EXPORT SE3TrajectoryGenerator
	:public::dynamicgraph::Entity
      {
        typedef SE3TrajectoryGenerator EntityClassName;
        DYNAMIC_GRAPH_ENTITY_DECL();
        
      public: 
        /* --- CONSTRUCTOR ---- */
        SE3TrajectoryGenerator( const std::string & name );

        void init(const double& dt);

        /* --- SIGNALS --- */
        DECLARE_SIGNAL_IN(initial_value,  dynamicgraph::Vector);
        DECLARE_SIGNAL(x,      OUT,       dynamicgraph::Vector);
        DECLARE_SIGNAL_OUT(dx,            dynamicgraph::Vector);
        DECLARE_SIGNAL_OUT(ddx,           dynamicgraph::Vector);

      protected:
        DECLARE_SIGNAL_OUT_FUNCTION(x,    dynamicgraph::Vector);

      public:

        /* --- COMMANDS --- */

        void playTrajectoryFile(const std::string& fileName);

        /** Print the current value of the specified component. */
        void getValue(const int& id);

        /** Move a component to a position with a minimum-jerk trajectory.
         * @param id integer index.
         * @param xFinal The desired final position of the component.
         * @param time The time to go from the current position to xFinal [sec].
         */
        void move(const int& id, const double& xFinal, const double& time);

        /** Start an infinite sinusoidal trajectory.
         * @param id integer index.
         * @param xFinal The position of the component corresponding to the max amplitude of the sinusoid.
         * @param time The time to go from the current position to xFinal [sec].
         */
        void startSinusoid(const int& id, const double& xFinal, const double& time);

        /** Start an infinite triangle trajectory.
         * @param id integer index.
         * @param xFinal The position of the component corresponding to the max amplitude of the trajectory.
         * @param time The time to go from the current position to xFinal [sec].
         */
        void startTriangle(const int& id, const double& xFinal, const double& time, const double& Tacc);

        /** Start an infinite trajectory with piece-wise constant acceleration.
         * @param id integer index.
         * @param xFinal The position of the component corresponding to the max amplitude of the trajectory.
         * @param time The time to go from the current position to xFinal [sec].
         * @param Tacc The time during witch acceleration is keept constant [sec].
         */
        void startConstAcc(const int& id, const double& xFinal, const double& time);

        /** Start a linear-chirp trajectory, that is a sinusoidal trajectory with frequency
         * being a linear function of time.
         * @param id integer index.
         * @param xFinal The position of the component corresponding to the max amplitude of the sinusoid [rad].
         * @param f0 The initial (min) frequency of the sinusoid [Hz]
         * @param f1 The final (max) frequency of the sinusoid [Hz]
         * @param time The time to get from f0 to f1 [sec]
         */
        void startLinearChirp(const int& id, const double& xFinal, const double& f0, const double& f1, const double& time);

        /** Stop the motion of the specified component. If id is -1
         * it stops the trajectory of all the vector.
         * @param id integer index.
         * */
        void stop(const int& id);

        /* --- ENTITY INHERITANCE --- */
        virtual void display( std::ostream& os ) const;
        virtual void commandLine(const std::string& cmdLine,
                                 std::istringstream& cmdArgs,
                                 std::ostream& os);

        void sendMsg(const std::string& msg, MsgType t=MSG_TYPE_INFO, const char* file="", int line=0)
        {
          getLogger().sendMsg("[SE3TrajectoryGenerator-"+name+"] "+msg, t, file, line);
        }
        
      protected:
        enum TG_Status
        {
          TG_STOP,
          TG_SINUSOID,
          TG_MIN_JERK,
          TG_LIN_CHIRP,
          TG_TRIANGLE,
          TG_CONST_ACC,
          TG_TEXT_FILE
        };

        bool              m_initSucceeded;    /// true if the entity has been successfully initialized
        bool              m_firstIter;        /// true if it is the first iteration, false otherwise
        double            m_dt;               /// control loop time period
        unsigned int      m_np;               /// size of position vector
        unsigned int      m_nv;               /// size of velocity vector
        unsigned int      m_iterLast;         /// last iter index

        std::vector<TG_Status> m_status;     /// status of the component
        std::vector<AbstractTrajectoryGenerator*>    m_currentTrajGen;
        std::vector<NoTrajectoryGenerator*>          m_noTrajGen;
        std::vector<MinimumJerkTrajectoryGenerator*> m_minJerkTrajGen;
        std::vector<SinusoidTrajectoryGenerator*>    m_sinTrajGen;
        std::vector<LinearChirpTrajectoryGenerator*> m_linChirpTrajGen;
        std::vector<TriangleTrajectoryGenerator*>    m_triangleTrajGen;
        std::vector<ConstantAccelerationTrajectoryGenerator*>    m_constAccTrajGen;
        TextFileTrajectoryGenerator*                 m_textFileTrajGen;

      }; // class SE3TrajectoryGenerator
      
    }    // namespace torque_control
  }      // namespace sot
}        // namespace dynamicgraph



#endif // #ifndef __sot_torque_control_nd_trajectory_generator_H__
