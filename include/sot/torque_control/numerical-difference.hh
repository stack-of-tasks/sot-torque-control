/*
 * Copyright 2014, Andrea Del Prete, LAAS-CNRS
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

#ifndef __sot_torque_control_NumericalDifference_H__
#define __sot_torque_control_NumericalDifference_H__
/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (numerical_difference_EXPORTS)
#    define SOTNUMERICALDIFFERENCE_EXPORT __declspec(dllexport)
#  else
#    define SOTNUMERICALDIFFERENCE_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTNUMERICALDIFFERENCE_EXPORT
#endif

//#define VP_DEBUG 1        /// enable debug output
//#define VP_DEBUG_MODE 20

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* HELPER */
#include <sot/torque_control/signal-helper.hh>
#include <sot/torque_control/utils/vector-conversions.hh>
#include <sot/torque_control/utils/stop-watch.hh>
#include <sot/torque_control/utils/logger.hh>
#include <boost/circular_buffer.hpp>

/* Polynomial estimators */
#include <sot/torque_control/utils/lin-estimator.hh>
#include <sot/torque_control/utils/quad-estimator.hh>

namespace dynamicgraph {
  namespace sot {
    namespace torque_control {




      /**
        * This Entity takes as inputs a signal and estimates its first two time derivatives.
        */
      class SOTNUMERICALDIFFERENCE_EXPORT NumericalDifference
          :public ::dynamicgraph::Entity
      {
        DYNAMIC_GRAPH_ENTITY_DECL();

      public:  /* --- SIGNALS --- */
        DECLARE_SIGNAL_IN(x,                 dynamicgraph::Vector);
        DECLARE_SIGNAL_OUT(x_filtered,       dynamicgraph::Vector);
        DECLARE_SIGNAL_OUT(dx,               dynamicgraph::Vector);
        DECLARE_SIGNAL_OUT(ddx,              dynamicgraph::Vector);

        /// The following inner signals are used because this entity has some output signals
        /// whose related quantities are computed at the same time by the same algorithm
        /// To avoid the risk of recomputing the same things twice, we create an inner signal that groups together
        /// all the quantities that are computed together. Then the single output signals will depend
        /// on this inner signal, which is the one triggering the computations.
        /// Inner signals are not exposed, so that nobody can access them.

        /// This signal contains the estimated positions, velocities and accelerations.
        DECLARE_SIGNAL_INNER(x_dx_ddx,              dynamicgraph::Vector);
        
      protected:
      
        double m_dt;      /// sampling timestep of the input signal
        double m_delay;   /// delay introduced by the estimation
        int x_size;
        /// std::vector to use with the filters
        /// All the variables whose name contains 'filter' are outputs of the filters
        std::vector<double> m_ddx_filter_std;  /// 2nd derivative
        std::vector<double> m_dx_filter_std;   /// 1st derivative
        std::vector<double> m_x_filter_std;    /// filtered output
        std::vector<double> m_x_std;           /// x signal


        /// polynomial-fitting filters
        PolyEstimator* m_filter;

      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /** --- CONSTRUCTOR ---- */
        NumericalDifference( const std::string & name );

        /** Initialize the NumericalDifference.
         * @param timestep Period (in seconds) after which the sensors' data are updated.
         * @param sigSize  Size of the input signal.
         * @param delay    Delay (in seconds) introduced by the estimation.
         *                 This should be a multiple of timestep.
         * @note The estimationDelay is half of the length of the window used for the
         * polynomial fitting. The larger the delay, the smoother the estimations.
         */
        void init(const double &timestep, const int& sigSize, const double &delay);

      protected:
        void sendMsg(const std::string& msg, MsgType t=MSG_TYPE_INFO, const char* file="", int line=0)
        {
          getLogger().sendMsg("["+name+"] "+msg, t, file, line);
        }

      public: /* --- ENTITY INHERITANCE --- */
        virtual void display( std::ostream& os ) const;

      }; // class NumericalDifference

    } // namespace torque_control
  } // namespace sot
} // namespace dynamicgraph



#endif // #ifndef __sot_torque_control_NumericalDifference_H__
