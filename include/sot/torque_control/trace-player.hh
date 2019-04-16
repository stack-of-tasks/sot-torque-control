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

#ifndef __sot_torque_control_trace_player_H__
#define __sot_torque_control_trace_player_H__

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (__sot_torque_control_trace_player_H__)
#    define SOTTRACEPLAYER_EXPORT __declspec(dllexport)
#  else
#    define SOTTRACEPLAYER_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTTRACEPLAYER_EXPORT
#endif


/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include <vector>
#include <map>
#include "boost/assign.hpp"
/* HELPER */
#include <dynamic-graph/signal-helper.h>
#include <sot/core/matrix-geometry.hh>
#include <sot/core/robot-utils.hh>
#include <sot/torque_control/utils/vector-conversions.hh>


namespace dynamicgraph {
  namespace sot {
    namespace torque_control {

      /* --------------------------------------------------------------------- */
      /* --- CLASS ----------------------------------------------------------- */
      /* --------------------------------------------------------------------- */


      /**
       * @brief Entity to play data saved using a Tracer.
       *
       * A typical use of this entity would be to call the command
       * addOutputSignal for every file you previously saved with
       * the Tracer. Then you can either call the command
       * playNext, or you can call recompute on the output
       * signal "trigger".
       */
      class SOTTRACEPLAYER_EXPORT TracePlayer
        :public::dynamicgraph::Entity
      {
        typedef TracePlayer EntityClassName;
        DYNAMIC_GRAPH_ENTITY_DECL();

      public:

        /* --- CONSTRUCTOR ---- */
        TracePlayer( const std::string & name );

        void init(const double& dt);

        /* --- SIGNALS --- */
        typedef dynamicgraph::Signal<dynamicgraph::Vector, int> OutputSignalType;
        std::map<std::string, OutputSignalType* > m_outputSignals;
        DECLARE_SIGNAL_OUT(trigger, int);

        /* --- COMMANDS --- */
        void addOutputSignal(const std::string & fileName,
                             const std::string & signalName);
        void playNext();
        void rewind();
        void clear();

        /* --- ENTITY INHERITANCE --- */
        virtual void display( std::ostream& os ) const;

        void sendMsg(const std::string& msg, MsgType t=MSG_TYPE_INFO, const char* file="", int line=0)
        {
	  Entity::sendMsg("["+name+"] "+msg, t, file, line);
        }

      protected:
        typedef dynamicgraph::Vector            DataType;
        typedef std::list< DataType >           DataHistoryType;
        typedef DataHistoryType::const_iterator DataPointerType;

        std::map<std::string, DataHistoryType> m_data;
        std::map<std::string, DataPointerType> m_dataPointers;

      }; // class TraceReader

    }    // namespace torque_control
  }      // namespace sot
}        // namespace dynamicgraph



#endif // #ifndef __sot_torque_control_trace_reader_H__
