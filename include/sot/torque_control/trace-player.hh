/*
 * Copyright 2017, Andrea Del Prete, LAAS-CNRS
 *
 */

#ifndef __sot_torque_control_trace_player_H__
#define __sot_torque_control_trace_player_H__

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined(WIN32)
#if defined(__sot_torque_control_trace_player_H__)
#define SOTTRACEPLAYER_EXPORT __declspec(dllexport)
#else
#define SOTTRACEPLAYER_EXPORT __declspec(dllimport)
#endif
#else
#define SOTTRACEPLAYER_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include <sot/core/robot-utils.hh>
#include <vector>
#include <map>
#include "boost/assign.hpp"
/* HELPER */
#include <dynamic-graph/signal-helper.h>
#include <sot/core/matrix-geometry.hh>
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
class SOTTRACEPLAYER_EXPORT TracePlayer : public ::dynamicgraph::Entity {
  typedef TracePlayer EntityClassName;
  DYNAMIC_GRAPH_ENTITY_DECL();

 public:
  /* --- CONSTRUCTOR ---- */
  TracePlayer(const std::string& name);

  void init(const double& dt);

  /* --- SIGNALS --- */
  typedef dynamicgraph::Signal<dynamicgraph::Vector, int> OutputSignalType;
  std::map<std::string, OutputSignalType*> m_outputSignals;
  DECLARE_SIGNAL_OUT(trigger, int);

  /* --- COMMANDS --- */
  void addOutputSignal(const std::string& fileName, const std::string& signalName);
  void playNext();
  void rewind();
  void clear();

  /* --- ENTITY INHERITANCE --- */
  virtual void display(std::ostream& os) const;

 protected:
  typedef dynamicgraph::Vector DataType;
  typedef std::list<DataType> DataHistoryType;
  typedef DataHistoryType::const_iterator DataPointerType;

  std::map<std::string, DataHistoryType> m_data;
  std::map<std::string, DataPointerType> m_dataPointers;

};  // class TraceReader

}  // namespace torque_control
}  // namespace sot
}  // namespace dynamicgraph

#endif  // #ifndef __sot_torque_control_trace_reader_H__
