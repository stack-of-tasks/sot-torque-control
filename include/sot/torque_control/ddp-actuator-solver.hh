
#ifndef _SOT_DDP_ACTUATOR_SOLVER_H
#define _SOT_DDP_ACTUATOR_SOLVER_H
/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (sot_ddp_actuator_EXPORTS)
#    define SOTDDPACTUATORSOLVER_EXPORT __declspec(dllexport)
#  else
#    define SOTDDPACTUATORSOLVER_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTDDPACTUATORSOLVER_EXPORT
#endif


#include <tsid/utils/stop-watch.hpp>
#include <dynamic-graph/signal-helper.h>
#include <sot/core/matrix-geometry.hh>
#include <sot/core/robot-utils.hh>
#include <sot/torque_control/utils/causal-filter.hh>

#include <ddp-actuator-solver/ddpsolver.hh>
#include <ddp-actuator-solver/ddpsolver.hh>

#include <ddp-actuator-solver/examples/dctemp.hh>
#include <ddp-actuator-solver/examples/costtemp.hh>

namespace dynamicgraph {
  namespace sot {
    namespace torque_control {

#define ALL_INPUT_SIGNALS m_pos_desSIN << m_pos_motor_measureSIN \
      << m_pos_joint_measureSIN << m_dx_measureSIN		 \
      << m_tau_measureSIN << m_temp_measureSIN 

#define ALL_OUTPUT_SIGNALS  m_tauSOUT
      
        class SOTDDPACTUATORSOLVER_EXPORT DdpActuatorSolver
	  :public :: dynamicgraph::Entity
	{
	  DYNAMIC_GRAPH_ENTITY_DECL();
	  
        public: /* --- SIGNALS --- */
	  DECLARE_SIGNAL_IN(pos_des,            dynamicgraph::Vector);
	  DECLARE_SIGNAL_IN(pos_motor_measure,  dynamicgraph::Vector);
	  DECLARE_SIGNAL_IN(pos_joint_measure,  dynamicgraph::Vector);
	  DECLARE_SIGNAL_IN(dx_measure,         dynamicgraph::Vector);
	  DECLARE_SIGNAL_IN(tau_measure,        dynamicgraph::Vector);
	  DECLARE_SIGNAL_IN(temp_measure,        dynamicgraph::Vector);	  
	  DECLARE_SIGNAL_OUT(tau,               dynamicgraph::Vector);

	protected:
	  double m_dt;
	  double m_ambiant_temperature;
	  DDPSolver<double,5,1>::stateVec_t m_xinit,m_xDes,m_x;
	  DDPSolver<double,5,1>::commandVec_t m_u;
	  DCTemp m_model;
	  CostTemp m_cost;
	  DDPSolver<double,5,1>  m_solver;
	  unsigned int m_T;
	  double m_stopCrit;
	  unsigned int m_iterMax;
	public:
	  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	    
	    /** Constructor */
	    DdpActuatorSolver(const std::string &name);
	  virtual void display(std::ostream &os) const;

	protected:
	  /** Initialize the DDP.
	   * @param timestep Control period (in seconds).
	   * @param T  Size of the preview window (in nb of timestep).
	   * @param nbItMax Maximum number of iterations.
	   * @param stopCriteria The value of the stopping criteria.
	   */
	  void param_init(const double &timestep,
			  const int &T,
			  const int &nbItMax,
			  const double &stopCriteria);
        };
    } // namespace torque_control
  } // namespace sot
} // namespace dynamicgraph
#endif // _SOT_DDP_ACTUATOR_SOLVER_H
