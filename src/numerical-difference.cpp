/*
 * Copyright 2014, Andrea Del Prete, LAAS-CNRS
 *
 * This file is part of sot-torque-control.
 * sot-dyninv is free software: you can redistribute it and/or
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

#include <sot/torque_control/numerical-difference.hh>
#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>
#include <sot/torque_control/commands-helper.hh>
#include <sot/torque_control/motor-model.hh>
#include <Eigen/Dense>

namespace dynamicgraph
{
  namespace sot
  {
    namespace torque_control
    {

#define ALL_INPUT_SIGNALS m_xSIN

#define ALL_OUTPUT_SIGNALS  m_x_filteredSOUT << m_dxSOUT  << m_ddxSOUT

      namespace dynamicgraph = ::dynamicgraph;
      using namespace dynamicgraph;
      using namespace dynamicgraph::command;
      using namespace metapod;
      using namespace Eigen;

      /// Define EntityClassName here rather than in the header file
      /// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
      typedef NumericalDifference EntityClassName;

      /* --- DG FACTORY ------------------------------------------------------- */
      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(NumericalDifference,"NumericalDifference");

      /* --- CONSTRUCTION ----------------------------------------------------- */
      /* --- CONSTRUCTION ----------------------------------------------------- */
      /* --- CONSTRUCTION ----------------------------------------------------- */
      NumericalDifference::
      NumericalDifference( const std::string & name )
        : Entity(name),
          CONSTRUCT_SIGNAL_IN(x,                dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_OUT(x_filtered,       dynamicgraph::Vector, m_x_dx_ddxSINNER)
        ,CONSTRUCT_SIGNAL_OUT(dx,               dynamicgraph::Vector, m_x_dx_ddxSINNER)
        ,CONSTRUCT_SIGNAL_OUT(ddx,              dynamicgraph::Vector, m_x_dx_ddxSINNER)
        ,CONSTRUCT_SIGNAL_INNER(x_dx_ddx,       dynamicgraph::Vector, m_xSIN)
      {
        Entity::signalRegistration( ALL_INPUT_SIGNALS << ALL_OUTPUT_SIGNALS);
        
        /* Commands. */
        addCommand("getTimestep",
                   makeDirectGetter(*this,&m_dt,
                                    docDirectGetter("Control timestep [s ]","double")));
        addCommand("getDelay",
                   makeDirectGetter(*this,&m_delay,
                                    docDirectGetter("Delay in the estimation of signal x","double")));
        addCommand("init", makeCommandVoid2(*this, &NumericalDifference::init,
                              docCommandVoid2("Initialize the estimator.",
                                              "Control timestep [s].",
                                              "Estimation delay for signal x")));
      }


      /* --- COMMANDS ---------------------------------------------------------- */
      /* --- COMMANDS ---------------------------------------------------------- */
      /* --- COMMANDS ---------------------------------------------------------- */
      void NumericalDifference::init(const double &timestep, const double& delay)
      {
        assert(timestep>0.0 && "Timestep should be > 0");
        assert(delay>=1.5*timestep && "Estimation delay should be >= 1.5*timestep");
        m_dt = timestep;
        m_delay = delay;
        int winSizeEnc     = (int)(2*delay/m_dt);
        assert(winSizeEnc>=3 && "Estimation-window's length should be >= 3");

        m_filter         = new QuadEstimator(winSizeEnc, N_JOINTS, m_dt);

        m_ddx_filter_std.resize(N_JOINTS);
        m_dx_filter_std.resize(N_JOINTS);
        m_x_filter_std.resize(N_JOINTS);
        m_x_std.resize(N_JOINTS);
      }

      /* --- SIGNALS ---------------------------------------------------------- */
      /* --- SIGNALS ---------------------------------------------------------- */
      /* --- SIGNALS ---------------------------------------------------------- */

      /** Estimate the joints' positions, velocities and accelerations. */
      DEFINE_SIGNAL_INNER_FUNCTION(x_dx_ddx, dynamicgraph::Vector)
      {
        sotDEBUG(15)<<"Compute x_dx_ddx inner signal "<<iter<<std::endl;

        // read encoders and copy in std vector
        const dynamicgraph::Vector& base_x = m_xSIN(iter);
        COPY_SHIFTED_VECTOR_TO_ARRAY(base_x, m_x_std, 6);

        // estimate joints' pos, vel and acc
        m_filter->estimate(m_x_filter_std, m_x_std);
        m_filter->getEstimateDerivative(m_dx_filter_std, 1);
        m_filter->getEstimateDerivative(m_ddx_filter_std, 2);

        // copy data in signal vector
        if(s.size()!=3*N_JOINTS)
          s.resize(3*N_JOINTS);
        for(int i=0; i<N_JOINTS; i++)
          s(i) = m_x_filter_std[i];
        for(int i=0; i<N_JOINTS; i++)
          s(i+N_JOINTS) = m_dx_filter_std[i];
        for(int i=0; i<N_JOINTS; i++)
          s(i+2*N_JOINTS) = m_ddx_filter_std[i];

        return s;
      }


      /// ************************************************************************* ///
      /// The following signals depend only on other inner signals, so they
      /// just need to copy the interested part of the inner signal they depend on.
      /// ************************************************************************* ///


      DEFINE_SIGNAL_OUT_FUNCTION(x_filtered, dynamicgraph::Vector)
      {
        sotDEBUG(15)<<"Compute x_filtered output signal "<<iter<<std::endl;

        const dynamicgraph::Vector &x_dx_ddx = m_x_dx_ddxSINNER(iter);
        if(s.size()!=N_JOINTS)
          s.resize(N_JOINTS);
	s.head<N_JOINTS>() = x_dx_ddx.head<N_JOINTS>();
        return s;
      }


      DEFINE_SIGNAL_OUT_FUNCTION(dx, dynamicgraph::Vector)
      {
        sotDEBUG(15)<<"Compute dx output signal "<<iter<<std::endl;

        const dynamicgraph::Vector &x_dx_ddx = m_x_dx_ddxSINNER(iter);
        if(s.size()!=N_JOINTS)
	  s.resize(N_JOINTS);
	s = x_dx_ddx.segment<N_JOINTS>(N_JOINTS);
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(ddx, dynamicgraph::Vector)
      {
        sotDEBUG(15)<<"Compute ddx output signal "<<iter<<std::endl;

        const dynamicgraph::Vector &x_dx_ddx = m_x_dx_ddxSINNER(iter);
        if(s.size()!=N_JOINTS)
          s.resize(N_JOINTS);
	s = x_dx_ddx.segment<N_JOINTS>(2*N_JOINTS);
        return s;
      }

      void NumericalDifference::display( std::ostream& os ) const
      {
        os << "NumericalDifference "<<getName()<<":\n";
        try
        {
          getProfiler().report_all(3, os);
        }
        catch (ExceptionSignal e) {}
      }

    } // namespace torque_control
  } // namespace sot
} // namespace dynamicgraph
