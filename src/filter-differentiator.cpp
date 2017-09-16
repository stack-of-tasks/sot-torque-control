/*
 * Copyright 2017-, Rohan Budhiraja LAAS-CNRS
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

#include <sot/torque_control/filter-differentiator.hh>
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

#define ALL_OUTPUT_SIGNALS  m_x_filteredSOUT << m_dxSOUT

      namespace dynamicgraph = ::dynamicgraph;
      using namespace dynamicgraph;
      using namespace dynamicgraph::command;
      using namespace Eigen;

      /// Define EntityClassName here rather than in the header file
      /// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
      typedef FilterDifferentiator EntityClassName;

      /* --- DG FACTORY ------------------------------------------------------- */
      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(FilterDifferentiator,"FilterDifferentiator");

      /* --- CONSTRUCTION ----------------------------------------------------- */
      /* --- CONSTRUCTION ----------------------------------------------------- */
      /* --- CONSTRUCTION ----------------------------------------------------- */
      FilterDifferentiator::
      FilterDifferentiator( const std::string & name )
        : Entity(name),
          CONSTRUCT_SIGNAL_IN(x,                dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_OUT(x_filtered,       dynamicgraph::Vector, m_x_dxSINNER)
        ,CONSTRUCT_SIGNAL_OUT(dx,               dynamicgraph::Vector, m_x_dxSINNER)
        ,CONSTRUCT_SIGNAL_INNER(x_dx,       dynamicgraph::Vector, m_xSIN)
      {
        Entity::signalRegistration( ALL_INPUT_SIGNALS << ALL_OUTPUT_SIGNALS);

        /* Commands. */
        addCommand("getTimestep",
                   makeDirectGetter(*this,&m_dt,
                                    docDirectGetter("Control timestep [s ]","double")));
        addCommand("getSize",
                   makeDirectGetter(*this,&m_x_size,
                                    docDirectGetter("Size of the x signal","int")));
        addCommand("init", makeCommandVoid6(*this, &FilterDifferentiator::init,
                              docCommandVoid6("Initialize the filter.",
                                              "Control timestep [s].",
                                              "Size of the input signal x",
                                              "Filter order Numerator",
                                              "Filter order Denominator",
                                              "Numerator of the filter",
                                              "Denominator of the filter")));
        addCommand("switch_filter",
                   makeCommandVoid4(*this, &FilterDifferentiator::switch_filter,
                                    docCommandVoid4("Switch Filter.",
                                                    "Filter order Numerator",
                                                    "Filter order Denominator",
                                                    "Numerator of the filter",
                                                    "Denominator of the filter")));

      }


      /* --- COMMANDS ---------------------------------------------------------- */
      /* --- COMMANDS ---------------------------------------------------------- */
      /* --- COMMANDS ---------------------------------------------------------- */
      void FilterDifferentiator::init(const double &timestep,
                               const int& xSize,
                               const int& filter_order_m,
                               const int& filter_order_n,
                               const Eigen::VectorXd& filter_numerator,
                               const Eigen::VectorXd& filter_denominator)
      {
        m_x_size = xSize;
        m_dt = timestep;
        m_filter = new CausalFilter(timestep, xSize,
                                    filter_order_m, filter_order_n,
                                    filter_numerator, filter_denominator);
        return;
      }

      void FilterDifferentiator::switch_filter(const int& filter_order_m,
                                        const int& filter_order_n,
                                        const Eigen::VectorXd& filter_numerator,
                                        const Eigen::VectorXd& filter_denominator)
      {
        m_filter->switch_filter(filter_order_m, filter_order_n,
                                filter_numerator, filter_denominator);
      }
      

      /* --- SIGNALS ---------------------------------------------------------- */
      /* --- SIGNALS ---------------------------------------------------------- */
      /* --- SIGNALS ---------------------------------------------------------- */

      /** Signal Filtering and Differentiation.
          a[0]*y[n] = b[0]*x[n] + b[1]*x[n-1] + ... + b[M]*x[n-M]
          - a[1]*y[n-1] - ... - a[N]*y[n-N]
          for the sample number n
      */
      DEFINE_SIGNAL_INNER_FUNCTION(x_dx, dynamicgraph::Vector)
      {
        sotDEBUG(15)<<"Compute x_dx inner signal "<<iter<<std::endl;
        if(s.size()!=2*m_x_size)
          s.resize(2*m_x_size);
        // read encoders
        const dynamicgraph::Vector& base_x = m_xSIN(iter);
        m_filter->get_x_dx(base_x, s);
        return s;
      }


      /// ************************************************************************* ///
      /// The following signals depend only on other inner signals, so they
      /// just need to copy the interested part of the inner signal they depend on.
      /// ************************************************************************* ///


      DEFINE_SIGNAL_OUT_FUNCTION(x_filtered, dynamicgraph::Vector)
      {
        sotDEBUG(15)<<"Compute x_filtered output signal "<<iter<<std::endl;

        const dynamicgraph::Vector &x_dx = m_x_dxSINNER(iter);
        if(s.size()!=m_x_size)
          s.resize(m_x_size);
	s = x_dx.head(m_x_size);
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(dx, dynamicgraph::Vector)
      {
        sotDEBUG(15)<<"Compute dx output signal "<<iter<<std::endl;

        const dynamicgraph::Vector &x_dx = m_x_dxSINNER(iter);
        if(s.size()!=m_x_size)
	  s.resize(m_x_size);
	s = x_dx.tail(m_x_size);
        return s;
      }

      void FilterDifferentiator::display( std::ostream& os ) const
      {
        os << "FilterDifferentiator "<<getName()<<":\n";
        try
        {
          getProfiler().report_all(3, os);
        }
        catch (ExceptionSignal e) {}
      }

    } // namespace torque_control
  } // namespace sot
} // namespace dynamicgraph
