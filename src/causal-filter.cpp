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

#include <iostream>

#include <sot/torque_control/utils/causal-filter.hh>



/*
Filter data with an IIR or FIR filter.

Filter a data sequence, x, using a digital filter. The filter is a direct form II transposed implementation of the standard difference equation.
This means that the filter implements:

a[0]*y[N] = b[0]*x[N] + b[1]*x[N-1] + ... + b[m-1]*x[N-(m-1)]
                      - a[1]*y[N-1] - ... - a[n-1]*y[N-(n-1)]

where m is the degree of the numerator, n is the degree of the denominator, and N is the sample number

*/

CausalFilter::CausalFilter(const double &timestep,
                           const int& xSize,
                           const Eigen::VectorXd& filter_numerator,
                           const Eigen::VectorXd& filter_denominator)
  
  : m_dt(timestep)
  , m_x_size(xSize)
  , m_filter_order_m(filter_numerator.size())
  , m_filter_order_n(filter_denominator.size())
  , m_filter_numerator(filter_numerator)
  , m_filter_denominator(filter_denominator)
  , pt_numerator(0)
  , pt_denominator(0)
  , input_buffer(Eigen::MatrixXd::Zero(xSize, filter_numerator.size()))
  , output_buffer(Eigen::MatrixXd::Zero(xSize, filter_denominator.size()-1))
{
  assert(timestep>0.0 && "Timestep should be > 0");
  assert(m_filter_numerator.size() == m_filter_order_m);
  assert(m_filter_denominator.size() == m_filter_order_n);
}


void CausalFilter::get_x_dx(const Eigen::VectorXd& base_x,
                            Eigen::VectorXd& x_output_dx)
{
  //const dynamicgraph::Vector &base_x = m_xSIN(iter);
  input_buffer.col(pt_numerator) = base_x;
  
  Eigen::VectorXd b(m_filter_order_m);
  Eigen::VectorXd a(m_filter_order_n-1);
  b.head(pt_numerator+1) = m_filter_numerator.head(pt_numerator+1).reverse();
  b.tail(m_filter_order_m-pt_numerator-1) =
    m_filter_numerator.tail(m_filter_order_m-pt_numerator-1).reverse();

  a.head(pt_denominator+1) = m_filter_denominator.segment(1, pt_denominator+1).reverse();
  a.tail(m_filter_order_n-pt_denominator-2) =
    m_filter_denominator.tail(m_filter_order_n-pt_denominator-2).reverse();
  x_output_dx.head(m_x_size) = (input_buffer*b-output_buffer*a)/m_filter_denominator[0];

  //Finite Difference
  x_output_dx.tail(m_x_size) = (x_output_dx.head(m_x_size)-output_buffer.col(pt_denominator))/m_dt;
  pt_numerator = (pt_numerator+1) < m_filter_order_m ? (pt_numerator+1) : 0;
  pt_denominator = (pt_denominator+1) < m_filter_order_n-1 ? (pt_denominator+1) : 0;
  output_buffer.col(pt_denominator) = x_output_dx.head(m_x_size);
  return;
}



void CausalFilter::switch_filter(const Eigen::VectorXd& filter_numerator,
                                 const Eigen::VectorXd& filter_denominator)
{
  int filter_order_m = filter_numerator.size();
  int filter_order_n = filter_denominator.size();
  if(filter_order_m > m_filter_order_m)
  {
    input_buffer.conservativeResize(Eigen::NoChange, filter_order_m);
    input_buffer.rightCols(filter_order_m - m_filter_order_m).setZero();
    Eigen::MatrixXd _tmp(m_x_size, m_filter_order_m - pt_numerator -1);
    _tmp = input_buffer.block(0, pt_numerator+1, m_x_size, m_filter_order_m - pt_numerator -1);
    input_buffer.rightCols(m_filter_order_m - pt_numerator -1) = _tmp;
    
    m_filter_numerator.resize(filter_order_m);
    m_filter_order_m = filter_order_m;
    m_filter_numerator = filter_numerator;
  }
  else if (filter_order_m == m_filter_order_m)
  {
    m_filter_numerator = filter_numerator;          
  }
  else
  {
    if (filter_order_m < pt_numerator+1)
    {
      Eigen::MatrixXd _tmp(input_buffer.block(0,pt_numerator-filter_order_m+1,
                                              m_x_size, filter_order_m));
      input_buffer.conservativeResize(Eigen::NoChange, filter_order_m);
      input_buffer = _tmp;
      pt_numerator = filter_order_m-1;
    }
    else if (filter_order_m == pt_numerator+1)
    {
      input_buffer.conservativeResize(Eigen::NoChange, filter_order_m);
    }
    else
    {
      Eigen::MatrixXd _tmp(input_buffer.rightCols(m_filter_order_m-pt_numerator-1));
      input_buffer.conservativeResize(Eigen::NoChange, filter_order_m);
      input_buffer.rightCols(m_filter_order_m-pt_numerator-1) = _tmp;
    }
    m_filter_order_m = filter_order_m;
    m_filter_numerator.resize(filter_order_m);
    m_filter_numerator = filter_numerator;
  }
  
  if(filter_order_n > m_filter_order_n)
  {
    output_buffer.conservativeResize(Eigen::NoChange, filter_order_n-1);
    output_buffer.rightCols(filter_order_n - m_filter_order_n).setZero();
    Eigen::MatrixXd _tmp(m_x_size, m_filter_order_n - pt_denominator -2);
    _tmp = 
      output_buffer.block(0, pt_denominator+1, m_x_size, m_filter_order_n - pt_denominator -2);
    output_buffer.rightCols(m_filter_order_n - pt_denominator -2) = _tmp;
    
    m_filter_denominator.resize(filter_order_n);
    m_filter_order_n = filter_order_n;
    m_filter_denominator = filter_denominator;
  }
  else if (filter_order_n == m_filter_order_n)
  {
    m_filter_denominator = filter_denominator;          
  }
  else
  {
    if (filter_order_n < pt_denominator+2)
    {
      Eigen::MatrixXd _tmp(output_buffer.block(0,pt_denominator-filter_order_n+2,
                                               m_x_size, filter_order_n-1));
      output_buffer.conservativeResize(Eigen::NoChange, filter_order_n-1);
      output_buffer = _tmp;
      pt_denominator = filter_order_n-2;
    }
    else if (filter_order_n == pt_denominator+2)
    {
      output_buffer.conservativeResize(Eigen::NoChange, filter_order_n-1);
    }
    else
    {
      Eigen::MatrixXd _tmp(output_buffer.rightCols(m_filter_order_n-pt_denominator-2));
      output_buffer.conservativeResize(Eigen::NoChange, filter_order_n-1);
      output_buffer.rightCols(m_filter_order_n-pt_denominator-2) = _tmp;
    }
    m_filter_order_n = filter_order_n;
    m_filter_denominator.resize(filter_order_n);
    m_filter_denominator = filter_denominator;
  }
  return;
}
