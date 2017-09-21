/*
 * Copyright 2017-, Rohan Budhirja, LAAS-CNRS
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



/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */
#include <Eigen/Core>

class CausalFilter
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  /** --- CONSTRUCTOR ---- */
  CausalFilter(const double &timestep,
               const int& xSize,
               const Eigen::VectorXd& filter_numerator,
               const Eigen::VectorXd& filter_denominator);
  
  void get_x_dx(const Eigen::VectorXd& base_x,
                Eigen::VectorXd& x_output_dx);
  
  void switch_filter(const Eigen::VectorXd& filter_numerator,
                     const Eigen::VectorXd& filter_denominator);
  
private:
  double m_dt;      /// sampling timestep of the input signal
  int m_x_size;
  int m_filter_order_m;
  int m_filter_order_n;
  
  Eigen::VectorXd m_filter_numerator;
  Eigen::VectorXd m_filter_denominator;
  int pt_numerator;
  int pt_denominator;
  Eigen::MatrixXd input_buffer;
  Eigen::MatrixXd output_buffer;
}; // class CausalFilter
