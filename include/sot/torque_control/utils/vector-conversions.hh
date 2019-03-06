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

#ifndef __sot_torque_control_vector_conversion_H__
#define __sot_torque_control_vector_conversion_H__

#include <Eigen/Dense>
#include <fstream>
#include <sstream>


#define EIGEN_CONST_VECTOR_FROM_STD_VECTOR(name,signal)	                 \
  Eigen::const_SigVectorXd name		               	                 \
  (						               	         \
   signal.data(),                                                        \
   signal.size()                                                         \
  )
#define EIGEN_VECTOR_FROM_STD_VECTOR(name,signal)	                 \
  Eigen::SigVectorXd name	 	               	                 \
  (						               	         \
   signal.data(),                                                        \
   signal.size()				               	         \
  )


/********************* VECTOR COPY ******************************/
// c arrays define only the [] operator
// mal vectors define only the () operator
// std vectors define only the [] operator
// Eigen vectors define both the [] and () operators

#define COPY_ARRAY_TO_ARRAY(src, dest, size) \
  for(unsigned int i=0; i<size; i++) \
    dest[i] = src[i]

#define COPY_ARRAY_TO_VECTOR(src, dest) \
  for(unsigned int i=0; i<dest.size(); i++) \
    dest(i) = src[i]

#define COPY_VECTOR_TO_ARRAY(src, dest) \
  for(unsigned int i=0; i<src.size(); i++) \
    dest[i] = src(i)

#define COPY_SHIFTED_VECTOR_TO_VECTOR(src, dest, offset) \
  for(unsigned int i=0; i< dest.size(); i++) \
    dest(i) = src(i+offset)

#define COPY_SHIFTED_VECTOR_TO_ARRAY(src, dest, offset) \
  for(unsigned int i=0; i< dest.size(); i++) \
    dest[i] = src(i+offset)

#define COPY_VECTOR_TO_SHIFTED_VECTOR(src, dest, offset) \
  for(unsigned int i=0; i< src.size(); i++) \
    dest(i+offset) = src(i)

#define COPY_ARRAY_TO_SHIFTED_VECTOR(src, dest, offset) \
  for(unsigned int i=0; i< src.size(); i++) \
    dest(i+offset) = src[i]

#endif // __sot_torque_control_vector_conversion_H__
