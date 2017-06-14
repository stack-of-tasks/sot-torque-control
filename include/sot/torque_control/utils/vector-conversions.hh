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

namespace Eigen
{
  #define EIGEN_MAKE_TYPEDEFS(Type, TypeSuffix, Size, SizeSuffix)   \
  /** \ingroup matrixtypedefs */                                    \
  typedef Matrix<Type, Size, Size> Matrix##SizeSuffix##TypeSuffix;  \
  /** \ingroup matrixtypedefs */                                    \
  typedef Matrix<Type, Size, 1>    Vector##SizeSuffix##TypeSuffix;  \
  /** \ingroup matrixtypedefs */                                    \
  typedef Matrix<Type, 1, Size>    RowVector##SizeSuffix##TypeSuffix;

  #define EIGEN_MAKE_FIXED_TYPEDEFS(Type, TypeSuffix, Size)         \
  /** \ingroup matrixtypedefs */                                    \
  typedef Matrix<Type, Size, Dynamic> Matrix##Size##X##TypeSuffix;  \
  /** \ingroup matrixtypedefs */                                    \
  typedef Matrix<Type, Dynamic, Size> Matrix##X##Size##TypeSuffix;

  #define EIGEN_MAKE_TYPEDEFS_ALL_SIZES(Type, TypeSuffix) \
  EIGEN_MAKE_TYPEDEFS(Type, TypeSuffix, 1, 1) \
  EIGEN_MAKE_TYPEDEFS(Type, TypeSuffix, 5, 5) \
  EIGEN_MAKE_TYPEDEFS(Type, TypeSuffix, 6, 6) \
  EIGEN_MAKE_TYPEDEFS(Type, TypeSuffix, 7, 7) \
  EIGEN_MAKE_FIXED_TYPEDEFS(Type, TypeSuffix, 1) \
  EIGEN_MAKE_FIXED_TYPEDEFS(Type, TypeSuffix, 5) \
  EIGEN_MAKE_FIXED_TYPEDEFS(Type, TypeSuffix, 6) \
  EIGEN_MAKE_FIXED_TYPEDEFS(Type, TypeSuffix, 7)

  EIGEN_MAKE_TYPEDEFS_ALL_SIZES(int,                  i)
  EIGEN_MAKE_TYPEDEFS_ALL_SIZES(float,                f)
  EIGEN_MAKE_TYPEDEFS_ALL_SIZES(double,               d)
  EIGEN_MAKE_TYPEDEFS_ALL_SIZES(std::complex<float>,  cf)
  EIGEN_MAKE_TYPEDEFS_ALL_SIZES(std::complex<double>, cd)

  #undef EIGEN_MAKE_TYPEDEFS_ALL_SIZES
  #undef EIGEN_MAKE_TYPEDEFS

  typedef Matrix<double,Dynamic,Dynamic,RowMajor>   MatrixRXd;
  typedef Map<MatrixRXd>                            SigMatrixXd;
  typedef Map<VectorXd>                             SigVectorXd;
  typedef const Map<const MatrixRXd>                const_SigMatrixXd;
  typedef const Map<const VectorXd>                 const_SigVectorXd;

  typedef Eigen::Ref<Eigen::VectorXd>              RefVector;
  typedef const Eigen::Ref<const Eigen::VectorXd>& ConstRefVector;
  typedef Eigen::Ref<Eigen::MatrixXd>              RefMatrix;
  typedef const Eigen::Ref<const Eigen::MatrixXd>  ConstRefMatrix;
}

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

//Use ## to combine macro parameters into a single token (token => variable name, etc)
//And # to stringify a macro parameter (very useful when doing "reflection" in C/C++)

///  METAPOD
#define METAPOD_FORCE_FROM_SIGNAL(force, signal) \
  const Eigen::VectorXd& tmp##force##_eig = signal;	\
  metapod::Spatial::ForceTpl<double> force(tmp##force##_eig.tail<3>(), tmp##force##_eig.head<3>())

#endif // __sot_torque_control_vector_conversion_H__
