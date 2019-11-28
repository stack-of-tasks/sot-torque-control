/*
   Oscar Efrain RAMOS PONCE, LAAS-CNRS
   Date: 28/10/2014
   Object to estimate a polynomial of first order that fits some data.
*/

#ifndef _QUAD_ESTIMATOR_HH_
#define _QUAD_ESTIMATOR_HH_

#include <sot/torque_control/utils/poly-estimator.hh>

/**
 * Object to fit a quadratic polynomial to a given data. This can be used to
 * compute the accelerations given the positions.
 *
 */
class QuadEstimator : public PolyEstimator {
 public:
  /**
   * Create a quadratic estimator on a window of length N
   * @param N is the window length.
   * @param dim is the dimension of the input elements (number of dofs).
   * @param dt is the control (sampling) time.
   */
  QuadEstimator(const unsigned int& N, const unsigned int& dim, const double& dt = 0.0);

  virtual void estimate(std::vector<double>& estimee, const std::vector<double>& el);

  virtual void estimateRecursive(std::vector<double>& estimee, const std::vector<double>& el, const double& time);

  virtual void getEstimateDerivative(std::vector<double>& estimeeDerivative, const unsigned int order);

 private:
  virtual double getEsteeme();
  virtual void fit();

  int dim_;  // size of the data

  // Sums for the recursive computation
  double sum_ti_;
  double sum_ti2_;
  double sum_ti3_;
  double sum_ti4_;
  std::vector<double> sum_xi_;
  std::vector<double> sum_tixi_;
  std::vector<double> sum_ti2xi_;

  // coefficients of the polynomial c2*x^2 + c1*x + c0
  std::vector<double> c0_;
  std::vector<double> c1_;
  std::vector<double> c2_;

  // Rows of the pseudo-inverse (when assuming constant sample time)
  double* pinv0_;
  double* pinv1_;
  double* pinv2_;
  // Half of the maximum time (according to the size of the window and dt)
  double tmed_;
};

#endif
