/*
   Oscar Efrain RAMOS PONCE, LAAS-CNRS
   Date: 28/10/2014
   Object to estimate a polynomial that fits some data.
*/

#ifndef _POLY_ESTIMATOR_HH_
#define _POLY_ESTIMATOR_HH_

#include <Eigen/Dense>
#include <vector>

/**
 * Compute the pseudo-inverse using Eigen
 * @param [in] matrix_in is the input matrix.
 * @param [out] pseudo_inv is the pseudo-inverse of the input matrix.
 * @param [in] pinvtoler is the tolerance in the SVD decomposition
 */
void pinv(const Eigen::MatrixXd& matrix_in, Eigen::MatrixXd& pseudo_inv, const double& pinvtoler = 1.0e-6);

/**
 * Object to fit a polynomial of a given order. It provides a generic fitting
 * polynomial of any order. However, it cannot be used by itself since the
 * proper coefficient of the polynomial (which represents the estimation) needs
 * to be specified. Moreover, the derived classes implement a faster computation
 * based on the specific case.
 *
 */
class PolyEstimator {
 public:
  /**
   * Create a polynomial estimator on a window of length N
   * @param order is the order of the polynomial estimator.
   * @param N is the window length.
   * @param dt is the control (sampling) time
   */
  PolyEstimator(const unsigned int& order, const unsigned int& N, const double& dt);

  /**
   * Estimate the generic polynomial given a new element. The order of the
   * polynomial is specified in the constructor. Note that this function can be
   * slow if no specialization of fit() has been done, since the generic
   * algorithm would be used.
   * @param [out] estimee is the calculated estimation
   * @param [in] data_element is the new data vector.
   * @param [in] time is the time stamp corresponding to the new data.
   */
  void estimate(std::vector<double>& estimee, const std::vector<double>& data_element, const double& time);

  /**
   * Estimate the polynomial given a new element assuming a constant time
   * difference. This constant time difference between consecutive samples is
   * given by dt (specified in the constructor). <br>Note: This function will
   * only work if dt is different to zero.
   * @param [out] estimee is the calculated estimation.
   * @param [in] data_element is the new data vector.
   */
  virtual void estimate(std::vector<double>& estimee, const std::vector<double>& data_element) = 0;

  /**
   * Estimate the polynomial given a new element using a recursive
   * algorithm. This method is faster. However, it takes the time as it is (it
   * does not set the lowest time to zero), which can create "ill-conditions".
   * @param [out] estimee is the calculated estimation
   * @param [in] data_element is the new data.
   * @param [in] time is the time stamp corresponding to the new data.
   */
  virtual void estimateRecursive(std::vector<double>& estimee, const std::vector<double>& data_element,
                                 const double& time) = 0;

  /**
   * Get the time derivative of the estimated polynomial.
   * @param [out] estimeeDerivative is the calculated time derivative.
   * @param [in] order The order of the derivative (e.g. 1 means the first derivative).
   */
  virtual void getEstimateDerivative(std::vector<double>& estimeeDerivative, const unsigned int order) = 0;

  /**
   * Set the size of the filter window.
   * @param [in] N size
   */
  void setWindowLength(const unsigned int& N);

  /**
   * Get the size of the filter window.
   * @return Size
   */
  unsigned int getWindowLength();

 protected:
  /**
   * Find the regressor which best fits in least square sense the last N data
   * sample couples. The order of the regressor is given in the constructor.
   */
  virtual void fit();

  /**
   * Get the estimation when using the generic fit function (in poly-estimator)
   */
  virtual double getEsteeme() = 0;

  /// Order of the polynomial estimator
  unsigned int order_;

  /// Window length
  unsigned int N_;

  /// Sampling (control) time
  double dt_;

  /// Indicate that dt is zero (dt is invalid)
  bool dt_zero_;

  /// Indicate that there are not enough elements to compute. The reason is that
  /// it is one of the first runs, and the estimate will be zero.
  bool first_run_;

  /// All the data (N elements of size dim)
  std::vector<std::vector<double> > elem_list_;

  /// Time vector corresponding to each element in elem_list_
  std::vector<double> time_list_;

  /// Circular index to each data and time element
  unsigned int pt_;

  /// Coefficients for the least squares solution
  Eigen::VectorXd coeff_;

  /// Time vector setting the lowest time to zero (for numerical stability).
  std::vector<double> t_;

  /// Data vector for a single dimension (a single dof). It is only one 'column'
  /// of the elem_list_ 'matrix'
  std::vector<double> x_;

  /// Matrix containing time components. It is only used for the generic fit
  /// computation, such that the estimation is \f$c = R^{\#} x\f$, where \f$x\f$
  /// is the data vector.
  Eigen::MatrixXd R_;
};

#endif
