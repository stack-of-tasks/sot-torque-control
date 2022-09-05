/*
   Oscar Efrain RAMOS PONCE, LAAS-CNRS
   Date: 28/10/2014
   Object to estimate a polynomial that fits some data.
*/

#include <sot/torque_control/utils/poly-estimator.hh>

void pinv(const Eigen::MatrixXd& matrix_in, Eigen::MatrixXd& pseudo_inv,
          const double& pinvtoler) {
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      matrix_in, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::VectorXd singular_values;
  Eigen::VectorXd singular_values_inv;
  singular_values = svd.singularValues();
  singular_values_inv.setZero(singular_values.size());

  for (int w = 0; w < singular_values.size(); ++w)
    if (singular_values(w) > pinvtoler)
      singular_values_inv(w) = 1 / singular_values(w);
  pseudo_inv = svd.matrixV() * singular_values_inv.asDiagonal() *
               svd.matrixU().transpose();
  return;
}

PolyEstimator::PolyEstimator(const unsigned int& order, const unsigned int& N,
                             const double& dt)
    : order_(order), N_(N), dt_(dt), dt_zero_(true), first_run_(true), pt_(0) {
  t_.resize(N_);
  x_.resize(N_);
  elem_list_.reserve(N_);
  elem_list_.resize(N_);
  time_list_.reserve(N_);
  time_list_.resize(N_);
  // Determine if the dt is valid
  if (dt_ > 1e-10) dt_zero_ = false;
  // Used only in the generic fit function
  R_.resize(N_, order_ + 1);
}

void PolyEstimator::estimate(std::vector<double>& esteem,
                             const std::vector<double>& el,
                             const double& time) {
  /* Feed Data */
  elem_list_.at(pt_) = el;
  time_list_.at(pt_) = time;

  if (first_run_) {
    if ((pt_ + 1) < N_) {
      // Return input vector when not enough elements to compute
      pt_++;
      for (unsigned int i = 0; i < esteem.size(); ++i) esteem.at(i) = el[i];
      return;
    } else {
      first_run_ = false;
    }
  }

  // Next pointer value
  pt_ = (pt_ + 1) < N_ ? (pt_ + 1) : 0;

  // Get the time substracting the lowest one, so that the minimum time is
  // always zero
  unsigned int idx;
  for (unsigned int j = 0; j < N_; ++j) {
    idx = pt_ + j;
    if (idx >= N_) idx = idx - N_;
    t_[j] = time_list_[idx] - time_list_[pt_];
  }

  // Get size of first element: N dof (assuming all elements have same size)
  size_t dim = elem_list_.at(0).size();
  // TODO: CHECK THAT SIZE OF ESTEEM = DIM

  // Cycle upon all elements
  for (unsigned int i = 0; i < dim; ++i) {
    // Retrieve the data vector
    for (unsigned int j = 0; j < N_; ++j) {
      idx = pt_ + j;
      if (idx >= N_) idx = idx - N_;
      x_[j] = elem_list_[idx][i];
    }
    // Fit the vector
    fit();
    esteem[i] = getEsteeme();
  }

  return;
}

void PolyEstimator::fit() {
  for (unsigned int i = 0; i < N_; ++i) {
    double xtemp = t_[i];
    R_(i, 0) = 1.0;

    for (unsigned int j = 1; j <= order_; ++j) {
      R_(i, j) = xtemp;
      xtemp *= xtemp;
    }
  }
  Eigen::Map<Eigen::VectorXd> ytemp(&x_[0], N_, 1);
  coeff_ = R_.householderQr().solve(ytemp);

  return;
}

void PolyEstimator::setWindowLength(const unsigned int& N) { N_ = N; }

unsigned int PolyEstimator::getWindowLength() { return N_; }
