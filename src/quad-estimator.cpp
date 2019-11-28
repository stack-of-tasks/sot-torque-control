/*
   Oscar Efrain RAMOS PONCE, LAAS-CNRS
   Date: 28/10/2014
   Object to estimate a polynomial of second order that fits some data.
*/

#include <iostream>

#include <sot/torque_control/utils/quad-estimator.hh>

QuadEstimator::QuadEstimator(const unsigned int& N, const unsigned int& dim, const double& dt)
    : PolyEstimator(2, N, dt), dim_(dim), sum_ti_(0.0), sum_ti2_(0.0), sum_ti3_(0.0), sum_ti4_(0.0) {
  /* Number of coefficients for a quadratic estimator: 3 */
  coeff_.resize(3);

  /* Initialize sums for the recursive computation */
  sum_xi_.resize(dim);
  sum_tixi_.resize(dim);
  sum_ti2xi_.resize(dim);
  for (unsigned int i = 0; i < dim; ++i) {
    sum_xi_.at(i) = 0.0;
    sum_tixi_.at(i) = 0.0;
    sum_ti2xi_.at(i) = 0.0;
  }

  /* Initialize (pre-compute) pseudo inverse matrix that assumes that the sample
     time is constant (only if dt is not zero) */
  if (!dt_zero_) {
    Eigen::MatrixXd Tmat(N_, 3);
    Eigen::MatrixXd pinvTmat(3, N_);
    double time = 0.0;
    for (unsigned int i = 0; i < N_; ++i) {
      Tmat(i, 2) = 0.5 * time * time;
      Tmat(i, 1) = time;
      Tmat(i, 0) = 1.0;
      time += dt_;
    }
    /* Half time used to estimate velocity and position*/
    tmed_ = time * 0.5;

    /* Pseudo inverse */
    pinv(Tmat, pinvTmat);
    pinv0_ = new double[N_];
    pinv1_ = new double[N_];
    pinv2_ = new double[N_];
    c0_.resize(dim_);
    c1_.resize(dim_);
    c2_.resize(dim_);
    c0_.assign(dim_, 0.0);
    c1_.assign(dim_, 0.0);
    c2_.assign(dim_, 0.0);

    /* Copy the pseudo inverse to an array */
    for (unsigned int i = 0; i < N_; ++i) {
      pinv2_[i] = pinvTmat(2, i);
      pinv1_[i] = pinvTmat(1, i);
      pinv0_[i] = pinvTmat(0, i);
    }
  }
}

double QuadEstimator::getEsteeme() { return coeff_(2); }

void QuadEstimator::estimateRecursive(std::vector<double>& esteem, const std::vector<double>& el, const double& time) {
  /* Feed Data */
  elem_list_.at(pt_) = el;
  time_list_.at(pt_) = time;

  double t, x;

  if (first_run_) {
    // Not enough elements in the window
    if ((pt_ + 1) < N_) {
      // t = time - time_list_.at(0);
      t = time;
      sum_ti_ += t;
      sum_ti2_ += t * t;
      sum_ti3_ += t * t * t;
      sum_ti4_ += t * t * t * t;

      for (unsigned int i = 0; i < esteem.size(); ++i) {
        // Return a zero vector
        esteem.at(i) = 0.0;
        x = el.at(i);
        // Fill the sumations
        sum_xi_.at(i) += x;
        sum_tixi_.at(i) += t * x;
        sum_ti2xi_.at(i) += t * t * x;
      }
      pt_++;
      return;
    } else {
      first_run_ = false;
    }
  }

  // Increase the 'circular' pointer
  pt_ = (pt_ + 1) < N_ ? (pt_ + 1) : 0;
  // The pointer now points to the "oldest" element in the vector

  // Get size of first element: N dof (assuming all elements have same size)
  size_t dim = elem_list_.at(0).size();
  // TODO: CHECK THAT SIZE OF ESTEEM = DIM

  // t = time - time_list_.at(pt_);
  t = time;
  sum_ti_ += t;
  sum_ti2_ += t * t;
  sum_ti3_ += t * t * t;
  sum_ti4_ += t * t * t * t;

  double den = 0.25 * N_ * sum_ti3_ * sum_ti3_ - 0.5 * sum_ti3_ * sum_ti2_ * sum_ti_ +
               0.25 * sum_ti2_ * sum_ti2_ * sum_ti2_ + 0.25 * sum_ti4_ * sum_ti_ * sum_ti_ -
               0.25 * N_ * sum_ti4_ * sum_ti2_;
  double den2 = 1.0 / (2.0 * den);
  // double den4 = 1.0/(4.0*den);

  double t_old = time_list_.at(pt_);
  double a;
  // double b;
  // unsigned int pt_med;
  for (unsigned int i = 0; i < dim; ++i) {
    x = el[i];
    // Fill the sumations
    sum_xi_[i] += x;
    sum_tixi_[i] += t * x;
    sum_ti2xi_[i] += t * t * x;

    a = den2 *
        (sum_ti2xi_[i] * (sum_ti_ * sum_ti_ - N_ * sum_ti2_) + sum_tixi_[i] * (N_ * sum_ti3_ - sum_ti2_ * sum_ti_) +
         sum_xi_[i] * (sum_ti2_ * sum_ti2_ - sum_ti3_ * sum_ti_));
    // b = den4*( sum_ti2xi_[i]*(N_*sum_ti3_-sum_ti2_*sum_ti_) +
    //            sum_tixi_[i]*(sum_ti2_*sum_ti2_-N_*sum_ti4_) +
    //            sum_xi_[i]*(sum_ti4_*sum_ti_-sum_ti3_*sum_ti2_) );

    esteem[i] = a;  // For acceleration

    // pt_med = (pt_+((N_-1)/2)) % N_;
    // esteem[i] = a*time_list_[pt_med] + b;  // For velocity

    x = elem_list_[pt_][i];
    sum_xi_[i] -= x;
    sum_tixi_[i] -= t_old * x;
    sum_ti2xi_[i] -= t_old * t_old * x;
  }
  sum_ti_ -= t_old;
  sum_ti2_ -= t_old * t_old;
  sum_ti3_ -= t_old * t_old * t_old;
  sum_ti4_ -= t_old * t_old * t_old * t_old;

  return;
}

void QuadEstimator::fit() {
  double sum_ti = 0.0;
  double sum_ti2 = 0.0;
  double sum_ti3 = 0.0;
  double sum_ti4 = 0.0;
  double sum_xi = 0.0;
  double sum_tixi = 0.0;
  double sum_ti2xi = 0.0;

  for (unsigned int i = 0; i < N_; ++i) {
    sum_ti += t_[i];
    sum_ti2 += t_[i] * t_[i];
    sum_ti3 += t_[i] * t_[i] * t_[i];
    sum_ti4 += t_[i] * t_[i] * t_[i] * t_[i];
    sum_xi += x_[i];
    sum_tixi += t_[i] * x_[i];
    sum_ti2xi += t_[i] * t_[i] * x_[i];
  }

  double den = 0.25 * N_ * sum_ti3 * sum_ti3 - 0.5 * sum_ti3 * sum_ti2 * sum_ti + 0.25 * sum_ti2 * sum_ti2 * sum_ti2 +
               0.25 * sum_ti4 * sum_ti * sum_ti - 0.25 * N_ * sum_ti4 * sum_ti2;
  double den4 = 1.0 / (4.0 * den);

  coeff_(2) = den4 * (sum_ti2xi * (sum_ti * sum_ti - N_ * sum_ti2) + sum_tixi * (N_ * sum_ti3 - sum_ti2 * sum_ti) +
                      sum_xi * (sum_ti2 * sum_ti2 - sum_ti3 * sum_ti));
  coeff_(1) = den4 * (sum_ti2xi * (N_ * sum_ti3 - sum_ti2 * sum_ti) + sum_tixi * (sum_ti2 * sum_ti2 - N_ * sum_ti4) +
                      sum_xi * (sum_ti4 * sum_ti - sum_ti3 * sum_ti2));
  // This has not been computed (because not needed for accel or velocity)
  coeff_(0) = 0;

  return;
}

void QuadEstimator::estimate(std::vector<double>& esteem, const std::vector<double>& el) {
  if (dt_zero_) {
    std::cerr << "Error: dt cannot be zero" << std::endl;
    // Return a zero vector
    for (unsigned int i = 0; i < esteem.size(); ++i) esteem[i] = 0.0;
    return;
  }

  /* Feed Data. Note that the time is not completed since it is assumed to be
     constant */
  elem_list_[pt_] = el;

  if (first_run_) {
    if ((pt_ + 1) < N_) {
      // Return input vector when not enough elements to compute
      pt_++;
      for (unsigned int i = 0; i < esteem.size(); ++i) esteem[i] = el[i];
      return;
    } else
      first_run_ = false;
  }

  // Next pointer value
  pt_ = (pt_ + 1) < N_ ? (pt_ + 1) : 0;

  unsigned int idx;

  double x;
  // Cycle all the elements in the vector
  for (int i = 0; i < dim_; ++i) {
    c0_[i] = 0.0;
    c1_[i] = 0.0;
    c2_[i] = 0.0;
    // Retrieve the data in the window
    for (unsigned int j = 0; j < N_; ++j) {
      idx = (pt_ + j);
      if (idx >= N_) idx -= N_;
      x = elem_list_[idx][i];
      c0_[i] += x * pinv0_[j];
      c1_[i] += x * pinv1_[j];
      c2_[i] += x * pinv2_[j];
    }

    // Polynomial (position)
    esteem[i] = 0.5 * c2_[i] * tmed_ * tmed_ + c1_[i] * tmed_ + c0_[i];
  }
}

void QuadEstimator::getEstimateDerivative(std::vector<double>& estimateDerivative, const unsigned int order) {
  switch (order) {
    case 0:
      for (int i = 0; i < dim_; ++i) estimateDerivative[i] = 0.5 * c2_[i] * tmed_ * tmed_ + c1_[i] * tmed_ + c0_[i];
      return;

    case 1:
      for (int i = 0; i < dim_; ++i) estimateDerivative[i] = c2_[i] * tmed_ + c1_[i];
      return;

    case 2:
      for (int i = 0; i < dim_; ++i) estimateDerivative[i] = c2_[i];
      return;

    default:
      for (int i = 0; i < dim_; ++i) estimateDerivative[i] = 0.0;
  }
}
