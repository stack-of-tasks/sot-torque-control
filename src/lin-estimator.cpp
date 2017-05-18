/* 
   Oscar Efrain RAMOS PONCE, LAAS-CNRS
   Date: 28/10/2014
   Object to estimate a polynomial of first order that fits some data.
*/


#include <iostream>

#include <sot/torque_control/utils/lin-estimator.hh>



LinEstimator::LinEstimator(const unsigned int& N,
                           const unsigned int& dim,
                           const double& dt)
  : PolyEstimator(1, N, dt)
  , dim_(dim)
  , sum_ti_(0.0)
  , sum_ti2_(0.0)
{
  /* Number of coefficients for a quadratic estimator: 2 */
  coeff_.resize(2);

  /* Initialize sums for the recursive computation */
  sum_xi_.resize(dim);
  sum_tixi_.resize(dim);
  for (unsigned int i = 0; i < dim; ++i) {
    sum_xi_.at(i)    = 0.0;
    sum_tixi_.at(i)  = 0.0;
  }

  /* Initialize (pre-compute) pseudo inverse matrix that assumes that the sample
     time is constant (only if dt is not zero) */
  if (!dt_zero_)
  {
    Eigen::MatrixXd Tmat(N_, 2);
    Eigen::MatrixXd pinvTmat(2, N_);
    double time = 0.0;
    for (unsigned int i = 0; i < N_; ++i)
    {
      Tmat(i,1) = time;
      Tmat(i,0) = 1.0;
      time += dt_;
    }
    /* Half time used to estimate the position */
    tmed_ = time*0.5;

    /* Pseudo inverse */
    pinv(Tmat, pinvTmat);
    pinv0_ = new double[N_];
    pinv1_ = new double[N_];
    c0_.resize(dim_);
    c1_.resize(dim_);
    c0_.assign(dim_, 0.0);
    c1_.assign(dim_, 0.0);

    /* Copy the pseudo inverse to an array */
    for(unsigned int i = 0; i < N_; ++i)
    {
      pinv0_[i] = pinvTmat(0,i);
      pinv1_[i] = pinvTmat(1,i);
    }
  }

}


double LinEstimator::getEsteeme()
{ 
  return coeff_(1);
}



void LinEstimator::estimateRecursive(std::vector<double>& esteem,
                                     const std::vector<double>& el,
                                     const double& time)
{
  /* Feed Data */
  elem_list_.at(pt_) = el;
  time_list_.at(pt_) = time;

  double t, x;
  
  if ( first_run_ ) {
    // Not enough elements in the window
    if ( (pt_+1) < N_ )
    {
      //t = time - time_list_.at(0);
      t = time;
      sum_ti_   += t;
      sum_ti2_ += t*t;
      for (unsigned int i = 0; i < esteem.size(); ++i) {
        // Return a zero vector
        esteem.at(i) = 0.0;
        x = el.at(i);
        // Fill the sumations
        sum_xi_.at(i)  += x;
        sum_tixi_.at(i) += t*x;
      }
      pt_++;
      return;
    }
    else {
      first_run_ = false;
    }
  }
  
  // Increase the 'circular' pointer
  pt_ = (pt_+1) < N_ ? (pt_+1) : 0;
  // The pointer now points to the "oldest" element in the vector

  // Get size of first element: N dof (assuming all elements have same size)
  size_t dim = elem_list_.at(0).size();
  // TODO: CHECK THAT SIZE OF ESTEEM = DIM

  //t = time - time_list_.at(pt_);
  t = time;
  sum_ti_   += t;
  sum_ti2_ += t*t;
  double den = N_*sum_ti2_ - sum_ti_*sum_ti_;

  //double t_old = time_list_.at(pt_);
  double t_old = time_list_.at(pt_);
  for (unsigned int i = 0; i < dim; ++i)
  {
    x = el[i];
    // Fill the sumations
    sum_xi_[i]  += x;
    sum_tixi_[i] += t*x;

    // the bias
    //coeff_(0) = (sum_xi*sum_titi - sum_ti*sum_tixi) / den;
    // the linear coefficient
    esteem[i] = (N_*sum_tixi_[i] - sum_ti_*sum_xi_[i]) / den;

    x = elem_list_[pt_][i];
    sum_xi_[i]   -= x;
    sum_tixi_[i] -= t_old*x;
  }
  sum_ti_   -= t_old;
  sum_ti2_ -= t_old*t_old;
  
  return;
}


void LinEstimator::fit()
{
  double sum_ti   = 0.0;
  double sum_titi = 0.0;
  double sum_xi   = 0.0;
  double sum_tixi = 0.0;
  
  for (unsigned int i = 0; i < N_; ++i)
  {
    sum_ti   += t_[i];
    sum_titi += t_[i] * t_[i];
    sum_xi   += x_[i];
    sum_tixi += t_[i] * x_[i];
  }
  
  double den = N_*sum_titi - sum_ti*sum_ti;
  
  // the bias
  coeff_(0) = (sum_xi*sum_titi - sum_ti*sum_tixi) / den;
  
  // the linear coefficient
  coeff_(1) = (N_*sum_tixi - sum_ti*sum_xi) / den;
  
  return;
}


void LinEstimator::estimate(std::vector<double>& esteem,
                            const std::vector<double>& el)
{
  if (dt_zero_)
  {
    std::cerr << "Error: dt cannot be zero" << std::endl;
    // Return a zero vector
    for (unsigned int i = 0; i < esteem.size(); ++i)
      esteem[i] = 0.0;
    return;
  }
  
  /* Feed Data. Note that the time is not completed since it is assumed to be
     constant */
  elem_list_[pt_] = el;

  if ( first_run_ )
  {
    if ( (pt_+1) < N_ )
    {
      // Return input vector when not enough elements to compute
      pt_++;
      for (unsigned int i = 0; i < esteem.size(); ++i)
        esteem[i] = el[i];
      return;
    }
    else
      first_run_ = false;
  }

  // Next pointer value
  pt_ = (pt_+1) < N_ ? (pt_+1) : 0;

  unsigned int idx;
  double x;
  // Cycle all the elements in the vector
  for (int i = 0; i < dim_; ++i)
  {
    c0_[i]=0.0; c1_[i]=0.0;
    // Retrieve the data in the window
    for (unsigned int j = 0; j < N_; ++j)
    {
      idx = (pt_+j);
      if (idx >= N_ ) idx -= N_;
      x = elem_list_[idx][i];
      c0_[i] += x*pinv0_[j];
      c1_[i] += x*pinv1_[j];
    }

    // Polynomial (position)
    esteem[i] = c1_[i]*tmed_ + c0_[i];
  }
}

void LinEstimator::getEstimateDerivative(std::vector<double>& estimateDerivative,
                                   const unsigned int order)
{
  switch(order)
  {
    case 0:
      for (int i = 0; i < dim_; ++i)
        estimateDerivative[i] = c1_[i]*tmed_ + c0_[i];
      return;

    case 1:
      for (int i = 0; i < dim_; ++i)
        estimateDerivative[i] = c1_[i];
      return;

    default:
      for (int i = 0; i < dim_; ++i)
        estimateDerivative[i] = 0.0;
  }
}
