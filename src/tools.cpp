#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // Check the validity of the inputs :
  //  1) The estimation vector size should not be zero
  //  2) The estimation vector size should equal ground truth vector size
  if (estimations.size() == 0) 
    cout << "CalculateRMSE() - Error : The estimation size is zero" << endl;
  else if (estimations.size() != ground_truth.size())
    cout << "CalculateRMSE() - Error : The estimation vector size is not equal to ground truth vector" << endl;

  VectorXd residual(4);
  VectorXd sum(4);
  sum << 0,0,0,0;

  // Accumulate squared residuals
  for (int i=0; i < estimations.size() ; ++i) {
    residual = estimations[i] - ground_truth[i];
    residual = residual.array()*residual.array();
    sum = sum + residual;
  }
  // Mean
  rmse = sum.array()/estimations.size();
  // Square root
  rmse = sqrt(rmse.array());

  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
    * Calculate a Jacobian for Extended KF.
  */
  MatrixXd Hj(3,4); // 3 for rho, phi, rho_dot
                    // 4 for px, py, vx, vy
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  float c1 = px*px + py*py;
  float c2 = sqrt(c1);
  float c3 = c1*c2; 

  // Check division by zero
  if (fabs(c1) < 0.0001) {
    cout << "CalculateJacobian() - Error : Division by Zero" << endl;
    return Hj;
  }
  
  // Calculate the Jacobian
  Hj <<                 px/c2 ,                 py/c2 ,     0 ,     0 ,
                     -(py/c1) ,                 px/c1 ,     0 ,     0 ,
        py*(vx*py - vy*px)/c3 , px*(px*vy - py*vx)/c3 , px/c2 , py/c2 ;

  return Hj;
}
