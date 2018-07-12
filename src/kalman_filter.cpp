#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
#define PI 3.141592653
using namespace std;
/*
Please note that the Eigen library does not initialize 
VectorXd or MatrixXd objects with zeros upon creation.
*/

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;  

}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  
  // do we need nu (noise?)
  x_ = F_ * x_; // don't have the noise vector, u

  //new prediction (incomplete on jupyter notes)
  P_ = F_ * P_ * (F_.transpose()) + Q_;

}

void KalmanFilter::Update(const VectorXd &z) {
    /**
    TODO:
    * update the state by using Kalman Filter equations
    */

    VectorXd y_ = (z - (H_ * x_));
    
    MatrixXd S_ = (H_ * P_ * (H_.transpose())) + R_;

    MatrixXd K_ = (P_ * (H_.transpose())  * (S_.inverse())) ;
    
    x_ = x_ + K_ * y_;
    
    MatrixXd I = MatrixXd::Identity(4,4);
    
    P_ = (I - K_ * H_ ) * P_;
    
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
    /* 
          MEASUREMENT SPACE TO STATE SPACE
    maps predicted space to measurement space 

    */

    MatrixXd h_x_prime = MatrixXd(3,1);
    
    float px = x_(0);
    float py = x_(1);
    float vx = x_(2);
    float vy = x_(3);
    
    double rho     = sqrt(px * px + py * py);
    // I forgot what the weird ass greek letter is...
    double thing   = atan2(py, px);
    double rho_dot =  ( px * vx + py * vy ) / rho ;

    h_x_prime << rho, thing, rho_dot;
    /* This is the end of what we know to do for this part */
    
    VectorXd y_ = (z - (h_x_prime));

    // Add or subtract until y_(0) is in the desired range
    /*
    if (y_(1) < -PI){
      while(y_(1) < -PI){
        y_(1) = y_(1) + 2 * PI;
      }
    }
    else if (y_(1) > PI){
      while(y_(1) > PI){
        y_(1) = y_(1) - 2 * PI;
      }
    }
    */
    y_(1) = std::fmod(y_(1), 2*PI);
/////////////////////////////////////////////////////
    
    
    MatrixXd S_ = (H_ * P_ * (H_.transpose())) + R_;
    
    MatrixXd K_ = (P_ * (H_.transpose())  * (S_.inverse())) ;

    // the new estimate
    x_ = x_ + (K_ * y_);
    MatrixXd I = MatrixXd::Identity(4,4);
    
    P_ = (I - K_ * H_ ) * P_;
    
}
