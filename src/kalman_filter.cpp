#include "kalman_filter.h"
#include <iostream>

using namespace std;

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

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
	x_ = F_ * x_;
	P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  //Update precess.
	VectorXd y = z - H_ * x_;
	MatrixXd S = H_ * P_ * H_.transpose() + R_;
	MatrixXd K = P_ * H_.transpose() * S.inverse();
	x_ = x_ + K * y;
	P_ = (I_ - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
	float rho = sqrt(x_(0) * x_(0) + x_(1) * x_(1));
	float ro_dot = (x_(0) * x_(2) + x_(1) * x_(3)) / rho;
	float phi;
	if (fabs(x_(1)) < 0.1) {
		phi = atan2(0, x_(0));
	}
	else
	{
		phi = atan2(x_(1), x_(0));
	}
	//For debugging.
	//cout << "rho = " << rho_esti << "  ";
	//cout << "phi = " << phi_esti << "  ";
	//cout << "ro_dot = " << ro_dot << "  ";
	//cout << "p,v = "<< ppx << "  " << vx << "  " << ppy << "  " << vy << endl;

	//Update precess.
	VectorXd h_dot = VectorXd(3);
	h_dot << rho, phi, ro_dot;
	VectorXd y = z - h_dot;
	MatrixXd S = H_ * P_ * H_.transpose() + R_;
	MatrixXd K = P_ * H_.transpose() * S.inverse();
	x_ = x_ + K * y;
	P_ = (I_ - K * H_) * P_;

}
