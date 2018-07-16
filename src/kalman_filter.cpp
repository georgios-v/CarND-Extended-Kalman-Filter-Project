#include "kalman_filter.h"

#define EPSILON 0.0001

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {
}

KalmanFilter::~KalmanFilter() {
}

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
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update_(const VectorXd &y) {
	/**
	TODO:
	 * update the state by using Kalman Filter equations
	 */
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}

void KalmanFilter::Update(const VectorXd &z) {
	/**
	TODO:
	 * update the state by using Kalman Filter equations
	 */
	VectorXd y = z - H_ * x_;
	Update_(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
	/**
	TODO:
	 * update the state by using Extended Kalman Filter equations
	 */
	float p_x = x_(0);
	float p_y = x_(1);
	float v_x = x_(2);
	float v_y = x_(3);
	float rho = sqrt(p_x * p_x + p_y * p_y);
	float phi = atan2(p_y, p_x);
	float rho_dot = 0.0;
	if (rho >= EPSILON)
		rho_dot = (p_x * v_x + p_y * v_y) / (rho);

	VectorXd h = VectorXd(3);
	h << rho, phi, rho_dot;

	VectorXd y = z - h;
	if (fabs(y[1]) > M_PI) {
		y[1] = atan2(sin(y[1]), cos(y[1]));
	}
	
	Update_(y);
}
