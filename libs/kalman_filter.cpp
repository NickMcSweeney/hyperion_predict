#include <hyperion_predict/kalman_filter.h>

#include <vector>

using namespace std;

KalmanFilter::KalmanFilter() {
  this->A_ = Eigen::Matrix4d::Identity();
  this->B_ = Eigen::Matrix4d::Zero();
  this->B_.topRightCorner(2, 2) = Eigen::Matrix4d::Identity(2, 2)*this->t_;
  this->P_ = Eigen::Matrix4d::Identity()*1000;
  this->Q_ = Eigen::Matrix4d::Identity()*10;
  this->H_ = Eigen::Matrix4d::Identity();
  this->R_ = Eigen::Matrix4d::Identity()*0.2;
}

Eigen::Vector4d KalmanFilter::filter(Eigen::Vector4d measurement, double dt, float time) {
	double elapsed = 0;
  this->z_ = measurement;
	while (elapsed < time) {
		this->predict();
		this->update();
		elapsed += dt;
	}
  return this->x_;
}

void KalmanFilter::predict() {
	// estimate = transition matrix * previous state
	// covariance matrix = transition matrix * previous state
  this->x_pred = this->A_*this->x_ + this->B_*this->u_;
  this->P_ = this->A_*this->P_*this->A_ + this->Q_;
}

void KalmanFilter::update() {
	// kalman gain = covariance matrix * transpose(measurement matrix) * inverse(H*P*transpose(H) +
	//  measurement noise)
	// estimate = estimate + kalman gain
	// covariance matrix = I - kalman gain*measurement matrix*covariance matrix
  Eigen::Matrix4d obs = this->H_*this->P_*Eigen::Transpose<Eigen::Matrix4d>(this->H_)+this->R_;
  this->K_ = this->P_*Eigen::Transpose<Eigen::Matrix4d>(this->H_)*obs.inverse();
  this->x_ = this->x_pred + this->K_*(this->z_-this->H_*this->x_pred);
  this->P_ = (Eigen::Matrix4d::Identity() - this->K_*this->H_)*this->P_;
}
