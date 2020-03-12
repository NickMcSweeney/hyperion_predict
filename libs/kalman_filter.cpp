#include <hyperion_predict/kalman_filter.h>

#include <vector>
#include <fstream>
#include <string>
#include <iostream>

using namespace std;

KalmanFilter::KalmanFilter(double dt) {
  this->t_ = dt;
  this->A_ = Eigen::Matrix4d::Identity();
  this->B_ = Eigen::Matrix4d::Zero();
  this->B_.topRightCorner(2, 2) = Eigen::MatrixXd::Identity(2, 2); //*this->t_;
  this->B_ = this->B_*(this->t_);
  this->P_ = Eigen::Matrix4d::Identity()*1000;
  this->Q_ = Eigen::Matrix4d::Identity()*10;
  this->H_ = Eigen::Matrix4d::Identity();
  this->R_ = Eigen::Matrix4d::Identity()*0.2;
}

Eigen::Vector4d KalmanFilter::filter(Eigen::Vector4d measurement, float time) {
	double elapsed = 0;
  this->z_ = measurement;
	while (elapsed < time) {
		this->predict();
		this->update();
		elapsed += this->t_;
	}
  return this->x_;
}

Eigen::Vector4d KalmanFilter::estimate(Eigen::Vector4d measurement, float time) {
	double elapsed = 0;

  this->x_ = measurement;
	while (elapsed < time) {
    this->u_ = this->x_;
		this->predict();
    this->x_ = this->x_pred;
		elapsed += this->t_;
	}

  std::stringstream ss;
  ss << "--- --- data --- ---" << endl << this->t_ << endl << elapsed << endl;
  ss << this->x_[0] << " " << this->x_[1] << " " << this->u_[2] << " " << this->u_[3] << " " << endl;
  ss << this->A_ << endl;
  ss << this->B_ << endl;
  string input = ss.str();
  ofstream out("kf_data");
  out << input;
  out.close();

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
