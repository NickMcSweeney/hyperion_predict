#include <hyperion_predict/kalman_filter.h>

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

using namespace std;

const string KalmanFilter::PATH =
    "/home/usernamix/Workspace/src/hyperion/hyperion_data/output/";

KalmanFilter::KalmanFilter(double dt) {
  this->t_ = dt;
  this->A_ = Eigen::Matrix4d::Identity();
  this->B_ = Eigen::Matrix4d::Zero();
  this->B_.topRightCorner(2, 2) = Eigen::MatrixXd::Identity(2, 2); //*this->t_;
  this->B_ = this->B_ * (this->t_);
  this->P_ = Eigen::Matrix4d::Identity() * 1000;
  this->Q_ = Eigen::Matrix4d::Identity() * 10;
  this->H_ = Eigen::Matrix4d::Identity();
  this->R_ = Eigen::Matrix4d::Identity() * 0.2;
}

KalmanFilter::KalmanFilter() {
  this->A_ = Eigen::Matrix4d::Identity();
  this->B_ = Eigen::Matrix4d::Zero();
  this->B_.topRightCorner(2, 2) = Eigen::MatrixXd::Identity(2, 2); //*this->t_;
  // this->B_ = this->B_*(this->t_);
  this->P_ = Eigen::Matrix4d::Identity() * 1000;
  this->Q_ = Eigen::Matrix4d::Identity() * 10;
  this->H_ = Eigen::Matrix4d::Identity();
  this->R_ = Eigen::Matrix4d::Identity() * 0.2;
}

Eigen::Vector4d KalmanFilter::filter(Eigen::Vector4d x, Eigen::Vector4d z,
                                     double dt) {
  this->t_ = dt;
  this->x_ = x;
  this->B_.topRightCorner(2, 2) = Eigen::MatrixXd::Identity(2, 2); //*this->t_;
  this->B_ = this->B_ * (this->t_);
  this->u_ = x;
  this->z_ = z;
  this->predict();
  this->update();
  return this->x_;
}

Eigen::Vector4d KalmanFilter::filter(Eigen::Vector4d x, double dt) {
  this->t_ = dt;
  this->B_.topRightCorner(2, 2) = Eigen::MatrixXd::Identity(2, 2); //*this->t_;
  this->B_ = this->B_ * (this->t_);
  this->u_ = x;
  this->x_ = x;
  this->predict();
  // this->update();
  return this->x_;
}

Eigen::Vector4d KalmanFilter::estimate(Eigen::Vector4d measurement,
                                       float time) {
  double elapsed = 0;

  this->x_ = measurement;
  while (elapsed < time) {
    this->u_ = this->x_;
    this->predict();
    this->x_ = this->x_pred;
    elapsed += this->t_;
  }

  return this->x_;
}

Eigen::Vector4d KalmanFilter::single_predict(Eigen::Vector4d measurement) {
  this->x_ = measurement;
  this->u_ = this->x_;
  this->predict();
  this->x_ = this->x_pred;
  return this->x_;
}

void KalmanFilter::predict() {
  // estimate = transition matrix * previous state
  // covariance matrix = transition matrix * previous state
  this->x_pred = this->A_ * this->x_ + this->B_ * this->u_;
  this->P_ = this->A_ * this->P_ * this->A_ + this->Q_;
}

void KalmanFilter::update() {
  // kalman gain = covariance matrix * transpose(measurement matrix) *
  // inverse(H*P*transpose(H) +
  //  measurement noise)
  // estimate = estimate + kalman gain
  // covariance matrix = I - kalman gain*measurement matrix*covariance matrix
  Eigen::Matrix4d obs =
      this->H_ * this->P_ * Eigen::Transpose<Eigen::Matrix4d>(this->H_) +
      this->R_;
  this->K_ =
      this->P_ * Eigen::Transpose<Eigen::Matrix4d>(this->H_) * obs.inverse();
  this->x_ = this->x_pred + this->K_ * (this->z_ - this->H_ * this->x_pred);
  this->P_ = (Eigen::Matrix4d::Identity() - this->K_ * this->H_) * this->P_;
}
