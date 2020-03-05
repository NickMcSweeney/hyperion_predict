#include <Eigen/Dense>

class KalmanFilter {
private:
	Eigen::Vector4d data_; // return value
  Eigen::Matrix4d A_;
  Eigen::Matrix4d B_;
  Eigen::Matrix4d P_;
  Eigen::Matrix4d Q_;
  Eigen::Matrix4d R_;
  Eigen::Matrix4d H_;
  Eigen::Matrix4d K_; // kalman gain
  Eigen::Vector4d z_; // measurement
  Eigen::Vector4d u_; // control
  Eigen::Vector4d x_; // input

  Eigen::Vector4d x_pred;

  double t_;

public:
	KalmanFilter();

private:
	void predict();
	void update();

	void filter(Eigen::Vector4d measurement, double dt, float time);
};
