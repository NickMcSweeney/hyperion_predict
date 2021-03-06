#include <Eigen/Dense>

class KalmanFilter {
private:
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
  static const std::string PATH;

public:
  KalmanFilter(double dt);
  KalmanFilter();

private:
  void predict();
  void update();

public:
  Eigen::Vector4d filter(Eigen::Vector4d x, Eigen::Vector4d z, double dt);
  Eigen::Vector4d filter(Eigen::Vector4d x, double dt);
  Eigen::Vector4d estimate(Eigen::Vector4d measurement, float time);
  Eigen::Vector4d single_predict(Eigen::Vector4d measurement);
};
