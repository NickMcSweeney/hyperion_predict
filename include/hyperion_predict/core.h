#include <Eigen/Dense>
#include <hyperion_msgs/TrackedPersons.h>
#include <hyperion_predict/kalman_filter.h>
#include <hyperion_predict/utils.h>

class Core {
private:
  float *dt_;
  std::vector<std::map<int, Eigen::Vector4d>> buffer_;
  std::map<int, Eigen::Vector4d> tracks_;
  std::map<int, Eigen::Vector4d> filtered_tracks_;
  Utils util_;
  KalmanFilter filter_;

  ros::NodeHandle nh;
  ros::Publisher pub;
  ros::Subscriber sub;

  static const std::string PATH;

public:
  Core(float *dt, ros::NodeHandle *nh);

private:
  void measurement(const hyperion_msgs::TrackedPersons::ConstPtr &data);

public:
  void run(double timestamp);
  void predict(double timestamp);
  void save(double timestamp);
  void eval(std::string note);
};
