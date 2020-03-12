#include <Eigen/Dense>
#include <hyperion_msgs/TrackedPersons.h>
#include <hyperion_predict/utils.h>

class Core {
private:
	float *dt_;
	std::map<int, Eigen::Vector4d> tracks_;
	Utils util_;

	ros::NodeHandle nh;
	ros::Publisher pub;
	ros::Subscriber sub;

public:
	Core(float *dt, ros::NodeHandle *nh);

private:
	void measurement(const hyperion_msgs::TrackedPersons::ConstPtr &data);

public:
	void run();
	void predict(double timestamp);
	void save(double timestamp);
	void eval(std::string note);
};
