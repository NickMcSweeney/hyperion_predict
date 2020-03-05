#include <Eigen/Dense>
#include <hyperion_msgs/TrackedPersons.h>

class Core {
private:
	float *dt_;
	std::map<int, Eigen::Vector4d> tracks_;

	ros::NodeHandle nh;
	ros::Publisher pub;
	ros::Subscriber sub;

public:
	Core(float *dt);

private:
	void measurement(const hyperion_msgs::TrackedPersons::ConstPtr &data);

public:
	void queue();
	void predict();
	void eval();
};
