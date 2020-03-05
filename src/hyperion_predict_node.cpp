**
 * Hyperion RGB-d Vision integration with ROS
 * pos prediction module
 *
 */
#include <thread>

#include <ros/ros.h>

#include "hyperion_predict/core.h"

using namespace std;

int main(int argc, char **argv) {
	/**
	 * main function. init and connect withtthe ROS  system
	 *
	 */

	ros::init(argc, argv, "hyperion_predict");
	ros::Rate loop_rate(10);

	float current_t = ros::Time::now().toSec();
	float last_t = ros::Time::now().toSec();
	float delta_t = 0;

	Core core = Core(&delta_t);

	while (ros::ok()) {
		last_t = current_t;
		current_t = ros::Time::now().toSec();
		delta_t = current_t - last_t;
		try {
			// run the interaction stuff here
			// thread pthread(&Core::predict, &core);
			// pthread.join();
			//core.queue();
			core.predict();
			// evaluate prediction accuracy
			//core.eval();
		} catch (int e) {
			ROS_ERROR("Sorry you broke it");
			exit(-1);
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
