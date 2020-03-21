/**
 * Hyperion RGB-d Vision integration with ROS
 * pos prediction module
 *
 */
#include <iostream>
#include <string>
#include <thread>

#include <ros/ros.h>

#include "hyperion_predict/core.h"

using namespace std;

int main(int argc, char **argv) {
  /**
   * main function. init and connect with the ROS  system
   *
   */

  ros::init(argc, argv, "hyperion_predict");
  ros::NodeHandle nh;
  ros::Rate loop_rate(100);
  ros::Rate long_loop_rate(10);

  double current_t = ros::Time::now().toSec();
  double last_t = ros::Time::now().toSec();
  float delta_t = 0;
  float clock = 0;
  float interval = 0;

  Core core = Core(&delta_t, &nh);

  ros::spinOnce();
  long_loop_rate.sleep();
  ROS_INFO("RUNNING");
  while (ros::ok() && clock < 30) {
    if (ros::Time::now().toSec() >= (current_t + 0.1)) {
      last_t = current_t;
      current_t = ros::Time::now().toSec();
      delta_t = current_t - last_t;
      ROS_INFO("clock time: %f", clock);
      try {
        // run the interaction stuff here
        core.run(clock);
        if (clock >= interval - 0.05) {
          core.predict(clock);
          core.save(clock);
          interval += 1.0;
        }
      } catch (const char *err) {
        ROS_ERROR("Sorry you broke it");
        ROS_ERROR("MSG: %s", err);
        exit(-1);
      }
      clock += delta_t;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  // evaluate prediction accuracy
  string test_note;
  cout << "enter any notes for this test: ";
  getline(cin, test_note);
  // might also want to get at title for the file?
  ROS_INFO("test: %s", test_note.c_str());
  core.eval(test_note);

  ROS_INFO("DONE");
  return 0;
}
