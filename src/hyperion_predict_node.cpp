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
   * main function. init and connect withtthe ROS  system
   *
   */

  ros::init(argc, argv, "hyperion_predict");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);

  double current_t = ros::Time::now().toSec();
  double last_t = ros::Time::now().toSec();
  float delta_t = 0;
  float clock = 0;
  float interval = 0;

  Core core = Core(&delta_t, &nh);

  ROS_INFO("RUNNING");
  while (ros::ok() && clock < 9) {
    last_t = current_t;
    current_t = ros::Time::now().toSec();
    delta_t = current_t - last_t;
    ROS_INFO("clock time: %f", clock);
    try {
      // run the interaction stuff here
      // thread pthread(&Core::predict, &core);
      // pthread.join();
      // core.queue();
      if (clock >= interval) {
        core.predict(clock);
        core.save(clock);
        interval += 1.0;
      }
    } catch (int e) {
      ROS_ERROR("Sorry you broke it");
      exit(-1);
    }
    clock += delta_t;
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
