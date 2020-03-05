// standard lib includes
#include <cstring>
#include <map>
#include <vector>
// ros includes
#include <ros/ros.h>
// custom includes
#include <hyperion_msgs/TrackedPerson.h>
#include <hyperion_predict/core.h>
#include <hyperion_predict/kalman_filter.h>

using namespace std;

/**
 * CORE module for prediction of movement
 * based on kalman filter prediction
 */

Core::Core(float *dt) {
  this->dt_ = dt;

  // initialize all the ROSey stuff here
  this->pub = this->nh.advertise<hyperion_msgs::TrackedPersons>(
      "/hyperion/prediction", 1000);
  this->sub = this->nh.subscribe("/detection/pos_array", 1000,
                                 &Core::measurement, this);
}

void Core::queue() {
  // queue data from the image detection node
}

void Core::predict() {
  // run prediction on the detection data
  Vector<Eigen::Vector4d> result;
  int i = 0;
  for (auto& [key, value]: this->tracks_) {
    Eigen::Vector4d X = value;
    float time = 1000; // how far into the future this should loop;
    HyperionPredict::KalmanFilter kf = KalmanFilter();
    result[i] = kf.filter(X, *this->dt_, time);
    i++;
  }
}

void Core::eval() {
  // evaluate the predictions
}

void Core::measurement(const hyperion_msgs::TrackedPersons::ConstPtr &data) {
  // add in measurement data
  vector<hyperion_msgs::TrackedPerson> persons(data->tracks);
  for (auto i = persons.begin(); i != persons.end(); ++i) {
    hyperion_msgs::TrackedPerson person = *i;
    int id = person.track_id;
    // vector<double> val(
    //{person.twist.twist.linear.x, person.twist.twist.linear.y,
    // person.twist.twist.linear.z});
    Eigen::Vector4d v(person.pose.pose.position.x,person.pose.pose.position.y,person.twist.twist.linear.x,person.twist.twist.linear.y);
    //Eigen::Matrix3d m(3, 3);
    //m << person.pose.pose.position.x, person.pose.pose.position.y,
        //person.pose.pose.position.z, person.twist.twist.linear.x,
        //person.twist.twist.linear.y, person.twist.twist.linear.z,
        //person.twist.twist.angular.x, person.twist.twist.angular.y,
        //person.twist.twist.angular.z;
    // this may need to be much much more complex.
    this->tracks_[id] = v;
  }
}
