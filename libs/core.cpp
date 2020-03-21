// standard lib includes
#include <algorithm>
#include <fstream>
#include <iostream>
#include <map>
#include <math.h>
#include <numeric>
#include <string>
#include <unordered_set>
#include <vector>
// ros includes
#include <ros/ros.h>
// custom includes
#include <hyperion_msgs/TrackedPerson.h>
#include <hyperion_predict/core.h>

#include <math.h>

using namespace std;

/**
 * CORE module for prediction of movement
 * based on kalman filter prediction
 */

const string Core::PATH =
    "/home/usernamix/Workspace/src/hyperion/hyperion_data/output/";

Core::Core(float *dt, ros::NodeHandle *nodehandle) {
  this->dt_ = dt;
  this->nh = *nodehandle;

  // initialize all the ROSey stuff here
  this->pub = this->nh.advertise<hyperion_msgs::TrackedPersons>(
      "/hyperion/prediction", 1000);
  this->sub =
      this->nh.subscribe("hyperion/detections", 1000, &Core::measurement, this);

  ofstream out;
  out.open(PATH + "measure.log");
  out.close();
  out.open(PATH + "raw_data.dat");
  out.close();
  out.open(PATH + "filtered_data.dat");
  out.close();
  out.open(PATH + "data_analysis.dat");
  out.close();

  this->util_ = Utils();
  this->filter_ = KalmanFilter(*dt);
}

void Core::run(double timestamp) {
  map<int, Eigen::Vector4d> tracks;
  try {
    // convert this->buffer_ to TrackedPerson;
    if (this->buffer_.empty())
      throw "buffer empty";
    tracks = this->buffer_.back();
    this->buffer_.clear();
    this->tracks_ = tracks;
  } catch (const char *err) {
    cerr << err << endl;
    return;
  }
  map<int, Eigen::Vector4d> prev_tracks = this->filtered_tracks_;
  map<int, Eigen::Vector4d> new_tracks;

  for (auto &track : tracks) {
    map<int, Eigen::Vector4d>::iterator it = prev_tracks.find(track.first);
    Eigen::Vector4d X = it->second;
    Eigen::Vector4d Z = track.second;
    Eigen::Vector4d result;
    if (it == prev_tracks.end()) {
      // this is the first reading for this person.
      result = Z; // this->filter_.filter(Z, *(this->dt_));
    } else {
      result = this->filter_.filter(X, Z, *(this->dt_));
    }
    new_tracks[track.first] = result;

    double state[4] = {result[0], result[1], result[2], result[3]};
    this->util_.print_data("filtered_data.dat", track.first, timestamp, state);
  }
  this->filtered_tracks_ = new_tracks;
  return;
}

void Core::predict(double timestamp) {
  // run prediction on the detection data
  if (this->tracks_.empty())
    return;
  map<int, Eigen::Vector4d> tracks = this->tracks_;

  float time = 1.0; // how far into the future this should loop;
  double current_time = 0;

  stringstream ss;
  ofstream file;
  file.open(PATH + "predict_" + to_string((int)round(timestamp)) + ".log");

  for (auto &track : tracks) {
    Eigen::Vector4d X = track.second;

    ss << track.first << " ";
    ss << X[0] << " " << X[1] << " " << X[2] << " " << X[3];
    ss << " " << round(current_time) << endl;
  }

  for (int j = 0; j < 8; j++) {
    int i = 0;
    current_time += time;
    map<int, Eigen::Vector4d> track_interval = tracks;
    for (auto &track : tracks) {
      Eigen::Vector4d X = track.second;
      Eigen::Vector4d Z;

      // KalmanFilter kf = KalmanFilter(*this->dt_);
      KalmanFilter kf = KalmanFilter(0.1);

      Eigen::Vector4d result = kf.estimate(X, time);
      try {
        if (timestamp < 1 || j < 7) {
          int prev_file = (int)round(timestamp - 1);
          int cur_timestamp = (int)round(timestamp + current_time);
          X = this->util_.get_previous_prediction(prev_file, track.first,
                                                  cur_timestamp);
          KalmanFilter kf_trained = this->filter_;
          result = kf_trained.filter(X, result, 0.1);
        } else
          throw "in last itteration";
      } catch (const char *err) {
        result = kf.single_predict(result);
      }

      ss << track.first << " " << result[0] << " " << result[1] << " "
         << result[2] << " " << result[3] << " " << round(current_time) << endl;
      track_interval[track.first] = result;
      i++;
    }
    tracks = track_interval;
  }
  file << ss.str();
  file.close();
  return;
}

void Core::save(double timestamp) {
  // run prediction on the detection data
  if (this->tracks_.empty())
    return;
  map<int, Eigen::Vector4d> tracks = this->tracks_;
  this->tracks_.clear();

  string input;
  for (auto &track : tracks) {
    Eigen::Vector4d result;
    Eigen::Vector4d X = track.second;

    std::stringstream ss;
    ss << track.first << " " << X[0] << " " << X[1] << " " << X[2] << " "
       << X[3] << " " << round(timestamp) << endl;
    input += ss.str();
  }

  ofstream file;
  file.open(PATH + "measure.log", ios_base::app);
  file << input;
  file.close();
  return;
}

void Core::eval(string note) {
  // evaluate the predictions
  map<int, map<int, vector<double>>> measurement_data;
  vector<map<int, map<int, vector<double>>>> all_predict_data;
  // map<int,vector<double>> p_maxmin;

  std::stringstream ess; // create a string stream and add header.
  ess << "### mean error data for gnuplot ###" << endl << "#" << endl;
  ess << "#time \t pred step \t personid \t mean Er xy \t stdev xy \t mean Er "
         "th \t stdev th"
      << endl;

  std::stringstream ss; // create a string stream and add header.
  ss << "----- KF Movement Prediction Report -----" << endl;
  ss << "Test note: " << note << endl;

  /* read in data from the measurement file */
  ifstream mfile;
  mfile.open(PATH + "measure.log");
  int t, p;
  double xx, xy, vx, vy;
  ss << endl;
  // ss << "----- MEASURED DATA -----" << endl;
  while (mfile >> p >> xx >> xy >> vx >> vy >> t) {
    (measurement_data[p])[t] = {xx, xy, vx, vy};
    // if (all_people.find(p) == all_people.end())
    // all_people.insert(p);
  }
  mfile.close();

  for (int i = 0; i < 25; i++) {
    map<int, vector<vector<vector<double>>>> data_timewise;

    ss << endl;
    ss << "---- PREDICT DATA ----" << endl;

    ifstream pfile;
    pfile.open(PATH + "predict_" + to_string(i) + ".log");
    map<int, map<int, vector<double>>> predict_data;
    int t, p;
    double xx, xy, vx, vy;
    while (pfile >> p >> xx >> xy >> vx >> vy >> t) {
      (predict_data[p])[t] = {xx, xy, vx, vy};
      ss << "time(" << i << ")[" << t << "] person[" << p << "] POS: ";
      ss << xx << "\t" << xy << "\t" << vx << "\t" << vy << endl;
    }
    pfile.close();

    ss << endl << "EVALUATION:" << endl;
    map<int, map<int, vector<double>>>::iterator it = predict_data.begin();
    for (pair<int, map<int, vector<double>>> pelem : predict_data) {

      int person_id = pelem.first;
      map<int, vector<double>> predict_people_trials = pelem.second;

      map<int, vector<double>>::iterator it = predict_people_trials.begin();
      for (pair<int, vector<double>> elem : predict_people_trials) {
        int pred_time = elem.first;
        map<int, map<int, vector<double>>>::iterator mit =
            measurement_data.find(person_id);
        if (mit != measurement_data.end()) {
          map<int, vector<double>>::iterator elem_m =
              mit->second.find(pred_time+i);
          if (elem_m != mit->second.end()) {
            vector<double> pos_m = elem_m->second;
            vector<double> pos_p = elem.second;

            // update eval data file
            if (pos_m.size() == 4 && pos_p.size() == 4) {

              // add timewise data
              map<int, vector<vector<vector<double>>>>::iterator tw_ittr =
                  data_timewise.find(pred_time);
              vector<vector<vector<double>>> temp_tw;
              if (tw_ittr != data_timewise.end()) {
                temp_tw = tw_ittr->second;
                (temp_tw[0]).push_back(pos_m);
                (temp_tw[1]).push_back(pos_p);
                tw_ittr->second = temp_tw;
              } else {
                vector<vector<double>> temp_m = {pos_m};
                vector<vector<double>> temp_p = {pos_p};
                temp_tw.push_back(temp_m);
                temp_tw.push_back(temp_p);
                (data_timewise[pred_time]) = temp_tw;
              }

              ss << "time(" << i << ")[" << pred_time << "] person["
                 << person_id << "] diff: ";
              ss << this->util_.difference(pos_p[0], pos_m[0]) << "\t";
              ss << this->util_.difference(pos_p[1], pos_m[1]) << "\t";
              ss << this->util_.difference(pos_p[2], pos_m[2]) << "\t";
              ss << this->util_.difference(pos_p[3], pos_m[3]) << endl;
            }
          }
        }
      }
    }
    for (pair<int, vector<vector<vector<double>>>> pos_data : data_timewise) {
      int trial = pos_data.first;
      vector<vector<double>> m = pos_data.second[0];
      vector<vector<double>> p = pos_data.second[1];

      // write gnuplot format data
      this->util_.error_data_out(&ess, i, trial, p,
                                 m); // time - trial - pred data - measure data
    }

    all_predict_data.push_back(predict_data);
  }

  ss << "\n\n--------------------------------- RESULT: "
        "---------------------------------\n";
  for (int i = 1; i < 9; i++) {
    ss << endl << "+++ prediction @ " << i << "seconds: " << endl;
    for (pair<int, map<int, vector<double>>> child : measurement_data) {
      int person_id = child.first;
      map<int, vector<double>> measure_clock_pos = child.second;
      ss << endl << "person track " << person_id << ":" << endl;
      vector<double> diff_xx;
      vector<double> diff_xy;
      vector<double> diff_vx;
      vector<double> diff_vy;
      vector<double> measure_xx;
      vector<double> measure_xy;
      vector<double> measure_vx;
      vector<double> measure_vy;
      for (pair<int, vector<double>> elemm : measure_clock_pos) {
        vector<double> vm = elemm.second;

        if (all_predict_data.size() > elemm.first) {
          map<int, map<int, vector<double>>>::iterator pittr =
              all_predict_data[elemm.first].find(person_id);
          if (pittr != all_predict_data[elemm.first].end()) {
            vector<double> vp = pittr->second[i];
            if (vm.size() == 4 && vp.size() == 4) {
              diff_xx.push_back(vm[0] - vp[0]);
              diff_xy.push_back(vm[1] - vp[1]);
              diff_vx.push_back(vm[2] - vp[2]);
              diff_vy.push_back(vm[3] - vp[3]);
              measure_xx.push_back(vm[0]);
              measure_xy.push_back(vm[1]);
              measure_vx.push_back(vm[2]);
              measure_vy.push_back(vm[3]);
            }
          }
        }
      }
      // write human readable format data
      ss << "pos X :" << endl;
      this->util_.mean_print(&ss, diff_xx, measure_xx);
      ss << "pos Y :" << endl;
      this->util_.mean_print(&ss, diff_xy, measure_xy);
      ss << "vel X :" << endl;
      this->util_.mean_print(&ss, diff_vx, measure_vx);
      ss << "vel Y :" << endl;
      this->util_.mean_print(&ss, diff_vy, measure_vy);
    }
  }

  ofstream out(PATH + "prediction_report.txt");
  out << ss.str();
  out.close();

  ofstream timewise(PATH + "data_analysis.dat");
  timewise << ess.str();
  timewise.close();

  return;
}

void Core::measurement(const hyperion_msgs::TrackedPersons::ConstPtr &data) {
  // add in measurement data
  stringstream ss;
  vector<hyperion_msgs::TrackedPerson> persons(data->tracks);
  double time = data->header.stamp.sec;
  map<int, Eigen::Vector4d> tracks;
  for (auto i = persons.begin(); i != persons.end(); ++i) {
    hyperion_msgs::TrackedPerson person = *i;
    int id = person.track_id;
    double xx = person.pose.pose.position.x;
    double xy = person.pose.pose.position.y;
    double vx = person.twist.twist.linear.x;
    double vy = person.twist.twist.linear.y;
    Eigen::Vector4d v(xx, xy, vx, vy);
    double state[4] = {xx, xy, vx, vy};
    tracks[id] = v;
    ss << id << " " << state[0] << " " << state[1] << " " << state[2] << " "
       << state[3] << " " << to_string(time) << endl;
  }

  ofstream out;
  out.open(PATH + "raw_data.dat", ios_base::app);
  out << ss.str();
  out.close();

  this->buffer_.push_back(tracks);
  return;
}
