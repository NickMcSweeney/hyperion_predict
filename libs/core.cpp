// standard lib includes
#include <algorithm>
#include <fstream>
#include <iostream>
#include <map>
#include <numeric>
#include <string>
#include <unordered_set>
#include <vector>
// ros includes
#include <ros/ros.h>
// custom includes
#include <hyperion_msgs/TrackedPerson.h>
#include <hyperion_predict/core.h>
#include <hyperion_predict/kalman_filter.h>

#include <math.h>

using namespace std;

/**
 * CORE module for prediction of movement
 * based on kalman filter prediction
 */

Core::Core(float *dt, ros::NodeHandle *nodehandle) {
	this->dt_ = dt;
	this->nh = *nodehandle;

	// initialize all the ROSey stuff here
	this->pub = this->nh.advertise<hyperion_msgs::TrackedPersons>("/hyperion/prediction", 1000);
	this->sub = this->nh.subscribe("/detection/pos_array", 1000, &Core::measurement, this);

	ofstream out("measure.log");
	out.close();

	this->util_ = Utils();
}

void Core::run() {
	// queue data from the image detection node
	// Vector<Eigen::Vector4d> result;
	// int i = 0;
	// for (auto& [key, value]: this->tracks_) {
	// Eigen::Vector4d X = value;
	// float time = 1000; // how far into the future this should loop;
	// HyperionPredict::KalmanFilter kf = KalmanFilter();
	// result[i] = kf.filter(X, *this->dt_, time);
	// i++;
	//}
}

void Core::predict(double timestamp) {
	// run prediction on the detection data
	// map<int, Eigen::Vector4d> tracks = this->tracks_;

	Eigen::Vector4d v(1, 1, 0.1, 0);
	map<int, Eigen::Vector4d> tracks;
	tracks[1] = v;
	tracks[2] = v;

	float time = 1.0; // how far into the future this should loop;
	double current_time = 0;

	ofstream out("predict_" + to_string((int)round(timestamp)) + ".log");
	out.close();

	for (int j = 0; j < 3; j++) {
		int i = 0;
		string input;
		current_time += time;
		map<int, Eigen::Vector4d> track_interval = tracks;
		for (auto &track : tracks) {
			Eigen::Vector4d X = track.second;

			KalmanFilter kf = KalmanFilter(*this->dt_);

			Eigen::Vector4d result = kf.estimate(X, time);

			std::stringstream ss;
			ss << track.first << " " << result[0] << " " << result[1] << " " << result[2] << " "
				 << result[3] << " " << round(current_time) << endl;
			input += ss.str();
			track_interval[track.first] = result;
			i++;
		}
		tracks = track_interval;

		ofstream file;
		file.open("predict_" + to_string((int)round(timestamp)) + ".log", ios_base::app);
		file << input;
		file.close();
	}
	return;
}

void Core::save(double timestamp) {
	// run prediction on the detection data
	// map<int, Eigen::Vector4d> tracks = this->tracks_;

	Eigen::Vector4d v(1, 1, 0.1, 0);
	Eigen::Vector4d w(1.6, 1.5, 0.1, 0);
	Eigen::Vector4d rr(1.8, 1.5, 0.1, 0);
	map<int, Eigen::Vector4d> tracks;
	tracks[1] = timestamp >= 6 ? rr : v;
	tracks[1] = timestamp > 3 && timestamp < 6 ? w : v;
	tracks[2] = v;

	string input;
	for (auto &track : tracks) {
		Eigen::Vector4d result;
		Eigen::Vector4d X = track.second;

		std::stringstream ss;
		ss << track.first << " " << X[0] << " " << X[1] << " " << X[2] << " " << X[3] << " "
			 << round(timestamp) << endl;
		input += ss.str();
	}

	ofstream file;
	file.open("measure.log", ios_base::app);
	file << input;
	file.close();
	return;
}

void Core::eval(string note) {
	// evaluate the predictions
	map<int, map<int, vector<double>>> measurement_data;
	vector<map<int, map<int, vector<double>>>> all_predict_data;
	// map<int,vector<double>> p_maxmin;

	std::stringstream ss; // create a string stream and add header.
	ss << "----- KF Movement Prediction Report -----" << endl;
	ss << "Test note: " << note << endl;

	/* read in data from the measurement file */
	ifstream mfile;
	mfile.open("measure.log");
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

	for (int i = 0; i < 4; i++) {
		ss << endl;
		ss << "---- PREDICT DATA ----" << endl;

		ifstream pfile;
		pfile.open("predict_" + to_string(i) + ".log");
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
				map<int, map<int, vector<double>>>::iterator mit = measurement_data.find(person_id);
				if (mit != measurement_data.end()) {
					map<int, vector<double>>::iterator elem_m = mit->second.find(pred_time);
					if (elem_m != mit->second.end()) {
						vector<double> pos_m = elem_m->second;
						vector<double> pos_p = elem.second;
						if (pos_m.size() == 4 && pos_p.size() == 4) {
							ss << "time(" << i << ")[" << pred_time << "] person[" << person_id << "] diff: ";
							ss << this->util_.difference(pos_p[0], pos_m[0]) << "\t";
							ss << this->util_.difference(pos_p[1], pos_m[1]) << "\t";
							ss << this->util_.difference(pos_p[2], pos_m[2]) << "\t";
							ss << this->util_.difference(pos_p[3], pos_m[3]) << endl;
						}
					}
				}
			}
		}
		all_predict_data.push_back(predict_data);
	}

	ss << "\n\n--------------------------------- RESULT: ---------------------------------\n";
	for (int i = 1; i < 4; i++) {
		ss << endl << "prediction @ " << i << "seconds: " << endl;
		for (pair<int, map<int, vector<double>>> child : measurement_data) {
			int person_id = child.first;
			map<int, vector<double>> measure_clock_pos = child.second;
			ss << "person track " << person_id << ":" << endl;
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
							diff_xx.push_back(vm[1] - vp[1]);
							diff_xx.push_back(vm[2] - vp[2]);
							diff_xx.push_back(vm[3] - vp[3]);
							measure_xx.push_back(vm[0]);
							measure_xy.push_back(vm[1]);
							measure_vx.push_back(vm[2]);
							measure_vy.push_back(vm[3]);
						}
					}
				}
			}

			double size = diff_xx.size();
			auto abs_mean = [size](double sum, double d) { return sum + abs(d) / size; };
			auto std_mean = [size](double sum, double d) { return sum + d / size; };

			double mean_xx = accumulate(diff_xx.begin(), diff_xx.end(), 0.0, std_mean);
			double mean_abs_xx = accumulate(diff_xx.begin(), diff_xx.end(), 0.0, abs_mean);
			vector<double> dxx(diff_xx.size());
			transform(diff_xx.begin(), diff_xx.end(), dxx.begin(),
								[mean_xx](double x) { return x - mean_xx; });
			double sq_sum_xx = inner_product(dxx.begin(), dxx.end(), dxx.begin(), 0.0);
			double stdev_xx = sqrt(sq_sum_xx / diff_xx.size());
			double measure_mean_abs_xx = accumulate(measure_xx.begin(), measure_xx.end(), 0.0, abs_mean);
			double mape_xx = (mean_abs_xx / measure_mean_abs_xx) * 100;

			double sum_xy = accumulate(diff_xy.begin(), diff_xy.end(), 0.0);
			double mean_abs_xy = sum_xy / diff_xy.size();

			double sum_vx = accumulate(diff_vx.begin(), diff_vx.end(), 0.0);
			double mean_abs_vx = sum_vx / diff_vx.size();

			double sum_vy = accumulate(diff_vy.begin(), diff_vy.end(), 0.0);
			double mean_abs_vy = sum_vy / diff_vy.size();

			ss << "Mean Absolute Error xx: " << mean_abs_xx << endl;
			ss << "Mean Absolute Percent Error xx: " << mape_xx << "%" << endl;
			ss << "stdev xx: " << stdev_xx << endl;

			ss << "Mean Absolute Error xy: " << mean_abs_xy << endl;
			ss << "sum xy: " << sum_xy << endl;
			ss << "Mean Absolute Error vx: " << mean_abs_vx << endl;
			ss << "sum vx: " << sum_vx << endl;
			ss << "Mean Absolute Error vy: " << mean_abs_vy << endl;
			ss << "sum vy: " << sum_vy << endl;
		}
	}

	ofstream out("prediction_report.txt");
	out << ss.str();
	out.close();

	return;
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
		Eigen::Vector4d v(person.pose.pose.position.x, person.pose.pose.position.y,
											person.twist.twist.linear.x, person.twist.twist.linear.y);
		// Eigen::Matrix3d m(3, 3);
		// m << person.pose.pose.position.x, person.pose.pose.position.y,
		// person.pose.pose.position.z, person.twist.twist.linear.x,
		// person.twist.twist.linear.y, person.twist.twist.linear.z,
		// person.twist.twist.angular.x, person.twist.twist.angular.y,
		// person.twist.twist.angular.z;
		// this may need to be much much more complex.
		this->tracks_[id] = v;
	}
}
