// standard lib includes
#include <algorithm>
#include <fstream>
#include <iostream>
#include <map>
#include <math.h>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

#include <hyperion_predict/utils.h>

using namespace std;

Utils::Utils() {}

const string Utils::PATH =
    "/home/usernamix/Workspace/src/hyperion/hyperion_data/output/";

map<int, map<int, vector<double>>> Utils::read_in(vector<string> data) {
  // convert a string block to a nested map.
  map<int, map<int, vector<double>>> out;
  for (auto const &line : data) {
    stringstream ss(line);
    string word;
    double elem[6];
    int i = 0;
    while (ss >> word) {
      elem[i] = stod(word);
    }
    vector<double> pos = {elem[1], elem[2], elem[3], elem[4]};
    (out[elem[5]])[elem[1]] = pos;
  }
  return out;
}

void Utils::read_to(map<int, map<int, vector<double>>> *dest,
                    vector<string> data) {
  // convert a string block to a nested map.
  map<int, map<int, vector<double>>> out = *dest;
  for (auto const &line : data) {
    stringstream ss(line);
    string word;
    double elem[6];
    int i = 0;
    while (ss >> word) {
      elem[i] = stod(word);
    }
    vector<double> pos = {elem[1], elem[2], elem[3], elem[4]};
    (out[elem[5]])[elem[1]] = pos;
  }
  *dest = out;
}

string Utils::difference(double p, double m) {
  if (m + p == 0)
    return "0";
  return to_string(fabs(p - m) / ((p + m) / 2));
}

void Utils::mean_print(stringstream *ss, std::vector<double> diff,
                       std::vector<double> m) {

  double size = diff.size();
  auto abs_mean = [size](double sum, double d) { return sum + abs(d) / size; };
  auto std_mean = [size](double sum, double d) { return sum + d / size; };

  double mean = accumulate(diff.begin(), diff.end(), 0.0, std_mean);
  double mean_abs = accumulate(diff.begin(), diff.end(), 0.0, abs_mean);
  vector<double> d(diff.size());
  transform(diff.begin(), diff.end(), d.begin(),
            [mean](double x) { return x - mean; });
  double sq_sum = inner_product(d.begin(), d.end(), d.begin(), 0.0);
  double stdev = sqrt(sq_sum / diff.size());
  double measure_mean_abs = accumulate(m.begin(), m.end(), 0.0, abs_mean);
  double mape = (mean_abs / measure_mean_abs) * 100;

  *ss << "Mean Absolute Error: " << mean_abs << endl;
  *ss << "Mean Absolute Percent Error: " << mape << "%" << endl;
  *ss << "stdev: " << stdev << endl;
}

void Utils::print_data(string file_name, int id, double time, double state[4]) {
  // write data to file
  stringstream ss;
  ss << id << " " << state[0] << " " << state[1] << " " << state[2] << " "
     << state[3] << " " << to_string(time) << endl;

  ofstream out;
  out.open(PATH + file_name, ios_base::app);
  out << ss.str();
  out.close();
}

Eigen::Vector4d Utils::get_previous_prediction(int file_number, int person_id,
                                               int time) {
  ifstream file;
  file.open(PATH + "predict_" + to_string(file_number) + ".log");
  if (!file.is_open())
    throw "file does not exist!";
  Eigen::Vector4d data;
  int t, p;
  double xx, xy, vx, vy;
  while (file >> p >> xx >> xy >> vx >> vy >> t) {
    if (p == person_id && t + file_number == time) {
      data = {xx, xy, vx, vy};
      file.close();
      return data;
    }
  }
  file.close();
  throw "prev track not found!";
}

void Utils::error_data_out(stringstream *ss, int t, int i,
                           vector<vector<double>> vp, vector<vector<double>> vm) {
  (*ss) << t << "\t" << i << "\t"  << "\t";
  
  vector<double> d_pos;
  vector<double> d_th;
  for(vector<vector<double>>::iterator p = vp.begin(),  m = vm.begin(); p !=vp.end() && m != vm.end(); ++m, ++p) {
    // calc distance between measured and predicted position.
	double xx_e = fabs((*m)[0]-(*p)[0]);
	double xy_e = fabs((*m)[1]-(*p)[1]);
	d_pos.push_back(sqrt(xx_e*xx_e + xy_e*xy_e));
	
	// calc angle between projected and predicted vectors
	double mx = (*m)[2];
	double px = (*p)[2];
	double my = (*m)[3];
	double py = (*p)[3];
	d_th.push_back(acos((px*mx+py+my)/(sqrt(px*px+py*py)*sqrt(mx*mx+my*my))));
  }
  double size = d_pos.size();
  auto mean = [size](double sum, double d) { return sum + d / size; };

	// calc average position error
	double mean_pos = accumulate(d_pos.begin(),d_pos.end(), 0.0, mean);

	// calc stdev in position
	vector<double> diff(d_pos.size());
	transform(d_pos.begin(), d_pos.end(), diff.begin(),
            [mean_pos](double x) { return x - mean_pos; });
	double sq_sum = inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
	double stdev_pos = sqrt(sq_sum / size);

	// calc average trajectory error
	double mean_th = accumulate(d_th.begin(),d_th.end(), 0.0, mean);

	// calc stdev in trajectory error
	diff = vector<double>(d_th.size());
	transform(d_th.begin(), d_th.end(), diff.begin(),
            [mean_th](double x) { return x - mean_th; });
	sq_sum = inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
	double stdev_th = sqrt(sq_sum / size);

	(*ss) << mean_pos << "\t" << stdev_pos << "\t" << mean_th << "\t" << stdev_th << endl;
}
