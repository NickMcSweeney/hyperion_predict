// standard lib includes
#include <fstream>
#include <iostream>
#include <map>
#include <math.h>
#include <sstream>
#include <string>
#include <vector>

#include <hyperion_predict/utils.h>

using namespace std;

Utils::Utils() {}

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

void Utils::read_to(map<int, map<int, vector<double>>> *dest, vector<string> data) {
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
