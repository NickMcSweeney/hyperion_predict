#include <Eigen/Dense>

class Utils {
public:
  Utils();

private:
  std::string error_;
  static const std::string PATH;

public:
  std::map<int, std::map<int, std::vector<double>>>
  read_in(std::vector<std::string> data);
  void read_to(std::map<int, std::map<int, std::vector<double>>> *dest,
               std::vector<std::string> data);
  std::string difference(double p, double m);

  std::string calc_error(std::vector<double> m, std::vector<double> p);
  void mean_print(std::stringstream *ss, std::vector<double> diff,
                  std::vector<double> m);
  void print_data(std::string file_name, int id, double time, double state[4]);

  Eigen::Vector4d get_previous_prediction(int file_number, int person_id,
                                          int time);

  void error_data_out(std::stringstream *ss, int time, int step,
                      std::vector<std::vector<double>> p,
                      std::vector<std::vector<double>> m);
};
