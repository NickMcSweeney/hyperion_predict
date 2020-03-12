class Utils {
public:
	Utils();

private:
	std::string error_;

public:
	std::map<int, std::map<int, std::vector<double>>> read_in(std::vector<std::string> data);
	void read_to(std::map<int, std::map<int, std::vector<double>>> *dest,
							 std::vector<std::string> data);
	std::string difference(double p, double m);
};
