#ifndef HELPER_FUNCTIONS_H
#define HELPER_FUNCTIONS_H

#include <vector>

using namespace std;

class HelperFunctions
{
  public:
  	HelperFunctions();
  	virtual ~HelperFunctions();
	static double deg2rad(double deg);
	static double rad2deg(double rad);
  	static vector<double> global2car(double car_x, double car_y, double psi, double px, double py);
	static vector<double> car2global(double car_x, double car_y, double psi, double px, double py);
	static double distance(double x1, double y1, double x2, double y2);
	static int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y);
	static int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y);
	static vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y);
	static vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y);
};

#endif /* HELPER_FUNCTIONS_H */