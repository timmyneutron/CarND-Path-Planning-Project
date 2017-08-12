#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "helperFunctions.h"

using namespace std;
using hf = HelperFunctions;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }
  
  double max_s = 6945.554;
  double ref_v = 0;
  int lane = 1;
  bool changing_lanes = false;

  h.onMessage([&changing_lanes,&lane,&ref_v,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;

    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];
            int num_cars = sensor_fusion.size();

          	json msgJson;

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

            car_yaw = hf::deg2rad(car_yaw);
            int path_size = previous_path_x.size();
            double dt = 0.02;
            double ref_yaw;

            for (int i = 0; i < path_size; ++i)
            {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }

            vector<double> spline_pts_x;
            vector<double> spline_pts_y;

            if (path_size < 2)
            {
              spline_pts_x.push_back(car_x);
              spline_pts_y.push_back(car_y);

              spline_pts_x.push_back(car_x + cos(car_yaw) * dt);
              spline_pts_y.push_back(car_y + sin(car_yaw) * dt);

              ref_yaw = car_yaw;
            }
            else
            {
              double x0 = previous_path_x[path_size-2];
              double x1 = previous_path_x[path_size-1];

              double y0 = previous_path_y[path_size-2];
              double y1 = previous_path_y[path_size-1];

              spline_pts_x.push_back(x0);
              spline_pts_x.push_back(x1);

              spline_pts_y.push_back(y0);
              spline_pts_y.push_back(y1);

              double x_diff = x1 - x0;
              double y_diff = y1 - y0;

              ref_yaw = atan2(y_diff, x_diff);
            }

            bool too_close = false;
            bool right_clear = true;
            bool left_clear = true;

            if (lane - 1 < 0)
            {
              left_clear  = false;
            }
            else if (lane + 1 > 2)
            {
              right_clear = false;
            }

            for (int i = 0; i < num_cars; ++i)
            {
              double next_car_s = sensor_fusion[i][5];
              double next_car_d = sensor_fusion[i][6];
              double s_diff = next_car_s - car_s;

              if (next_car_d > car_d - 2 && next_car_d < car_d + 2 && s_diff > 0 && s_diff < 30)
              {
                too_close = true;
              }

              if (next_car_d > 4.0 * (lane + 1) && next_car_d <  4.0 * (lane + 2) && fabs(s_diff) < 30)
              {
                right_clear = false;
              }

              if (next_car_d > 4.0 * (lane - 1) && next_car_d <  4.0 * lane && fabs(s_diff) < 30)
              {
                left_clear = false;
              }
            }

            if (too_close && !changing_lanes)
            {
              if (right_clear)
              {
                lane += 1;
              }
              else if (left_clear)
              {
                lane -= 1;
              }
              else
              {
                ref_v -= 2 * dt;
              }
            }
            else if (ref_v < 45.0 * 0.447)
            {
              ref_v += 5 * dt;
            }

            double target_d = 2.0 + lane * 4.0;
            double ds_wp;

            if (fabs(car_d - target_d > 2.0))
            {
              changing_lanes = true;
              ds_wp = 100;
            }
            else
            {
              changing_lanes = false;
              ds_wp = 30;
            }

            for (int i = 1; i < 4; ++i)
            {
              double next_s = car_s + ds_wp * i;
              vector<double> wp_xy = hf::getXY(next_s, target_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
              spline_pts_x.push_back(wp_xy[0]);
              spline_pts_y.push_back(wp_xy[1]);
            }

            vector<double> xy_car, spline_pts_x_car, spline_pts_y_car;

            for (int i = 0; i < 5; ++i)
            {
              xy_car = hf::global2car(spline_pts_x[1], spline_pts_y[1], ref_yaw, spline_pts_x[i], spline_pts_y[i]);
              spline_pts_x_car.push_back(xy_car[0]);
              spline_pts_y_car.push_back(xy_car[1]);
            }

            tk::spline s;
            s.set_points(spline_pts_x_car, spline_pts_y_car);

            for (int i = 1; i < 51 - path_size; ++i)
            {
             double next_x_car = ref_v * dt * i;
             double next_y_car = s(next_x_car);

             vector<double> xy_global = hf::car2global(spline_pts_x[1], spline_pts_y[1], ref_yaw, next_x_car, next_y_car);

             next_x_vals.push_back(xy_global[0]);
             next_y_vals.push_back(xy_global[1]);
            }

          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}