#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"

#include "utils.h"
#include "waypoints.h"

using namespace std;
using Eigen::Vector2d;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
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


static double last_s = 0.0;


int main()
{
  uWS::Hub h;
  Waypoints wp;

  h.onMessage([&wp](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
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
          	double car_speed = double(j[1]["speed"]) * 0.447;

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];
//            cout << end_path_s << car_s << endl;

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];
          	json msgJson;

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;


 // double dist_inc = 0.5;
 //    for(int i = 0; i < 50; i++)
 //    {
 //          next_x_vals.push_back(car_x+(dist_inc*i)*cos(NUtils::deg2rad(car_yaw)));
 //          next_y_vals.push_back(car_y+(dist_inc*i)*sin(NUtils::deg2rad(car_yaw)));
 //    }
            cout << car_s << ", " << end_path_s << ", " << last_s;
            const double delta_s = 0.4;
            const int n = 2000 / 20;

            const int path_size = previous_path_x.size();
            for (int i = 0; i < path_size; ++i)
            {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }

            double s = car_s;

            if (path_size)
            {
              s = last_s;
//              const double d = NUtils::distance(car_x, car_y, previous_path_x[path_size-1], previous_path_y[path_size-1]);
//              s += d;
            }

            for (int i = path_size; i < n; ++i)
            {
                s += delta_s;
                const auto p = wp.getXY_interpolated(s, -6.0);
                last_s = s;
                //cout << s << ", " << p(0) << ", " << p(1) << endl;
                next_x_vals.push_back(p(0));
                next_y_vals.push_back(p(1));
            }
            cout << ", " << previous_path_x.size() << ", " << next_x_vals.size() << endl;

          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";
            //this_thread::sleep_for(chrono::milliseconds(40));
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
  // h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
  //                    size_t, size_t) {
  //   const std::string s = "<h1>Hello world!</h1>";
  //   if (req.getUrl().valueLength == 1) {
  //     res->end(s.data(), s.length());
  //   } else {
  //     // i guess this should be done more gracefully?
  //     res->end(nullptr, 0);
  //   }
  // });

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
