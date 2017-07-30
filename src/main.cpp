//==============================================================================================
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
//==============================================================================================
#include "Utils.h"
#include "VehicleController.h"
#include "CarState.h"
//==============================================================================================
using namespace std;
using Eigen::Vector2d;
using json = nlohmann::json;
//==============================================================================================

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

//----------------------------------------------------------------------------------------------

int main()
{
  uWS::Hub h;
  VehicleController vehicle_controller;

  h.onMessage([&vehicle_controller](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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

        if (event == "telemetry")
        {
          	const double car_x = j[1]["x"];
          	const double car_y = j[1]["y"];
          	const double car_s = j[1]["s"];
          	const double car_d = j[1]["d"];
          	const double car_yaw = j[1]["yaw"];
          	const double car_speed = double(j[1]["speed"]) * 0.447;
          	//const double end_path_s = j[1]["end_path_s"];
          	//const double end_path_d = j[1]["end_path_d"];
            const CarState kCarState = { car_x, car_y, car_s, car_d, car_yaw, car_speed };

          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

            vehicle_controller.UpdateTrajectory(
              previous_path_x, previous_path_y,
              kCarState,
              next_x_vals, next_y_vals);

          	json msgJson;
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


//----------------------------------------------------------------------------------------------
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


//----------------------------------------------------------------------------------------------

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

//==============================================================================================
