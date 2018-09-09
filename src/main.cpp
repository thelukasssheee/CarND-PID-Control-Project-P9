#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <vector>
#include <ctime>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;

  // DONE: Initialize the pid variable.
  PID pid;
  pid.optimizer_on = true;
  pid.Kp_init = 0.3;
  pid.Ki_init = 0.002;
  pid.Kd_init = 4.;
  pid.Init();

  // Initialize timing variables
  pid.start = std::clock();

  pid.t0 = (std::clock() - pid.start) / (double) CLOCKS_PER_SEC;

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */

          // Reset simulator if vehicle has left the road
          // if ((t_delta.count() > 5.) && (abs(cte) > 5.)) {
          pid.t1 = (std::clock() - pid.start) / (double) CLOCKS_PER_SEC;
          if (((pid.t1 - pid.t0) > 0.3) && (abs(cte) > 5.)) {
            // Generate error meassage in std::cout
            std::cout << "Reset!" << std::endl;

            // Generate and send error message for simulator to trigger the reset
            std::string msg("42[\"reset\", {}]");
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

            // Re-initialize PID controller:
            pid.Init();

            // Reset timer
            pid.t0 = (std::clock() - pid.start) / (double) CLOCKS_PER_SEC;

            // Exit function
            return;
          }

          // Update PID Error terms
          pid.UpdateError(cte);
          pid.TotalError(cte);

          // Calculate steer angle
          steer_value = pid.CalcSteerAngle(cte, speed, angle);

          // Limit steer angle to max values
          if (steer_value > 1.0)  { steer_value = 1.0; }
          if (steer_value < -1.0) { steer_value = -1.0;}

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!! Handle: " << &h << std::endl;

  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected!!! Handle: " << &h << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();

  return 0;
}
