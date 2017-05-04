#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <fstream>
#include <ctime>

#include <stdlib.h>

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

void dumpToFile(std::string filename, std::string text) {
  std::ofstream dataFile;
  dataFile.open(filename, std::ios::trunc);
  dataFile << text;
  dataFile.close();
}


int main()
{
  uWS::Hub h;

  // Initialize the PID controller.
  PID pid;

  // PID parameters
  double init_Kp = 0.66;
  double init_Ki = 0.15;
  double init_Kd = 12.0;
  unsigned int integral_length = 2;
  unsigned int twiddle_length = 1400; // this is when to optimize twiddle. ideally from each lap. UNUSED

  pid.Init(init_Kp, init_Ki, init_Kd, integral_length, twiddle_length);

  // extra control parameters outside of PID
  double speed_limit = 70.0; // full throttle below this speed. brake above this speed.
  double safe_steering = 0.15; // full throttle below this steering angle. no throttle above

  // logging infrastructure
  long long frame = 0;
  int prev_time = 0;

  h.onMessage(
    [&pid,&init_Kp,&init_Ki,&init_Kd,&frame,&prev_time,&safe_steering,&speed_limit]
    (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data));

      if (s != "") {

        double in_steering =0.0;
        double in_throttle =0.0;
        double in_speed =0.0;
        double in_cte = 0.0;
        double out_throttle =0.0;
        double out_steering =0.0;

        // increment measurement count
        frame++;

        // calculate delta_t from last observation
        int stop = clock();
        double dt = (prev_time > 0) ? (stop - prev_time) / double(CLOCKS_PER_SEC) : 0.001;
        prev_time = stop;

        // print number of measurements processed from time to time
        if (std::ldiv(frame, 200).rem==0)
          std::cout << "frame: " << frame << std::endl;

        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          in_steering = std::stod(j[1]["steering_angle"].get<std::string>());
          in_throttle = std::stod(j[1]["throttle"].get<std::string>());
          in_speed = std::stod(j[1]["speed"].get<std::string>());
          in_cte = std::stod(j[1]["cte"].get<std::string>());

          // limit speed with braking
          if (in_speed > speed_limit)
            out_throttle = -0.1;
          else
            out_throttle = 1.0;

          // vary PID parameters with speed. higher speed makes system more sensitive
          if (in_speed < 50.)
          {
            pid.Kp_ = init_Kp;
            pid.Kd_ = init_Kd;
          }
          else if (in_speed < 60.)
          {
            pid.Kp_ = init_Kp * 0.95;
            pid.Kd_ = init_Kd * 1.2;
          }
          else if (in_speed < 66.)
          {
            pid.Kp_ = init_Kp * 0.9;
            pid.Kd_ = init_Kd * 1.4;
          }
          else if (in_speed < 73.)
          {
            pid.Kp_ = init_Kp * 0.85;
            pid.Kd_ = init_Kd * 1.6;
          }
          else
          {
            pid.Kp_ = init_Kp * 0.6;
            pid.Kd_ = init_Kd * 1.8;
          }

          // predict the steering angle using PID controller
          out_steering = pid.PredictSteering(in_cte, in_speed, dt);

          // ease off accelerator if steering is too steep
          if (frame > 100 && (abs(out_steering) > safe_steering))
            out_throttle = 0.0;

          // normalize steering to be [-1, 1]
          if (out_steering < -1.)
            out_steering = -1.;
          else if (out_steering > 1.)
            out_steering = 1.;

          // record new error in PID history for later use
          pid.UpdateError(in_cte, in_speed, dt);
          // Twiddle iteration process. UNUSED as it was found ineffective.
          //pid.TwiddleIfEnoughHistory();

          // DEBUG print
          std::cout << "IN: CTE: " << in_cte << ", speed: " << in_speed << ".     OUT: Steering Value: " << out_steering << ". Throttle: " << out_throttle << std::endl;

          json msgJson;
          msgJson["steering_angle"] = out_steering;
          msgJson["throttle"] = out_throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";

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
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
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
}
