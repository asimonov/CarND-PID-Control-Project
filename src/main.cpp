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
  static double init_Kp = 0.66;
  static double init_Ki = 0.15;
  static double init_Kd = 11.0;
  static unsigned int integral_length = 2;
  //static unsigned int twiddle_length = 5660; // this is when to optimize twiddle. every lap
  //static unsigned int twiddle_length = 3800; // this is when to optimize twiddle. every lap
  static unsigned int twiddle_length = 1400; // this is when to optimize twiddle. every lap
  pid.Init(init_Kp, init_Ki, init_Kd, integral_length, twiddle_length);


  // initialize logging infrastructure. all static to be accessible from lambdas
  static long long frame = 0;
  // full log
  static std::ostringstream fileFullLogName;
  fileFullLogName << "data/log-full.txt";
  static std::ofstream fileFullLog;
  fileFullLog.open(fileFullLogName.str(), std::ios::trunc); // remove file contents if it already exists
  fileFullLog.close();
  // controls log
  static std::ostringstream fileLogName;
  fileLogName << "data/log.txt";
  static std::ofstream fileLog;
  fileLog.open(fileLogName.str(), std::ios::trunc); // remove file contents if it already exists
  fileLog.close();

  static int prev_time = 0;

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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

        frame++;

        int stop = clock();
        double dt = (prev_time > 0) ? (stop - prev_time) / double(CLOCKS_PER_SEC) : 0.001;
        prev_time = stop;

        if (std::ldiv(frame, 200).rem==0)
          std::cout << "frame: " << frame << std::endl;

        // save full log
//        fileFullLog.open(fileFullLogName.str(), std::ios::app);
//        fileFullLog << "frame: " << frame << std::endl;
//        fileFullLog << s << std::endl;
//        fileFullLog.close();

        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          in_steering = std::stod(j[1]["steering_angle"].get<std::string>());
          in_throttle = std::stod(j[1]["throttle"].get<std::string>());
          in_speed = std::stod(j[1]["speed"].get<std::string>());
          in_cte = std::stod(j[1]["cte"].get<std::string>());

          /*
          * Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */

          // limit speed with braking
          if (in_speed > 90.)
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

          // predict the steering angle
          out_steering = pid.PredictSteering(in_cte, in_speed, dt);

          // slow down if steering too steep
          if (frame > 100 && (abs(out_steering) > 0.15))
            out_throttle = 0.0;

          // normalize steering
          if (out_steering < -1.)
            out_steering = -1.;
          else if (out_steering > 1.)
            out_steering = 1.;

          pid.UpdateError(in_cte, in_speed, dt);
          //pid.TwiddleIfEnoughHistory();

          // DEBUG
          //std::cout << "IN CTE: " << in_cte << ", speed: " << in_speed << ". Total Error: " << pid.TotalError() << " Steering Value: " << out_steering << ". Throttle: " << out_throttle << std::endl;
          std::cout << "IN CTE: " << in_cte << ", speed: " << in_speed << ". Steering Value: " << out_steering << ". Throttle: " << out_throttle << std::endl;

          json msgJson;
          msgJson["steering_angle"] = out_steering;
          msgJson["throttle"] = out_throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
//          std::cout << msg << std::endl;

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }

        // save full log
        fileLog.open(fileLogName.str(), std::ios::app);
        fileLog << frame << " " << in_steering << " " << in_throttle << " " << in_speed << " " << in_cte << " "
                << out_steering << " " << out_throttle << std::endl;
        fileLog.close();

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
