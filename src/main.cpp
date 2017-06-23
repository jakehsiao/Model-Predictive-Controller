#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <cstdio>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main(int argc, char* argv[]) {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;
  if(argc == 8){ // if parameters are specialized in running
    mpc.Init(atof(argv[1]), atof(argv[2]), atof(argv[3]), atof(argv[4]), atof(argv[5]), atof(argv[6]), atof(argv[7]));
  }
  else{
    mpc.Init(1, 1, 1, 200, 100, 1, 1); // use pre-tuned parameters
  }

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double x = j[1]["x"];
          double y = j[1]["y"];
          double psi = j[1]["psi"];
          double v_raw = j[1]["speed"]; // mph to m/s
	  double v = v_raw * 0.447;

          /*
          * TODO: Calculate steeering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          double steer_value;
          double throttle_value;
	  steer_value = j[1]["steering_angle"];
	  throttle_value = j[1]["throttle"];
            
	  // PART: get the state, print the state
          // Vehicle's version of ptsx
          int num_of_pts = ptsx.size();
          Eigen::VectorXd v_pts_x = Eigen::VectorXd(num_of_pts);
          Eigen::VectorXd v_pts_y = Eigen::VectorXd(num_of_pts);
          for(int i=0; i<num_of_pts; i++){
	    double dx = ptsx[i] - x;
	    double dy = ptsy[i] - y;
	    double cos_psi = cos(-psi);
	    double sin_psi = sin(-psi);
            v_pts_x(i) = dx * cos_psi - dy * sin_psi;
            v_pts_y(i) = dx * sin_psi + dy * cos_psi;
          }
          Eigen::VectorXd coeffs = polyfit(v_pts_x, v_pts_y, 3);
          double cte = coeffs[0];
          double epsi = psi - atan(coeffs[1]);

	  // there's a delay in actual control and the predicted state if current state is used,
	  // so use the predicted state after delay instead.
	  double dt = 0.1;
	  double Lf = 2.67;

	  double x_p = v * dt;
	  double y_p = 0;
	  double psi_p = - v * steer_value * dt / Lf;
	  double v_p = v + throttle_value * dt;
	  double cte_p = cte + v * sin(epsi) * dt;
	  double epsi_p = epsi + psi_p;

          Eigen::VectorXd state(6);
          state << x_p, y_p, psi_p, v_p, cte_p, epsi_p;
            
	  // PART: solve it, no debug require
          vector<double> actuators = mpc.Solve(state, coeffs);
          steer_value = - actuators[0]; //actual steer is the negative of solved steer
          throttle_value = actuators[1];
	  std::cout << "cte:" << cte << " epsi:" << epsi << std::endl;
	  std::cout << "steer:" << steer_value << " throttle:" << throttle_value << std::endl;
	  std::cout << "coeffs:" << coeffs[0] << " " << coeffs[1] << " " << coeffs[2] << " " << coeffs[3] << std::endl;
	  printf("---\n");

          json msgJson;
          msgJson["steering_angle"] = steer_value / 0.436; //Convert to -1 ~ 1
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
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
