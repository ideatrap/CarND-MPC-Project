#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"
#include <math.h>

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

  Eigen::VectorXd yvals_ = Eigen::VectorXd::Map(yvals.data(), yvals.size());
  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals_);
  return result;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    //cout << "sdata: " << sdata << endl;
    //cout << "   " << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"]; // vehicle's current position - x
          double py = j[1]["y"]; // vehicle's current position - y
          double psi = j[1]["psi"];
          double v = j[1]["speed"];

          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */

          double steer_angle = j[1]["steering_angle"];
          double throttle_reading = j[1]["throttle"];
		  cout <<  "steer value: " <<  steer_angle <<  ", throttle value: " << throttle_reading << endl;
          //Find state at 100ms in the future
          double latency = 0.1;

          const int n_points = ptsx.size();

          //Transform the co-ords and convert to Eigen::VectorXd
          Eigen::VectorXd ptsx_vector(n_points), ptsy_vector(n_points);
          for(int i = 0; i < n_points; ++i) {
              double x = ptsx[i] - px;
              double y = ptsy[i] - py;
              ptsx_vector[i] = x * cos(-psi) - y * sin(-psi);
              ptsy_vector[i] = y * cos(-psi) + x * sin(-psi);
          }

          //fit a three degree polynomial to waypoints
          auto coeffs = polyfit(ptsx_vector, ptsy_vector, 3);

          // Calculate the car's future position after latency time
          double psi_pred = (-v* steer_angle * latency)/2.67; //orientation
          double px_pred = v*cos(psi) * latency; //assuming the same speed, predicted x position
          double v_pred = v + throttle_reading * latency;

          // predict the cross track error by putting car in future position (x = px)
          // y = 0 for current location
          double cte = polyeval(coeffs, px_pred);//cross track error

          //Calculate the orientatiin error
          double derivative = coeffs[1] + 2*coeffs[2]*px_pred + 3*coeffs[3]*px_pred*px_pred;
          double epsi = psi_pred - atan(derivative); //orientation error

          Eigen::VectorXd state(6);
          // the car is always at the center of its own position
		  state << px_pred, 0, psi_pred, v_pred, cte, epsi;

          //Calculate steeering angle and throttle using MPC.
          auto vars = mpc.Solve(state, coeffs);

          double steer_value = vars[0];
          double throttle_value = vars[1];

          //cout <<  "steer applying: " <<  steer_value <<  ", throttle applying: " << throttle_value << endl;
          cout << "  "<< endl;


          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = -steer_value;
          msgJson["throttle"] = throttle_value;


          //Display the MPC predicted trajectory
          //add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          // not starting 0 to eliminate points behind the car
          for (int i = 4; i < vars.size(); i+=2) {
            mpc_x_vals.push_back(vars[i]);
            mpc_y_vals.push_back(vars[i+1]);
          }


           // x_start 0 y_start 15 delta_start 90 a_start 104 N = 15
          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          for (int i = 1; i < n_points ; ++i ) {
                next_x_vals.push_back(ptsx_vector[i]);
                next_y_vals.push_back(ptsy_vector[i]);
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
		  //cout << " "<< endl;
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
