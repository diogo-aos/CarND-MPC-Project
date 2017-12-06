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

int main(int argc, char *argv[]) {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;
  mpc.model_latency = 0;
  mpc.sys_latency = 100;
  mpc.print_in = 1;
  mpc.print_out = 1;
  mpc.print_errors = 1;
  if (argc == 2){
    mpc.init(argv[1]);
  }

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    if (mpc.print_in)
      cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          auto start = std::chrono::high_resolution_clock::now();

          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          double delta = j[1]["steering_angle"];
          double a = j[1]["throttle"];

          mpc.prev_delta = delta;
          mpc.prev_a = a;

          // normalize psi
          // while (psi > M_PI)  psi -= 2.*M_PI;
          // while (psi < -M_PI)  psi += 2.*M_PI;

          //
          // TODO: transform waypoints from map to car reference
          //
          Eigen::VectorXd wp_x = Eigen::VectorXd(ptsx.size());
          Eigen::VectorXd wp_y = Eigen::VectorXd(ptsy.size());
          double rot_teta = -psi;
          for(int i=0; i < ptsx.size(); i++){
            double x_t = ptsx[i] - px,  // x, y translated
                   y_t = ptsy[i] - py;
            wp_x[i] = x_t * cos(rot_teta) - y_t * sin(rot_teta);
            wp_y[i] = x_t * sin(rot_teta) + y_t * cos(rot_teta);
          }

          // reset vehicle position in accordance to transformation
          px = 0.0; py = 0.0; psi = 0;

          // fit polynimal to received waypoints
          auto coeffs = polyfit(wp_x, wp_y, 3);

          //
          // ..:: compute errors ::..
          //
          // compute cte
          double cte = polyeval(coeffs, px) - py;
          // compute epsi
          Eigen::VectorXd coeffs_d = Eigen::VectorXd(3);
          coeffs_d << coeffs[1], 2 * coeffs[2], 3 * coeffs[3];
          double epsi = psi - atan(polyeval(coeffs_d, px));

          //
          // TODO: model latency
          //
          // project vehicle coordinates into future by dt = latency
          // model latency might take into account computation latency as well
          if (mpc.model_latency > 0){
            vector<double> proj_pos = mpc.project({px, py, psi, v, cte, epsi}, {delta, a}, mpc.model_latency / 1000);
            px = proj_pos[0];
            py = proj_pos[1];
            psi = proj_pos[2];
            v = proj_pos[3];
            cte = proj_pos[4];
            epsi = proj_pos[5];
          }

          // create state vector
          Eigen::VectorXd state = Eigen::VectorXd(6);
          state << px, py, psi, v, cte, epsi;
          if (mpc.print_errors){
            cout << "cte=" << cte << endl;
            cout << "epsi=" << epsi << endl;
          }

          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */

          double steer_value;
          double throttle_value;

          vector<double> solution = mpc.Solve(state, coeffs);

          steer_value = solution[0];  // minus sign because axis y is inverted in simulator
          throttle_value = solution[1];  // minus sign because axis y is inverted in simulator

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = -steer_value / deg2rad(25);
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          int i, pos_start=2;
          for (i=pos_start; i<solution.size(); i+=2){
            mpc_x_vals.push_back(solution[i]);
            mpc_y_vals.push_back(solution[i+1]);

          }
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          for (int i=0; i<wp_x.size(); i++){
            next_x_vals.push_back(wp_x[i]);
            next_y_vals.push_back(wp_y[i]);

          }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          if (mpc.print_out)
            std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.

          auto finish = std::chrono::high_resolution_clock::now();
          std::chrono::duration<double> elapsed = finish - start;
          if (mpc.print_errors)
            std::cout << "Elapsed time: " << elapsed.count() << " s\n";
          if (mpc.sys_latency > 0)
            this_thread::sleep_for(chrono::milliseconds(mpc.sys_latency));
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
