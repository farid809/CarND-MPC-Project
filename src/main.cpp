#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "MPC.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

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
    std::cout << sdata << std::endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          double delta = j[1]["steering_angle"];
          double acceleration = j[1]["throttle"];
          /**
           * TODO: Calculate steering angle and throttle using MPC.
           * Both are in between [-1, 1].
           */

          
          // to estimate next state parameters after latency
          //  double latency = 0.1;
          //   double Lf = 2.67;
          //    px = px + (v * cos(psi) * latency);
          //    py = py + (v * sin(psi) * latency);
          //    psi = psi - ((v * delta * latency)/Lf);
          //    v = v + (acceleration * latency);
          


          //std::cout << "***********" << ptsx[0] << std::endl;
          //std::cout << "**********" << ptsx.size() << std::endl;
          //6 waypoints

          //the server returns waypoints using the map's coordinate system, which is different than the car's coordinate system. Transforming these waypoints will make it easier to both display them and to calculate the CTE and Epsi values for the model predictive controller.
          //Step1 Tranform from map to vechile coordinates
             Eigen::VectorXd waypointsX_Xd = Eigen::VectorXd::Map(ptsx.data(), 6);
             Eigen::VectorXd waypointsY_Xd = Eigen::VectorXd::Map(ptsy.data(), 6);

          
          for(int i=0; i< waypointsY_Xd.size(); i++)
          {
            //translate and rotate coordinate in one shot
            auto x=ptsx[i]-px;
            auto y=ptsy[i]-py;
            waypointsX_Xd[i]= (x)*cos(-psi) - (y)*sin(-psi);
            waypointsY_Xd[i]= (x)*sin(-psi) + (y)*cos(-psi);  
          }
          

          //std::cout << "waypointsX_Xd=" << waypointsX_Xd << std::endl;
          
          //polyfit to 3rd degree
           auto coeffs = polyfit(waypointsX_Xd, waypointsY_Xd, 3);
          
          //std::cout << "coeffs ="<< coeffs << std::endl;
          // // The cross track error is calculated by evaluating at polynomial at x,y =0
           double cte = polyeval(coeffs, 0);
        
          // // Due to the sign starting at 0, the orientation error is -f'(x).
          // //double epsi = psi - atan(coeffs[1]);
           double epsi = - atan(coeffs[1]);

          

           

         
          Eigen::VectorXd state(6);

           //VectorXd state(6);
           state << 0, 0, 0, v, cte, epsi;

           auto mpc_solution = mpc.Solve(state, coeffs);


          //MPC returns the actuators value
          double steer_value=mpc_solution[0];
          double throttle_value=mpc_solution[1];

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the 
          //   steering value back. Otherwise the values will be in between 
          //   [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] =steer_value/(deg2rad(25));
          msgJson["throttle"] = throttle_value;

          // Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          /**
           * TODO: add (x,y) points to list here, points are in reference to 
           *   the vehicle's coordinate system the points in the simulator are 
           *   connected by a Green line
           */


          for (int i = 2; i < mpc_solution.size();) {
              mpc_x_vals.push_back(mpc_solution[i]);
              mpc_y_vals.push_back(mpc_solution[i+1]);
            i=i+2;
          }

     



          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          // Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: add (x,y) points to list here, points are in reference to 
           *   the vehicle's coordinate system the points in the simulator are 
           *   connected by a Yellow line
           */


           for (double i = 0; i < 100; i += 3){
            next_x_vals.push_back(i);
            next_y_vals.push_back(polyeval(coeffs, i));
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          //   the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          //   around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE SUBMITTING.
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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