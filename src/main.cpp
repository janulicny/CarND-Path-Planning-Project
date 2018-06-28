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

using namespace std;

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
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

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
  double max_s = 6945.554;

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

  int lane = 1;  // we start in center lane
  double ref_vel = 0; // and at standstill
  
  // flag for lane change in progress
  bool lane_change_in_progress = false;
  // counter for lane change in progress
  int lane_change_counter = 0;
  
  
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &lane, &ref_vel, &lane_change_counter, &lane_change_in_progress](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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

          	json msgJson;
            
            
            // size of unsed path from previous iteration
            int path_size = previous_path_x.size();
            
            // reference vehicle position
            double pos_x = car_x;
            double pos_y = car_y;
            double pos_s = car_s;
            double angle = car_yaw;
            
            // coarse points for new path
            vector<double> ptsx;
          	vector<double> ptsy;

            // in case there is not enough points in unused path, extrapolate based on car angle
            if(path_size < 2)
            {
                ptsx.push_back(car_x - cos(car_yaw));
                ptsx.push_back(car_x);
                ptsy.push_back(car_y - sin(car_yaw));
                ptsy.push_back(car_y);
            }
            // otherwise use the last points from the previous paths
            else
            {
                pos_x = previous_path_x[path_size-1];
                pos_y = previous_path_y[path_size-1];
                pos_s = end_path_s;
                double prev_pos_x = previous_path_x[path_size-2];
                double prev_pos_y = previous_path_y[path_size-2];
                angle = atan2(pos_y-prev_pos_y, pos_x-prev_pos_x);
                ptsx.push_back(prev_pos_x);
                ptsx.push_back(pos_x);
                ptsy.push_back(prev_pos_y);
                ptsy.push_back(pos_y);
                
            }
            
            //sensor fusion
            
            //flags
            bool in_my_lane = false;
            bool left_occupied = false;
            bool right_occupied = false;
            
            
            
            // go through all other vehicles and set up flags
            for(int i = 0; i < sensor_fusion.size(); i++)
            {
                //flag for vehicle in the same longitudinal space
                bool next_to_us = false;
              
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double s = sensor_fusion[i][5];
                double d = sensor_fusion[i][6];
                
                double v = sqrt(vx*vx+vy*vy);
                
                //predict the longitudinal position
                double pred_s = s + (double)path_size*0.02*v;

                
                // check if there is vehicle in front of us, in the same lane and if it is closer than 30m
                if(d>(4*lane) && d<(4+4*lane) && pred_s>pos_s && (pred_s-pos_s)<30)
                {
                    in_my_lane = true;
                }
                // check if it is close to us (in longitudinal sense)
                // we will set the flag if
                // vehicle is slower and ahead of us in some distance
                // or if it is behind us some distance and has grater speed
                if(((pred_s>pos_s-30) && (pred_s<pos_s+20) && (v>car_speed)) || ((pred_s>pos_s-5) && (pred_s<pos_s+40) && (v<car_speed)))
                {
                    next_to_us = true;
                }                
                
                // check left lane change
                // disable left lane change if we are already in the leftmost lane
                if(lane==0)
                {
                    left_occupied = true;
                }
                else
                {
                    int target_lane = lane-1;
                    // check if the car is left of us and next to us
                    if(d>(4*target_lane) && d<(4+4*target_lane) && next_to_us)
                    {                                                                                                                                                                       
                        left_occupied = true;                                                
                    } 
                }
                              
                // check right lane change
                // disable right lane change if we are already in the right lane
                if(lane==2)
                {
                    right_occupied = true;
                }
                else
                {
                    int target_lane = lane+1;
                    if(d>(4*target_lane) && d<(4+4*target_lane) && next_to_us)
                    {
                        right_occupied = true;                        
                    } 
                }
            }
            
            // state machine
            // if there is a vehicle ahead of us
            if(in_my_lane)
            {
                // consider left turn
                if(left_occupied==false && lane_change_in_progress==false)
                {
                  lane -= 1;
                  lane_change_in_progress = true;
                  lane_change_counter = 0;
                }
                // if not possible consider right turn
                else if(right_occupied==false && lane_change_in_progress==false)
                {
                  lane += 1;
                  lane_change_in_progress = true;
                  lane_change_counter = 0;
                }
                // if not possible slow down
                else
                {
                    ref_vel -= 0.12;
                }
                
            }
            // if there is not vehicle ahead of us
            // if we are in left lane, consider lane change back to center
            else if(lane==0 && right_occupied==false && lane_change_in_progress==false)
            {
                lane = 1;
                lane_change_in_progress = true;
                lane_change_counter = 0;
            }
            // if we are in right lane, consider lane change back to center
            else if(lane==2 && left_occupied==false && lane_change_in_progress==false)
            {
                lane = 1;
                lane_change_in_progress = true;
                lane_change_counter = 0;
            }
            // or just simply speed up to the speed limit
            else if(ref_vel<22)
            {
                ref_vel += 0.12;
            }
            
            // this bit is implemented to disable erratic lane changes, when the vehicle can't choose between lanes
            // after we initiate lane change, the next one can be started only after some time
            lane_change_counter += 1;
            if(lane_change_counter>3*50)
            {
                lane_change_in_progress = false;
            }
            
          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
            // create prediction for 30, 60 and 90 meters from current car position
            for(int i = 1; i < 4; i++)
            {
                vector<double> next_wp = getXY(pos_s + 30*i, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                ptsx.push_back(next_wp[0]);
                ptsy.push_back(next_wp[1]);
            }
            
            // transform to local vehicle coordinates
            for(int i = 0; i < ptsx.size(); i++)
            {
                double shift_x = ptsx[i]-pos_x;
                double shift_y = ptsy[i]-pos_y;
                
                ptsx[i] = shift_x*cos(-angle)-shift_y*sin(-angle);
                ptsy[i] = shift_x*sin(-angle)+shift_y*cos(-angle);
                
            }
            
            // prepare spline based on coarse waypoints
            tk::spline s;
            s.set_points(ptsx, ptsy);
        
            // this will be the next path, we wann follow
            vector<double> next_x_vals;
          	vector<double> next_y_vals;
            
            // let's fill it with previous path
            for(int i = 0; i < path_size; i++)
            {    
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
            }
            
            // split spline, so we travel at reference speed
            double target_x = 30;
            double target_y = s(target_x);
            double target_dist = sqrt(target_x*target_x + target_y*target_y);
            
            double x_add_on = 0;
           

            for(int i = 1; i <= 50-path_size; i++)
            {    
                double N = target_dist/(0.02*ref_vel);
                double x_point = x_add_on + target_x/N;
                double y_point = s(x_point);
                
                x_add_on = x_point;
                
                //back to global coordinates 
                double temp_x = x_point;
                double temp_y = y_point;
                x_point = pos_x + temp_x*cos(angle)-temp_y*sin(angle);
                y_point = pos_y + temp_x*sin(angle)+temp_y*cos(angle);

                next_x_vals.push_back(x_point);
                next_y_vals.push_back(y_point);
            }
            
            // Debug
            cout<<"Ahead: "<<in_my_lane<<endl;
            cout<<"Left:  "<<left_occupied<<endl;
            cout<<"Right: "<<right_occupied<<endl;
            cout<<"Ch.l.: "<<lane_change_in_progress<<endl;
            cout<<"-------------"<<endl;
                     
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
