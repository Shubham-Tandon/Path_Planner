
#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

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


  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
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

  static int lane       = 1;  // Lane ID: 0 - lEFT lane; 1 - middle ln; 2 - right ln  
  static int lane_count = 0;  // Counter for time in current lane
  static double ref_vel = 0;  // Reference Velocity in mph

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event

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

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

          /*************************************************************************
          *  Section 1: Initilize Variables
          *************************************************************************/
          vector<double> ptsx;
          vector<double> ptsy;

          double target_vel = NEAR_SPEED_LIMIT;
          double ref_x      = car_x;
          double ref_y      = car_y;
          double ref_yaw    = deg2rad(car_yaw);
          double strt_waypt = WAYPOINT_START;
          double mddl_waypt = WAYPOINT_MID;
          double end_waypt  = WAYPOINT_END;
          bool car_close    = false;
          bool lt_lane_chng = false;
          bool rt_lane_chng = false;
          bool lane_change  = false;
          int target_lane   = MIDDLE_LANE;
          int prev_size     = previous_path_x.size();

          /* 
          5 surrounding cars are tracked in the algo
                          [2] |  [0]  | [4]
                              |       |   
                              |[Host] |
                              |       |
                          [1] |       | [3]

          */
          obj pos0;
          obj pos1;
          obj pos2;
          obj pos3;
          obj pos4;

          // Initialize the objects to default initial values
          initialize_objects(pos0);
          initialize_objects(pos1);
          initialize_objects(pos2);
          initialize_objects(pos3);
          initialize_objects(pos4);

          /* Section1: END */


          // Update the lane counter and restrict it to max value of 
          // STAY_IN_LANE_AFTER_LC + 5
          lane_count = (lane_count < STAY_IN_LANE_AFTER_LC + 5) ? lane_count + 1 :
                        lane_count;

          // Set the current s value to last s value of the previous path
          if (prev_size > 0) 
          {
            car_s = end_path_s;
          }

          /*************************************************************************
          * Section 2: Find the current lane of the host vehicle based on predefined
          *            width of th lanes.
          *************************************************************************/

          if (car_d < LANE_WIDTH && 
              car_d > 0.0          )
          {
            lane = LEFT_LANE;
          }
          else if (car_d < (2 * LANE_WIDTH) && 
                   car_d > LANE_WIDTH)
          {
            lane = MIDDLE_LANE;
          }
          else if (car_d < (3 * LANE_WIDTH) && 
                   car_d > (2 * LANE_WIDTH)   )
          {
            lane = RIGHT_LANE;
          }

          // Set the target lane as the current lane.
          target_lane = lane; 

          /* Section2: END */

          /*************************************************************************
          * Section3: Loop over the sension fusion data to find the car surrounding 
          *           the host vehicle. This data will be later used to decide the 
          *           behaviour of the host vehicle (i.e. follow the car in-front or
          *           change lanes).
          *************************************************************************/
          for(int i = 0; i < sensor_fusion.size(); i++)
          {
            float d = sensor_fusion[i][6];

            // Check if the car in the lane of the Self-Driving Car

            if (d < (HALF_LANE_WIDTH + LANE_WIDTH * lane + HALF_LANE_WIDTH) && 
                d > (HALF_LANE_WIDTH + LANE_WIDTH * lane - HALF_LANE_WIDTH))
            {
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx * vx + vy * vy);
              double check_car_s = sensor_fusion[i][5];

              // Find the s value of the car projected into future time
              check_car_s += ((double)prev_size * TIME_INCREMENT * check_speed);

              /*
              * if:
              *     car is infront of the host vehicle, AND
              *     car is closer that the "SAFE_DISTANCE (30m)", AND
              *     car is closer thatn any other car in the host lane.
              */
              if((check_car_s > car_s)                   && 
                 ((check_car_s - car_s) < SAFE_DISTANCE) &&
                 (check_car_s < pos0.s)                    )
              {
                car_close = true;
                pos0.vel  = check_speed;
                pos0.s    = check_car_s;

                if(check_speed < NEAR_SPEED_LIMIT)
                {
                  target_vel = check_speed;
                } 
              }
            }

            /*
            *  Check for cars in the left lane (only if the host is in the middle 
            *  or right lane).
            */
            if (lane == MIDDLE_LANE || lane == RIGHT_LANE)
            { 
              lt_lane_chng = true;

              // Check if a car is in the lane immediate left to the host.
              if ((d < (HALF_LANE_WIDTH + LANE_WIDTH*(lane - 1) + HALF_LANE_WIDTH))&& 
                  (d > (HALF_LANE_WIDTH + LANE_WIDTH*(lane - 1) - HALF_LANE_WIDTH)))
              {
                double object_s     = sensor_fusion[i][5];
                double object_vx    = sensor_fusion[i][3];
                double object_vy    = sensor_fusion[i][4];
                double object_speed = sqrt(object_vx * object_vx + object_vy * object_vy);

                object_s += ((double)prev_size * TIME_INCREMENT * object_speed);

                /*
                *  Searching for car in pos 1:
                *  if:
                *     car is behind the host, AND
                *     car is closer to host that any other vehicle in the left lane
                *  
                */
                if (object_s < car_s && 
                    fabs(object_s - car_s) < fabs(pos1.s - car_s))
                {
                  pos1.id   = i;
                  pos1.s    = object_s;
                  pos1.vel  = object_speed;
                }

                /*
                *  Searching for car in pos 2:
                *  if:
                *     car is ahead of the host, AND
                *     car is closer to host that any other vehicle in the left lane
                *  
                */
                else if (object_s > car_s && 
                         fabs(object_s - car_s) < fabs(pos2.s - car_s))
                {
                  pos2.id   = i;
                  pos2.s    = object_s;
                  pos2.vel  = object_speed;
                }
              }
            }

            /*
            *  Check for cars in the right lane (only if the host is in the middle 
            *  or left lane).
            */
            if (lane == MIDDLE_LANE || lane == LEFT_LANE)
            { 
              rt_lane_chng = true;

              // Check if a car is in the lane immediate right to the host.
              if((d < (HALF_LANE_WIDTH + LANE_WIDTH*(lane + 1) + HALF_LANE_WIDTH))&& 
                 (d > (HALF_LANE_WIDTH + LANE_WIDTH*(lane + 1) - HALF_LANE_WIDTH)))
              {
                double object_s     = sensor_fusion[i][5];
                double object_vx    = sensor_fusion[i][3];
                double object_vy    = sensor_fusion[i][4];
                double object_speed = sqrt(object_vx * object_vx + object_vy * object_vy);

                object_s += ((double)prev_size * TIME_INCREMENT * object_speed);

                /*
                *  Searching for car in pos 3:
                *  if:
                *     car is behind the host, AND
                *     car is closer to host that any other vehicle in the right lane
                *  
                */
                if (object_s < car_s && 
                    fabs(object_s - car_s) < fabs(pos3.s - car_s))
                {
                  pos3.id   = i;
                  pos3.s    = object_s;
                  pos3.vel  = object_speed;
                }

                /*
                *  Searching for car in pos 4:
                *  if:
                *     car is ahead of the host, AND
                *     car is closer to host that any other vehicle in the right lane
                *  
                */
                else if (object_s > car_s && 
                         fabs(object_s - car_s) < fabs(pos4.s - car_s))
                {
                  pos4.id   = i;
                  pos4.s    = object_s;
                  pos4.vel  = object_speed;
                }
              }
            }      
          }

          /* Section3: END */

          /*************************************************************************
          *  Section4: Decide Lane Change. 
          *            Lane change is decided based on eight pre-defined conditions
          *            defined by the positions and velocities of cars in position
          *            1, 2, 3 and 4.
          *************************************************************************/

          /* 
          *  check for lange change 
          *  if:
          *     Either left lane or right lane change maneuver is valid, AND
          *     There is a car close enough in front of the host vehcile to warrant
          *            a lane change, AND  
          *     The host has spent enought time in the current lane (this prevents
          *            rapid fluctuations in lane changes).
          */
          if ((lt_lane_chng == true || rt_lane_chng == true) && car_close == true &&
                lane_count > STAY_IN_LANE_AFTER_LC)
          {
            /*
            *  Preference 1: If there are no cars in the left lane, then left lane
            *                change is a safe maneuver.
            */
            if (pos1.id == -1 && pos2.id == -1 && lt_lane_chng == true)
            { 
              target_lane = lane - 1;
              target_vel  = car_speed;
              lane_change = true;
            }

            /*
            *  Preference 2: If there are no cars in the right lane, then right lane
            *                change is a safe maneuver.
            */
            if(pos3.id == -1 && pos4.id == -1 && 
               rt_lane_chng == true && lane_change == false)
            {
              target_lane = lane + 1;              
              target_vel  = car_speed;
              lane_change = true;
            }

            /*
            *  Preference 3: If there are no cars in front of the host vehicle the 
            *                left lane but there is a car behind in that lane. 
            */
            if(pos2.id == -1 && lt_lane_chng == true && lane_change == false)
            {

              double vel_diff             = pos1.vel - car_speed;
              double rear_obj_dist_thresh = interpol(vel_diff, MIN_VEL_REAR_CAR_LC, 
                                              MAX_DIST_REAR_CAR_LC, 
                                              MIN_DIST_REAR_CAR_LC, 
                                              MAX_DIST_REAR_CAR_LC);

              /* Check the distance of the car from the host agains a threshold 
                 calculated based on the velocity difference of the car and the host*/
              if (fabs(pos1.s - car_s) > rear_obj_dist_thresh)
              {
                target_lane = lane - 1;
                target_vel  = (car_speed > pos1.vel) ? car_speed 
                                                     : (pos1.vel + car_speed)/2;
                lane_change = true;  
              }

            }

            /*
            *  Preference 4: If there are no cars in front of the host vehicle the 
            *                right lane but there is a car behind in that lane. 
            */
            if(pos4.id == -1 && rt_lane_chng == true && lane_change == false)                    
            { 
              double vel_diff             = pos3.vel - car_speed;
              double rear_obj_dist_thresh = interpol(vel_diff, MIN_VEL_REAR_CAR_LC, 
                                              MAX_DIST_REAR_CAR_LC, 
                                              MIN_DIST_REAR_CAR_LC, 
                                              MAX_DIST_REAR_CAR_LC);

              /* Check the distance of the car from the host agains a threshold 
                 calculated based on the velocity difference of the car and the host*/
              if (fabs(pos3.s - car_s) > rear_obj_dist_thresh)
              {
                target_lane = lane + 1;
                target_vel  = (car_speed > pos3.vel) ? car_speed 
                                                     : (pos3.vel + car_speed)/2;
                lane_change = true;  
              }           
            }

            /*
            *  Preference 5: If there is a car in front of the host vehicle the 
            *                left lane but there is no car behind.
            */
            if(pos1.id == -1 && lt_lane_chng == true && lane_change == false)                    
            { 
              double vel_diff              = pos0.vel - pos2.vel;
              double front_obj_dist_thresh = interpol(vel_diff, MIN_VEL_FRONT_CAR_LC, 
                                              MAX_DIST_FRONT_CAR_LC, 
                                              MIN_DIST_FRONT_CAR_LC, 
                                              MAX_DIST_FRONT_CAR_LC);

              /* Check the distance of the car from the host agains a threshold 
                 calculated based on the velocity difference of the car and the host*/
              if ((pos2.s - pos0.s) > front_obj_dist_thresh) 
              {
                target_lane = lane - 1;              
                target_vel  = (car_speed < pos2.vel) ? car_speed : pos2.vel;
                lane_change = true;                 
              }
            } 

            /*
            *  Preference 6: If there is a car in front of the host vehicle the 
            *                right lane but there is no car behind. 
            */
            if(pos3.id == -1 && rt_lane_chng == true && lane_change == false)
            {              
              double vel_diff              = pos0.vel - pos4.vel;
              double front_obj_dist_thresh = interpol(vel_diff, MIN_VEL_FRONT_CAR_LC, 
                                              MAX_DIST_FRONT_CAR_LC, 
                                              MIN_DIST_FRONT_CAR_LC, 
                                              MAX_DIST_FRONT_CAR_LC);

              /* Check the distance of the car from the host agains a threshold 
                 calculated based on the velocity difference of the car and the host*/
              if ((pos4.s - pos0.s) > front_obj_dist_thresh) 
              {
                target_lane = lane + 1;              
                target_vel  = (car_speed < pos4.vel) ? car_speed : pos4.vel;
                lane_change = true;                 
              } 
            }

            /*
            *  Preference 7: If there is a car in front and behind the host vehicle  
            *                in the left lane.
            */
            if(pos1.id > -1 && pos2.id > -1 && 
               lt_lane_chng == true && lane_change == false)                    
            { 

              double vel_diff              = pos0.vel - pos2.vel;
              double front_obj_dist_thresh = interpol(vel_diff, MIN_VEL_FRONT_CAR_LC, 
                                              MAX_DIST_FRONT_CAR_LC, 
                                              MIN_DIST_FRONT_CAR_LC, 
                                              MAX_DIST_FRONT_CAR_LC);
              vel_diff = pos1.vel - car_speed;
              double rear_obj_dist_thresh = interpol(vel_diff, MIN_VEL_REAR_CAR_LC, 
                                MAX_DIST_REAR_CAR_LC, 
                                MIN_DIST_REAR_CAR_LC, 
                                MAX_DIST_REAR_CAR_LC);

              /* Check the distance of the car from the host agains a threshold 
                 calculated based on the velocity difference of the car and the host*/
              if ((car_s  - pos1.s) > rear_obj_dist_thresh && 
                  (pos2.s - pos0.s) > front_obj_dist_thresh  ) 
              {
                target_lane = lane - 1;              
                target_vel  = car_speed;
                lane_change = true;                 
              }
            } 

            /*
            *  Preference 8: If there is a car in front and behind the host vehicle  
            *                in the right lane.
            */
            if(pos3.id > -1 && pos4.id > -1 && 
                rt_lane_chng == true && lane_change == false)                    
            { 
              double vel_diff              = pos0.vel - pos4.vel;
              double front_obj_dist_thresh = interpol(vel_diff, MIN_VEL_FRONT_CAR_LC, 
                                              MAX_DIST_FRONT_CAR_LC, 
                                              MIN_DIST_FRONT_CAR_LC, 
                                              MAX_DIST_FRONT_CAR_LC);
              vel_diff = pos3.vel - car_speed;
              double rear_obj_dist_thresh = interpol(vel_diff, MIN_VEL_REAR_CAR_LC, 
                                MAX_DIST_REAR_CAR_LC, 
                                MIN_DIST_REAR_CAR_LC, 
                                MAX_DIST_REAR_CAR_LC);

              /* Check the distance of the car from the host agains a threshold 
                 calculated based on the velocity difference of the car and the host*/
              if ((car_s  - pos3.s) > rear_obj_dist_thresh && 
                  (pos4.s - pos0.s) > front_obj_dist_thresh  ) 
              {
                target_lane = lane + 1;              
                target_vel  = car_speed;
                lane_change = true;                 
              }
            } 
          }
          /* Section4: END */

          /*
          *  If lane change is detemined by the algorithm, then the spline waypoint
          *  are chosen accordingly. The spline waypoints in this case are chosen 
          *  further apart (30, 60, 90) to allow for a smoother lane change maneuver.
          */
          if (lane_change == true)
          {
            strt_waypt = WAYPOINT_START_LANE_CHANGE;
            mddl_waypt = WAYPOINT_MID_LANE_CHANGE;
            end_waypt  = WAYPOINT_END_LANE_CHANGE;
          }

          /*
          *  If the ref_vel is less that the target velocity then increment otherwise
          *  decrement the velocity. 
          */
          if (ref_vel < target_vel) 
          {
            ref_vel += SPEED_INCREMENT;
          }
          else
          {
            ref_vel -= SPEED_INCREMENT;
          }

          /* Restrict the ref_vel to be always within Speed Limit*/
          ref_vel = (ref_vel < NEAR_SPEED_LIMIT) ? ref_vel : NEAR_SPEED_LIMIT;

          /*************************************************************************
          * Section5: Find the path of the vehicle.
          *************************************************************************/

          /* Pick up the previous two points of the car toa llow for a smoother
          *  spline generation. */
          if (prev_size < 2)
          {
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }
          else
          {
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];

            double ref_x_prev = previous_path_x[prev_size - 2];
            double ref_y_prev = previous_path_y[prev_size - 2];
            ref_yaw           = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }

          /* 
          *  Get three waypoint in X, Y coordinate using the target lane information
          *  obtained from lane change section.
          */
          vector<double> next_wp0 = getXY(car_s + strt_waypt, 
                                          (HALF_LANE_WIDTH + LANE_WIDTH * target_lane), 
                                           map_waypoints_s, map_waypoints_x, 
                                           map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + mddl_waypt, 
                                          (HALF_LANE_WIDTH + LANE_WIDTH * target_lane), 
                                           map_waypoints_s, map_waypoints_x, 
                                           map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + end_waypt, 
                                          (HALF_LANE_WIDTH + LANE_WIDTH * target_lane), 
                                           map_waypoints_s, map_waypoints_x, 
                                           map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          /* Convert into local coordinate system of the host vehicle */
          for (int i = 0; i < ptsx.size(); i++ )
          {
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
            ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
          }

          /* Generate the spline curve based on the three points provided*/
          tk::spline s;
          s.set_points(ptsx, ptsy);

          /* Add previous path points which have not been traversed yet by the 
          *   simulator.*/
          for (int i = 0; i < previous_path_x.size(); i++)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          double target_x    = WAYPOINT_START;
          double target_y    = s(target_y);
          double target_dist = sqrt((target_x)*(target_x) + (target_y)*(target_y));
          double x_add_on    = 0.0;

          /* Add new path points so that the total path points = 50*/
          for(int i = 1; i <= PATH_POINTS - previous_path_x.size(); i++)
          {

            /* Divide the spline into smaller segments based on the ref_vel*/
            double N = (target_dist/(TIME_INCREMENT * ref_vel/MPH2MS));
            double x_point = x_add_on + (target_x)/N;
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;


            x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
            y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);

          }

          /*
          *  END: Section5 
          */


          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

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