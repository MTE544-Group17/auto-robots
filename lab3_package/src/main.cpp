//  ///////////////////////////////////////////////////////////
//
// turtlebot_example.cpp
// This file contains example code for use with ME 597 lab 2
// It outlines the basic setup of a ros node and the various
// inputs and outputs needed for this lab
//
// Author: James Servos
// Edited: Nima Mohajerin
//
// //////////////////////////////////////////////////////////
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <vector>
// #include <iostream>
// #include <string>
// #include <ctime>
// #include <chrono>
#include <cmath>

#define WAYPOINT_THRESHOLD    0.1
#define THETA_THRESHOLD    0.01
#define P_CONTROL_LIN    0.75
#define P_CONTROL_ANG    0.25
#define MAP_DIM    10.0
#define MAP_RES    0.1
#define X_OFFSET   -1.0
#define Y_OFFSET   -5.0

const int MAP_SIZE = 100;
const int ROBOT_WIDTH = 3;
const int NUM_POINTS = 1400;
const int NUM_CONNECTIONS = 5;

using namespace std;
vector<int> shortestPath;

float line_segment_x = 0;
float line_segment_y = 0;
float line_segment_x_final = 0;
float line_segment_y_final = 0;
float dist_error = 0;
int current_index = 1;

int collisionMap[MAP_SIZE * MAP_SIZE];

bool mapReady = false;

struct Point {
  int x = 0;
  int y = 0;
  vector<int> nearestNeighbours;
  bool visited = false;
  int parentNodeIdex;
};

struct Neighbour {
  int pointIndex;
  double dist;
};

// ros::Publisher pose_publisher;
ros::Publisher marker_pub;

double ips_x;
double ips_y;
double ips_yaw;

short sgn(int x) { return x >= 0 ? 1 : -1; }

double random_gen() {
  return (double)(rand() % 10000) / 10000.0;
}

//Callback function for the Position topic (SIMULATION)
void pose_callback(const gazebo_msgs::ModelStates& msg) {
  int i;
  for(i = 0; i < msg.name.size(); i++) if(msg.name[i] == "mobile_base") break;

  ips_x = msg.pose[i].position.x - X_OFFSET;
  ips_y = msg.pose[i].position.y -   Y_OFFSET;
  ips_yaw = tf::getYaw(msg.pose[i].orientation);
  ROS_DEBUG("pose_callback X: %f Y: %f Yaw: %f", ips_x, ips_y, ips_yaw);

  //Create tf broadcaster
  ROS_DEBUG("sending broadcaster");
  static tf::TransformBroadcaster broadcaster_map;
  tf::Transform transform_map;
  transform_map.setOrigin( tf::Vector3(ips_x, ips_y, 0.0) );
  tf::Quaternion q;
  q.setRPY(0,0,ips_yaw);
  transform_map.setRotation(q);
  broadcaster_map.sendTransform(tf::StampedTransform(transform_map, ros::Time::now(), "base_link", "map"));

    //Create tf broadcaster
  ROS_DEBUG("sending broadcaster");
  static tf::TransformBroadcaster broadcaster_vis;
  tf::Transform transform_vis;
  transform_vis.setOrigin( tf::Vector3(ips_x, ips_y, 0.0) );
  tf::Quaternion w;
  w.setRPY(0,0,ips_yaw);
  transform_vis.setRotation(w);
  broadcaster_map.sendTransform(tf::StampedTransform(transform_vis, ros::Time::now(), "base_link", "visualization_marker"));
}

//Callback function for the Position topic (LIVE)
/*
void pose_callback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
  ips_x X = msg.pose.pose.position.x; // Robot X psotition
  ips_y Y = msg.pose.pose.position.y; // Robot Y psotition
  ips_yaw = tf::getYaw(msg.pose.pose.orientation); // Robot Yaw
  ROS_DEBUG("pose_callback X: %f Y: %f Yaw: %f", X, Y, Yaw);
}*/


//Callback function for the map
void map_callback(const nav_msgs::OccupancyGrid& msg) {
  ROS_INFO("Map Width : %d", msg.info.width);
  ROS_INFO("Map Height : %d", msg.info.height);

  for(int i = 0; i < MAP_SIZE * MAP_SIZE; i++) {
    collisionMap[i] = (int) msg.data[i];
  }

  ROS_INFO("Map Ready ...");
  mapReady = true;
}

//Bresenham line algorithm (pass empty vectors)
// Usage: (x0, y0) is the first point and (x1, y1) is the second point. The calculated
//        points (x, y) are stored in the x and y vector. x and y should be empty
//    vectors of integers and shold be defined where this function is called from.
void bresenham(int x0, int y0, int x1, int y1, vector<int>& x, vector<int>& y) {

    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int dx2 = x1 - x0;
    int dy2 = y1 - y0;

    const bool s = abs(dy) > abs(dx);

    if (s) {
        int dx2 = dx;
        dx = dy;
        dy = dx2;
    }

    int inc1 = 2 * dy;
    int d = inc1 - dx;
    int inc2 = d - dx;

    x.push_back(x0);
    y.push_back(y0);

    while (x0 != x1 || y0 != y1) {
        if (s) y0+=sgn(dy2); else x0+=sgn(dx2);
        if (d < 0) d += inc1;
        else {
            d += inc2;
            if (s) x0+=sgn(dx2); else y0+=sgn(dy2);
        }

        //Add point to vector
        x.push_back(x0);
        y.push_back(y0);
    }
}

/***************************************
PRM
***************************************/

vector<Point> pointMap;

void generateRandomMap() {
  for(int i = 0; i < NUM_POINTS; i++) {
    Point newPoint;
    newPoint.x = (int) (random_gen() * (MAP_SIZE - 2) + 1);
    newPoint.y = (int) (random_gen() * (MAP_SIZE - 2) + 1);

    pointMap.push_back(newPoint);
  }
}

bool inCollision(int index) {
  int min_x = index % 100 - ROBOT_WIDTH;
  int max_x = index % 100 + ROBOT_WIDTH;

  int min_y = (int)(index / 100) - ROBOT_WIDTH;
  int max_y = (int)(index / 100) + ROBOT_WIDTH;

  if(min_x < 0 || max_x > MAP_SIZE) { return true; }
  if(min_y < 0 || max_y > MAP_SIZE) { return true; }

  for(int x = min_x; x < max_x; x++) {
    for(int y = min_y; y < max_y; y++) {
      if(collisionMap[y*100 + x] > 0) {
        return true;
      }
    }
  }

  return false;
}

void removeCollisionPoints() {
  ROS_INFO("Number of points : %d", (int) pointMap.size());
  ROS_INFO("Removing points in collision....");

  vector<Point>::iterator p = pointMap.begin();

  while(p != pointMap.end()) {
    int index = p->y * 100 + p->x;

    if(inCollision(index)) {
      p = pointMap.erase(p);
    } else {
      p++;
    }
  }

  ROS_INFO("Final number of points : %d\n", (int) pointMap.size());
}

bool compareByDistance(const Neighbour &a, const Neighbour &b) {
  return a.dist < b.dist;
}

void connectNearestNeighbours() {
  for(int pIndex = 0; pIndex < pointMap.size(); pIndex++) {
    vector<Neighbour> allNeighbours;

    for(int nIndex = 0; nIndex < pointMap.size(); nIndex++) {
      if(nIndex == pIndex) { continue; }

      Neighbour n;
      n.pointIndex = nIndex;
      n.dist = sqrt(pow(pointMap[nIndex].x - pointMap[pIndex].x, 2) + pow(pointMap[nIndex].y - pointMap[pIndex].y, 2));
      allNeighbours.push_back(n);
    }

    sort(allNeighbours.begin(), allNeighbours.end(), compareByDistance);
    int iterations = NUM_CONNECTIONS;
    for(int i = 0; i < iterations; i++) {
      if(allNeighbours[i].dist > 1) {
        pointMap[pIndex].nearestNeighbours.push_back(allNeighbours[i].pointIndex);
      } else {
        iterations++;
      }
    }
  }

  // for(int pIndex = 0; pIndex < pointMap.size(); pIndex++) {
  //   ROS_INFO("Point %d : [%d %d %d %d %d]", pIndex, pointMap[pIndex].nearestNeighbours[0], pointMap[pIndex].nearestNeighbours[1], pointMap[pIndex].nearestNeighbours[2], pointMap[pIndex].nearestNeighbours[3], pointMap[pIndex].nearestNeighbours[4]);
  // }
}

void removeCollisionPaths() {
  for(auto &point:pointMap) {
    vector<int>::iterator neighbour = point.nearestNeighbours.begin();

    while(neighbour != point.nearestNeighbours.end()) {
      vector<int> x;
      vector<int> y;

      bresenham(point.x, point.y, pointMap[*neighbour].x, pointMap[*neighbour].y, x, y);

      bool pathCollides = false;
      for(int i = 0; i < x.size(); i++) {
        if(collisionMap[MAP_SIZE*y[i] + x[i]] > 0) {
          pathCollides = true;
        }
      }

      if(pathCollides) {
        neighbour = point.nearestNeighbours.erase(neighbour);
      } else {
        neighbour++;
      }
    }
  }

  // for(int pIndex = 0; pIndex < pointMap.size(); pIndex++) {
  //   ROS_INFO("$ Point %d : [%d paths]", pIndex, (int)pointMap[pIndex].nearestNeighbours.size());
  // }
}

/***************************************
A * Path Search
***************************************/

int findClosestPoint(int x, int y) {
  vector<Neighbour> allNeighbours;

  for(int nIndex = 0; nIndex < pointMap.size(); nIndex++) {
    Neighbour n;
    n.pointIndex = nIndex;
    n.dist = sqrt(pow(pointMap[nIndex].x - x, 2) + pow(pointMap[nIndex].y - y, 2));
    allNeighbours.push_back(n);
  }

  sort(allNeighbours.begin(), allNeighbours.end(), compareByDistance);

  return allNeighbours[0].pointIndex;
}

void prepareForTraversal() {
  for(auto & point:pointMap) {
    point.visited = false;
    point.parentNodeIdex = -2;
  }
}

struct PathPoint {
  int x;
  int y;
  int index;
  double cost;
  double heuristicCost;
};

void pushOntoStack(vector<PathPoint> * stack, int nodeIndex, double cost, int goalNode) {
  PathPoint p;

  p.x = pointMap[nodeIndex].x;
  p.y = pointMap[nodeIndex].y;
  p.index = nodeIndex;
  p.cost = cost;
  p.heuristicCost = sqrt(pow(p.x - pointMap[goalNode].x, 2) + pow(p.y - pointMap[goalNode].y, 2));

  pointMap[nodeIndex].visited = true;
  ROS_INFO("Right before push # %d", nodeIndex);

  stack->push_back(p);
  ROS_INFO("Pushed to stack # %d", nodeIndex);
}

bool compareByFullCost(const PathPoint &a, const PathPoint &b) {
  return (a.cost + a.heuristicCost) > (b.cost + b.heuristicCost);
}

vector<int> getShortestPath(int startNode, int goalNode) {
  prepareForTraversal();

  vector<int> shortestPath;
  vector<PathPoint> stack;

  pointMap[startNode].parentNodeIdex = -1;
  pushOntoStack(&stack, startNode, 0.0f, goalNode);

  while(!stack.empty()) {
    ROS_INFO("Start loop of size : %d ", (int) stack.size());
    sort(stack.begin(), stack.end(), compareByFullCost);
    PathPoint currentNode = stack.back();
    stack.pop_back();
    ROS_INFO("Popped, size : %d ", (int) stack.size());

    if(currentNode.index == goalNode) {
      ROS_INFO("Found goal node!");
      shortestPath.push_back(currentNode.index);
      int parent = pointMap[currentNode.index].parentNodeIdex;
      ROS_INFO("Starting looping result");
      do {
        ROS_INFO("Looping parent %d", parent);
        shortestPath.push_back(parent);
        parent = pointMap[parent].parentNodeIdex;
      } while (parent != -1);

      ROS_INFO("Found path of length : %d", (int) (shortestPath.size()));
      return shortestPath;
    }

    for(auto & neighbour:pointMap[currentNode.index].nearestNeighbours) {
      ROS_INFO("Looping over neighbours");
      if(!pointMap[neighbour].visited) {
        pointMap[neighbour].parentNodeIdex = currentNode.index;
        double interNodeCost = sqrt(pow(currentNode.x - pointMap[neighbour].x, 2) + pow(currentNode.y - pointMap[neighbour].y, 2));
        ROS_INFO("Pushing on to stack");
        pushOntoStack(&stack, neighbour, currentNode.cost + interNodeCost, goalNode);
      }
    }

    ROS_INFO("Looping ~~");
  }

  ROS_INFO("Exited *****");
  shortestPath.clear();
  return shortestPath;
}

/***************************************
Robot Movement Controller
***************************************/

float theta_error(float line_segment_y, float line_segment_x) {
  float theta_ref = atan2(line_segment_y, line_segment_x);
  float theta_error = theta_ref - ips_yaw;

  if(theta_error > M_PI){
    theta_error = theta_error - 2*M_PI;
  }
  else if (theta_error < -M_PI) {
    theta_error = theta_error + 2*M_PI;
  }
  return theta_error;
}

int main(int argc, char **argv)
{
    //Initialize the ROS framework
    ros::init(argc,argv,"main_control");
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
        ros::console::notifyLoggerLevelsChanged();
    }
    ros::NodeHandle n;

    //Subscribe to the desired topics and assign callbacks
    // ros::Subscriber pose_sub = n.subscribe("/indoor_pos", 1, pose_callback);
    ros::Subscriber pose_sub = n.subscribe("/gazebo/model_states", 1, pose_callback);
    ros::Subscriber map_sub = n.subscribe("/map", 1, map_callback);

    //Setup topics to Publish from this node
    ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
    // pose_publisher = n.advertise<geometry_msgs::PoseStamped>("/pose", 1, true);
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);

    //Velocity control variable
    geometry_msgs::Twist vel;

    //Set the loop rate
    ros::Rate loop_rate(20);    //20Hz update rate

    visualization_msgs::Marker points;
    points.header.frame_id = "/map";
    points.header.stamp = ros::Time::now();
    points.ns = "points";
    points.action = visualization_msgs::Marker::ADD;
    points.type = visualization_msgs::Marker::POINTS;
    points.id = 1;
    points.points.clear();

    points.scale.x = 0.08f;
    points.scale.y = 0.08f;

    points.color.r = 1.0f;
    points.color.a = 1.0f;

    visualization_msgs::Marker collisionPoints;
    collisionPoints.header.frame_id = "/map";
    collisionPoints.header.stamp = ros::Time::now();
    collisionPoints.ns = "collisionPoints";
    collisionPoints.action = visualization_msgs::Marker::ADD;
    collisionPoints.type = visualization_msgs::Marker::POINTS;
    collisionPoints.id = 2;
    collisionPoints.points.clear();

    collisionPoints.scale.x = 0.12f;
    collisionPoints.scale.y = 0.12f;

    collisionPoints.color.r = 0.8f;
    collisionPoints.color.g = 0.8f;
    collisionPoints.color.b = 1.0f;
    collisionPoints.color.a = 1.0f;

    visualization_msgs::Marker waypointMarkers;
    waypointMarkers.header.frame_id = "/map";
    waypointMarkers.header.stamp = ros::Time::now();
    waypointMarkers.ns = "waypointMarkers";
    waypointMarkers.action = visualization_msgs::Marker::ADD;
    waypointMarkers.type = visualization_msgs::Marker::POINTS;
    waypointMarkers.id = 3;
    waypointMarkers.points.clear();

    waypointMarkers.scale.x = 0.18f;
    waypointMarkers.scale.y = 0.18f;

    waypointMarkers.color.r = 0.0f;
    waypointMarkers.color.g = 1.0f;
    waypointMarkers.color.b = 0.0f;
    waypointMarkers.color.a = 1.0f;

    visualization_msgs::Marker allPaths;
    allPaths.header.frame_id = "/map";
    allPaths.header.stamp = ros::Time::now();
    allPaths.ns = "allPaths";
    allPaths.action = visualization_msgs::Marker::ADD;
    allPaths.type = visualization_msgs::Marker::LINE_LIST;
    allPaths.id = 4;
    allPaths.points.clear();

    allPaths.scale.x = 0.03f;
    allPaths.scale.y = 0.03f;

    allPaths.color.r = 0.0f;
    allPaths.color.g = 1.0f;
    allPaths.color.b = 0.0f;
    allPaths.color.a = 0.1f;

    visualization_msgs::Marker traversedPaths;
    traversedPaths.header.frame_id = "/map";
    traversedPaths.header.stamp = ros::Time::now();
    traversedPaths.ns = "traversedPaths";
    traversedPaths.action = visualization_msgs::Marker::ADD;
    traversedPaths.type = visualization_msgs::Marker::LINE_LIST;
    traversedPaths.id = 5;

    traversedPaths.scale.x = 0.08f;
    traversedPaths.scale.y = 0.08f;

    traversedPaths.color.r = 1.0f;
    traversedPaths.color.g = 1.0f;
    traversedPaths.color.b = 1.0f;
    traversedPaths.color.a = 1.0f;


    ROS_DEBUG("pose_callback X: %f Y: %f Yaw: %f", ips_x, ips_y, ips_yaw);

    while (ros::ok())
    {
      loop_rate.sleep(); //Maintain the loop rate
      ros::spinOnce();   //Check for new messages
      static double waypoints[16] = {ips_x, ips_y, ips_yaw,  5.0, 5.0, 0.0,   9.0, 1.0, 3.14,    9.0, 5.0, -3.14,    8.0, 8.0, 3.14};

      //Main loop code goes here:
      // vel.linear.x = 1.0; // set linear speed
      // vel.angular.z = 1.0; // set angular speed
      ROS_INFO("Looping...");
      if(mapReady) {
        generateRandomMap();
        removeCollisionPoints();
        connectNearestNeighbours();
        removeCollisionPaths();

        vector<int> waypointNodes;
        for(int i = 0; i < 12; i += 3) {
          int index = findClosestPoint((int) (waypoints[i] * 10), (int) (waypoints[i+1] * 10));
          waypointNodes.push_back(index);
        }


        // for(int i = 0; i < waypointNodes.size(); i++) {
          // vector<int> shortestPath = getShortestPath(waypointNodes[i], waypointNodes[i+1]);
          shortestPath = getShortestPath(waypointNodes[0], waypointNodes[1]);
          if(shortestPath.empty()) {
            mapReady = false;
            ROS_INFO("Path not found");
            continue;
          }
          traversedPaths.points.clear();

          for(int j = 0; j < shortestPath.size() - 1; j++) {
            geometry_msgs::Point p1;
            geometry_msgs::Point p2;

            p1.x = (pointMap[shortestPath[j]].x)*MAP_RES + X_OFFSET;
            p1.y = (pointMap[shortestPath[j]].y)*MAP_RES + Y_OFFSET;
            traversedPaths.points.push_back(p1);
            p2.x = (pointMap[shortestPath[j+1]].x)*MAP_RES + X_OFFSET;
            p2.y = (pointMap[shortestPath[j+1]].y)*MAP_RES + Y_OFFSET;
            traversedPaths.points.push_back(p2);
          }
          ROS_DEBUG("size_shortest= %d", shortestPath.size());


          for(auto &p:traversedPaths.points) {
            ROS_INFO("Path Node : [%f, %f]", p.x, p.y);
          }
        // }



        // geometry_msgs::Point wp1;
        // wp1.x = 20;
        // wp1.y = 10;
        // waypointMarkers.points.push_back(wp1);
        // geometry_msgs::Point wp2;
        // wp2.x = 80;
        // wp2.y = 10;
        // waypointMarkers.points.push_back(wp2);

        for(auto &point:pointMap) {
          geometry_msgs::Point p1;
          p1.x = (point.x)*MAP_RES + X_OFFSET;
          p1.y = (point.y)*MAP_RES + Y_OFFSET;
          for(auto &neighbour:point.nearestNeighbours) {
            geometry_msgs::Point p2;
            p2.x = (pointMap[neighbour].x)*MAP_RES + X_OFFSET;
            p2.y = (pointMap[neighbour].y)*MAP_RES + Y_OFFSET;
            allPaths.points.push_back(p1);
            allPaths.points.push_back(p2);
          }
        }

        mapReady = false;

        vector<geometry_msgs::Point> displayPointsVec;
        for(auto &point:pointMap) {
          geometry_msgs::Point p;
          p.x = (point.x)*MAP_RES + X_OFFSET;
          p.y = (point.y)*MAP_RES + Y_OFFSET;
          displayPointsVec.push_back(p);
        }
        points.points = displayPointsVec;

        vector<geometry_msgs::Point> displayCollisionPointsVec;
        for(int i =0; i < MAP_SIZE * MAP_SIZE; i++) {
          if(collisionMap[i] > 0) {
            geometry_msgs::Point p;
            p.x = (i%MAP_SIZE)*MAP_RES + X_OFFSET;
            p.y = ((int) (i /MAP_SIZE))*MAP_RES + Y_OFFSET;
            displayCollisionPointsVec.push_back(p);
          }
        }
        collisionPoints.points = displayCollisionPointsVec;

        reverse(shortestPath.begin(), shortestPath.end());
      }

      marker_pub.publish(allPaths);
      marker_pub.publish(traversedPaths);
      marker_pub.publish(points);
      marker_pub.publish(collisionPoints);
      marker_pub.publish(waypointMarkers);

      ROS_DEBUG("shortestPath size = %d", shortestPath.size());


      ROS_DEBUG("current_index = %d", current_index);
      ROS_DEBUG("shortestPath[current_index] = %d", shortestPath[current_index]);

      ROS_DEBUG("pointMap.x raw = %d", pointMap[shortestPath[current_index]].x);
      ROS_DEBUG("pointMap.x [m] = %f", pointMap[shortestPath[current_index]].x * MAP_RES);
      ROS_DEBUG("pointMap.y raw = %d", pointMap[shortestPath[current_index]].y);
      ROS_DEBUG("pointMap.y [m] = %f", pointMap[shortestPath[current_index]].y * MAP_RES);


      line_segment_x = pointMap[shortestPath[current_index]].x * MAP_RES - ips_x;
      line_segment_y = pointMap[shortestPath[current_index]].y * MAP_RES - ips_y;

      ROS_DEBUG("line_segment_x = %f", line_segment_x);
      ROS_DEBUG("line_segment_y = %f", line_segment_y);

      line_segment_x_final = pointMap[shortestPath[shortestPath.size()-1]].x * MAP_RES - ips_x;
      line_segment_y_final = pointMap[shortestPath[shortestPath.size()-1]].y * MAP_RES - ips_y;

      if ((abs(line_segment_x_final) <= WAYPOINT_THRESHOLD && abs(line_segment_y_final) <= WAYPOINT_THRESHOLD)) {
         ROS_DEBUG("DONE");
         vel.linear.x = 0.0;
         vel.angular.z = 0.0;
      }
      else {
        ROS_DEBUG("theta error: %f", theta_error(line_segment_y, line_segment_x));
        if (abs(theta_error(line_segment_y, line_segment_x)) <= THETA_THRESHOLD){
          ROS_DEBUG("DONE TURNING, my yaw: %f, the yaw of the waypoint:%f", ips_yaw, atan2(line_segment_y, line_segment_x));
          vel.angular.z = 0.0;
          if (abs(line_segment_x) < WAYPOINT_THRESHOLD && abs(line_segment_y) < WAYPOINT_THRESHOLD) {
            float X = pointMap[shortestPath[current_index]].x * MAP_RES;
            float Y = pointMap[shortestPath[current_index]].y * MAP_RES;
            ROS_DEBUG("DONE waypoint %f %f", X, Y);
            vel.linear.x = 0.0;
            current_index++;
          } else {
            float X = pointMap[shortestPath[current_index]].x * MAP_RES;
            float Y = pointMap[shortestPath[current_index]].y * MAP_RES;
            ROS_DEBUG("moving to waypoint %f %f currently at %f %f", X, Y, ips_x, ips_y);
            float dist_error = sqrt(line_segment_x*line_segment_x + line_segment_y*line_segment_y);
            vel.linear.x = P_CONTROL_LIN*dist_error;
          }
        } else {
          vel.linear.x = 0.0;
          ROS_DEBUG(" my yaw: %f, the yaw of the waypoint:%f", ips_yaw, atan2(line_segment_y, line_segment_x));
          vel.angular.z = P_CONTROL_ANG*theta_error(line_segment_y, line_segment_x);
        }
      }
      velocity_publisher.publish(vel); // Publish the command velocity
    }

    return 0;
}
