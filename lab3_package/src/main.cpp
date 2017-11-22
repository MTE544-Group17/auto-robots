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
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <vector>
// #include <iostream>
// #include <string>
// #include <ctime>
// #include <chrono>

#include <cmath>

const int MAP_SIZE = 100;
const int ROBOT_WIDTH = 3;
const int NUM_POINTS = 400;
const int NUM_CONNECTIONS = 8;

using namespace std;

int collisionMap[MAP_SIZE * MAP_SIZE];

bool mapReady = false;

struct Point {
  int x = 0;
  int y = 0;
  vector<int> nearestNeighbours;
};

struct Neighbour {
  int pointIndex;
  double dist;
};

ros::Publisher pose_publisher;
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

  ips_x = msg.pose[i].position.x ;
  ips_y = msg.pose[i].position.y ;
  ips_yaw = tf::getYaw(msg.pose[i].orientation);
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
//	  vectors of integers and shold be defined where this function is called from.
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

    for(int i = 0; i < NUM_CONNECTIONS; i++) {
      pointMap[pIndex].nearestNeighbours.push_back(allNeighbours[i].pointIndex);
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

  for(int pIndex = 0; pIndex < pointMap.size(); pIndex++) {
    ROS_INFO("$ Point %d : [%d]", pIndex, (int)pointMap[pIndex].nearestNeighbours.size());
  }
}

int main(int argc, char **argv)
{
	  //Initialize the ROS framework
    ros::init(argc,argv,"main_control");
    ros::NodeHandle n;

    //Subscribe to the desired topics and assign callbacks
    ros::Subscriber pose_sub = n.subscribe("/gazebo/model_states", 1, pose_callback);
    ros::Subscriber map_sub = n.subscribe("/map", 1, map_callback);

    //Setup topics to Publish from this node
    ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
    pose_publisher = n.advertise<geometry_msgs::PoseStamped>("/pose", 1, true);
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);

    //Velocity control variable
    geometry_msgs::Twist vel;

    //Set the loop rate
    ros::Rate loop_rate(20);    //20Hz update rate

    visualization_msgs::Marker points;
    points.header.frame_id = "/my_frame";
    points.header.stamp = ros::Time::now();
    points.ns = "points";
    points.action = visualization_msgs::Marker::ADD;
    points.type = visualization_msgs::Marker::POINTS;
    points.id = 1;

    points.scale.x = 0.8f;
    points.scale.y = 0.8f;

    points.color.r = 1.0f;
    points.color.a = 1.0f;

    visualization_msgs::Marker collisionPoints;
    collisionPoints.header.frame_id = "/my_frame";
    collisionPoints.header.stamp = ros::Time::now();
    collisionPoints.ns = "collisionPoints";
    collisionPoints.action = visualization_msgs::Marker::ADD;
    collisionPoints.type = visualization_msgs::Marker::POINTS;
    collisionPoints.id = 1;

    collisionPoints.scale.x = 0.8f;
    collisionPoints.scale.y = 0.8f;

    collisionPoints.color.r = 0.8f;
    collisionPoints.color.g = 0.8f;
    collisionPoints.color.b = 1.0f;
    collisionPoints.color.a = 1.0f;

    while (ros::ok())
    {
    	loop_rate.sleep(); //Maintain the loop rate
    	ros::spinOnce();   //Check for new messages

    	//Main loop code goes here:
    	// vel.linear.x = 1.0; // set linear speed
    	// vel.angular.z = 1.0; // set angular speed

      if(mapReady) {
        generateRandomMap();
        removeCollisionPoints();
        connectNearestNeighbours();
        removeCollisionPaths();
        mapReady = false;

        vector<geometry_msgs::Point> displayPointsVec;
        for(auto &point:pointMap) {
          geometry_msgs::Point p;
          p.x = point.x - MAP_SIZE/2;
          p.y = point.y - MAP_SIZE/2;
          displayPointsVec.push_back(p);
        }
        points.points = displayPointsVec;

        vector<geometry_msgs::Point> displayCollisionPointsVec;
        for(int i =0; i < MAP_SIZE * MAP_SIZE; i++) {
          if(collisionMap[i] > 0) {
            geometry_msgs::Point p;
            p.x = i%MAP_SIZE - MAP_SIZE/2;
            p.y = (int) (i /MAP_SIZE) - MAP_SIZE/2;
            displayCollisionPointsVec.push_back(p);
          }
        }
        collisionPoints.points = displayCollisionPointsVec;
      }

      marker_pub.publish(points);
      marker_pub.publish(collisionPoints);

    	velocity_publisher.publish(vel); // Publish the command velocity
    }

    return 0;
}
