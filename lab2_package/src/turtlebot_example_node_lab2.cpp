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
#define MAP_SIZE 25.0

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <vector>
#include <iostream>
#include <ctime>
#include <chrono>

#include <cmath>

using namespace std;

ros::Publisher pose_publisher;
ros::Publisher marker_pub;

double ips_x;
double ips_y;
double ips_yaw;

int num_particles = 300;

//Create a stuct to define the cornidates and weight of the particle
struct Particle {
  double x = 0;
  double y = 0;
  double yaw = 0;
  double weight = 0;
};

short sgn(int x) { return x >= 0 ? 1 : -1; }

double random_gen() {
  return (double)(rand() % 10000) / 10000.0;
}

//Callback function for the Position topic (SIMULATION)
void pose_callback(const gazebo_msgs::ModelStates& msg) {
  int i;
  for(i = 0; i < msg.name.size(); i++) if(msg.name[i] == "mobile_base") break;

  // ips_x = msg.pose[i].position.x ;
  // ips_y = msg.pose[i].position.y ;
  // ips_yaw = tf::getYaw(msg.pose[i].orientation);

  ips_x = msg.pose[i].position.x ;
  ips_y = msg.pose[i].position.y ;
  ips_yaw = tf::getYaw(msg.pose[i].orientation);
}

double dx = 0;
double dy = 0;
double dang = 0;

auto t1 = chrono::high_resolution_clock::now();
auto t2 = chrono::high_resolution_clock::now();

bool check_odo = true;

void odometry_callback(const nav_msgs::Odometry& msg) {
  if(check_odo) {
    t1 = t2;
    t2 = chrono::high_resolution_clock::now();

    double t_delta = ((double) std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count()) / (1000000000.0);

    dx = t_delta * msg.twist.twist.linear.x;
    dy = t_delta * msg.twist.twist.linear.y;
    dang = t_delta * msg.twist.twist.angular.z;
  }

  check_odo = !check_odo;
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

vector<Particle> particle_set(num_particles);

Particle create_particle () {
  Particle p;
  p.x = random_gen() * MAP_SIZE - MAP_SIZE / 2.0;
  p.y = random_gen() * MAP_SIZE - MAP_SIZE / 2.0;
  p.yaw = random_gen() * 2 * M_PI - M_PI;
  p.weight = 0;
  // double distance = sqrt((particle_set.x-ips_x)*(particle_set.x-ips_x)
                      // + (particle_set.y-ips_y)^(particle_set.y-ips_y));
  // p.weight = 1/distance;

  return p;
}

void random_particle_allocator() {
  for(auto &particle:particle_set) {
    particle = create_particle();
  }
}

void move_particles() {
  for(auto &particle:particle_set) {
    // particle.yaw += dang * (1.0 + random_gen() * 0.08 - 0.04);
    // double dist = sqrt(dx*dx + dy*dy) * (1.0 + random_gen() * 0.08 - 0.04);
    // particle.x += dist * cos(particle.yaw);
    // particle.y += dist * sin(particle.yaw);

    // particle.yaw += dang * (1.0 + random_gen() * 0.1 - 0.05);
    // double dist = sqrt(dx*dx + dy*dy) * (1.0 + random_gen() * 0.1 - 0.05);
    // particle.x += dist * cos(particle.yaw);
    // particle.y += dist * sin(particle.yaw);

    // particle.yaw += dang + random_gen() * 0.04 - 0.02;
    // double dist = sqrt(dx*dx + dy*dy) + + random_gen() * 0.2 - 0.1;
    // particle.x += dist * cos(particle.yaw);
    // particle.y += dist * sin(particle.yaw);

    particle.yaw += dang + (random_gen() * 0.08 - 0.04);
    double dist = sqrt(dx*dx + dy*dy) + (random_gen() * 0.3 - 0.15);
    particle.x += dist * cos(particle.yaw);
    particle.y += dist * sin(particle.yaw);

    // particle.yaw += dang;
    // double dist = sqrt(dx*dx + dy*dy);
    // particle.x += dist * cos(particle.yaw);
    // particle.y += dist * sin(particle.yaw);
  }
}

double distanceToIPS(Particle p) {
  double dist = sqrt((p.x - ips_x) * (p.x - ips_x) + (p.y - ips_y) * (p.y - ips_y));
  return dist;
}

double angDiff(Particle p) {
  double ang = abs(p.yaw - ips_yaw);
  if(ang >= M_PI) {
    ang = 2 * M_PI - ang;
  }
  return ang;
}

double max_weight = 0.0;
double total_weight = 0.0;

void particle_weighting() {
  max_weight = 0.0;
  for(auto &particle:particle_set) {
    double denom = (distanceToIPS(particle) + angDiff(particle));
    // double denom = (distanceToIPS(particle));
    if(denom < 0.0001) {
      denom  = 0.0001;
    }
    particle.weight = 1.0 / denom;

    // ROS_INFO("Inner:  %f", particle.weight);
    if(particle.weight > max_weight) {
      max_weight = particle.weight;
    }
  }

  for(auto &particle:particle_set) {
    particle.weight /= max_weight;
    total_weight += particle.weight;
  }
}

void resampling() {
  vector<Particle> newParticle_set;
  int index = rand() % (num_particles - 1);
  double beta = 0.0;
  for(int i = 0; i < num_particles; i++){
    beta += random_gen() * 2.0 * max_weight;
    while(beta > particle_set[index].weight){
      beta -= particle_set[index].weight;
      index = (index + 1) % (num_particles - 1);
    }
    newParticle_set.push_back(particle_set[index]);
  }

  particle_set = newParticle_set;
}

double predicted_x = 0;
double predicted_y = 0;
double predicted_yaw = 0;

void predict_pos() {
  predicted_x = 0;
  predicted_y = 0;

  for(auto &particle:particle_set) {
    predicted_x += particle.x;
    predicted_y += particle.y;
  }

  predicted_x /= num_particles;
  predicted_y /= num_particles;

  // if(sqrt((predicted_x - ips_x) * (predicted_x - ips_x) + (predicted_y - ips_y) * (predicted_y - ips_y)) < 0.5) {
  //   num_particles = 2;
  // }
}

int main(int argc, char **argv)
{
	  //Initialize the ROS framework
    ros::init(argc,argv,"main_control");
    ros::NodeHandle n;

    //Subscribe to the desired topics and assign callbacks
    ros::Subscriber pose_sub = n.subscribe("/gazebo/model_states", 1, pose_callback);
    ros::Subscriber map_sub = n.subscribe("/map", 1, map_callback);
    ros::Subscriber odom_sub = n.subscribe("/odom", 1, odometry_callback);

    //Setup topics to Publish from this node
    ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
    pose_publisher = n.advertise<geometry_msgs::PoseStamped>("/pose", 1, true);
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);

    //Velocity control variable
    geometry_msgs::Twist vel;

    //Set the loop rate
    ros::Rate loop_rate(20);    //20Hz update rate

    random_particle_allocator();

    visualization_msgs::Marker points;
    points.header.frame_id = "/my_frame";
    points.header.stamp = ros::Time::now();
    points.ns = "points";
    points.action = visualization_msgs::Marker::ADD;
    points.type = visualization_msgs::Marker::POINTS;
    points.id = 1;

    points.scale.x = 0.15;
    points.scale.y = 0.15;

    points.color.r = 1.0f;
    points.color.a = 1.0;

    visualization_msgs::Marker base_point;
    base_point.header.frame_id = "/my_frame";
    base_point.header.stamp = ros::Time::now();
    base_point.ns = "base_point";
    base_point.action = visualization_msgs::Marker::ADD;
    base_point.type = visualization_msgs::Marker::POINTS;
    base_point.id = 2;

    base_point.scale.x = 0.1;
    base_point.scale.y = 0.1;

    base_point.color.r = 1.0f;
    base_point.color.g = 1.0f;
    base_point.color.b = 1.0f;
    base_point.color.a = 1.0;

    bool update = true;

    while (ros::ok())
    {
    	loop_rate.sleep(); //Maintain the loop rate
    	ros::spinOnce();   //Check for new messages

    	//Main loop code goes here:
    	vel.linear.x = 0.5; // set linear speed
    	vel.angular.z = 0.5; // set angular speed

      if(update) {
        move_particles();
        particle_weighting();
        resampling();
      }

      update = !update;

      vector<geometry_msgs::Point> points_vec;
      for(auto &particle:particle_set) {
        geometry_msgs::Point p;
        p.x = particle.x;
        p.y = particle.y;
        p.z = 0;
        points_vec.push_back(p);
      }
      points.points = points_vec;

      // predict_pos();
      //
      // geometry_msgs::Point p;
      // p.x = predicted_x;
      // p.y = predicted_y;
      // p.z = 0;
      // vector<geometry_msgs::Point> points_vec;
      // points_vec.push_back(p);
      // points.points = points_vec;


      geometry_msgs::Point base;
      base.x = ips_x;
      base.y = ips_y;
      base.z = 0;
      vector<geometry_msgs::Point> base_vec;
      base_vec.push_back(base);
      base_point.points = base_vec;

      marker_pub.publish(points);
      marker_pub.publish(base_point);

      // ROS_INFO("IPS: (%f, %f, %f)", ips_x, ips_y, ips_yaw);
      // ROS_INFO("Predicted: (%f, %f, %f)", predicted_x, predicted_y, predicted_yaw);

    	velocity_publisher.publish(vel); // Publish the command velocity
    }

    return 0;
}
