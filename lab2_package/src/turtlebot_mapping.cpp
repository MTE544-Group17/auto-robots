//  ///////////////////////////////////////////////////////////
//
// Lab2 Code for MTE 544
// This file contains example code for use with ME 597 lab 2
// It outlines the basic setup of a ros node and the various
// inputs and outputs needed for this lab
//
// Author: Rishab Sareen & Pavel Shering
//
// //////////////////////////////////////////////////////////
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>

#include <fstream>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

#define MAP_RESOLUTION    81.0
#define MAP_D    10.0

double ips_x;
double ips_y;
double ips_yaw;
double angle_min;
double angle_max;
double angle_inc;
double range_max;
int map_data_origin;
double p_occ = 0.7;
double l_p_occ = log(p_occ/(1-p_occ));
double p_free = 0.3;
double l_p_free = log(p_free/(1-p_free));
double p_0 = 0.5;
double l_p_0 = log(p_0/(1-p_0));
bool got_laser_ranges = false;
std::vector<double> ranges(640);
nav_msgs::OccupancyGrid map;
std::vector<double> l_map_data;


short sgn(int x) { return x >= 0 ? 1 : -1; }

//Bresenham line algorithm (pass empty vectors)
// Usage: (x0, y0) is the first point and (x1, y1) is the second point. The calculated
//        points (x, y) are stored in the x and y vector. x and y should be empty
//	  vectors of integers and shold be defined where this function is called from.
void bresenham(int x0, int y0, int x1, int y1, std::vector<int>& x, std::vector<int>& y)
{
    ROS_INFO("starting bresenham");
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

//Callback function for the Position topic (SIMULATION)
void pose_callback(const gazebo_msgs::ModelStates& msg)
{
    //Create tf broadcaster
    tf::TransformBroadcaster broadcaster;

    int i;
    for(i = 0; i < msg.name.size(); i++) if(msg.name[i] == "mobile_base") break;

    ips_x = msg.pose[i].position.x ;
    ips_y = msg.pose[i].position.y ;
    ips_yaw = tf::getYaw(msg.pose[i].orientation);\

    broadcaster.sendTransform(
          tf::StampedTransform(
            tf::Transform(
              tf::Quaternion(ips_yaw, 0, 0), tf::Vector3(ips_x, ips_y, 0.0)),
              ros::Time::now(),"base_link", "map"));
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

//Callback function for the Laser Scan data topic
void laser_callback(const sensor_msgs::LaserScan& msg)
{
    ROS_INFO("laser_callback");
    angle_min = msg.angle_min;
    angle_max = msg.angle_max;
    angle_inc = msg.angle_increment;
    range_max = msg.range_max;

    for (int i=0; i<msg.ranges.size(); i++)
    {
      if (msg.ranges[i]>msg.range_min&&msg.ranges[i]<msg.range_max)
      {
        ranges[i] = msg.ranges[i];
      }
      else
      {
        //Garbage value, set to max so that update_map will discard
        ranges[i] = range_max;
      }
    }
    ROS_INFO("laser_callback_complete");
    got_laser_ranges = true;
}

void build_map()
{
    // map.header.frame_id = ros::this_node::getName() + "/local_map";
    map.info.map_load_time = ros::Time::now();
    map.info.resolution = MAP_D / MAP_RESOLUTION;
    map.info.width = MAP_RESOLUTION;
    map.info.height = MAP_RESOLUTION;

    map.info.origin.position.x = -static_cast<double>(map.info.width) / 2 * map.info.resolution;
    map.info.origin.position.y = -static_cast<double>(map.info.height) / 2 * map.info.resolution;
    map.info.origin.orientation.w = 1.0;

    map_data_origin = (MAP_RESOLUTION)*((MAP_RESOLUTION-1)/2) + ((MAP_RESOLUTION-1)/2);

    map.data.assign(map.info.width * map.info.height, 50); // fill the map with 50/50 chance of occupancy
    ROS_INFO("%d", map.data.size());
    double log_odds = log(0.5/(1-0.5));
    l_map_data.assign(map.data.size(), log_odds);
}

bool save_map(const std::string& name)
{
    std::string filename;
    if (name.empty()) {
        const ros::Time time = ros::Time::now();
        const int sec = time.sec;
        const int nsec = time.nsec;

        std::stringstream sname;
        sname << "map_";
        sname << std::setw(5) << std::setfill('0') << sec;
        sname << std::setw(0) << "_";
        sname << std::setw(9) << std::setfill('0') << nsec;
        sname << std::setw(0) << ".txt";
        filename = sname.str();
    } else {
     filename = name;
    }

    std::ofstream ofs;
    ofs.open(filename.c_str());
    if (!ofs.is_open()) {
        ROS_ERROR("Cannot open %s", filename.c_str());
        return false;
    }
    for (size_t i = 0; i < map.data.size(); i++) {
        ofs << static_cast<int>(map.data[i]);
        if ((i % map.info.width) == (map.info.width - 1)) {
            ofs << "\n";
        } else {
            ofs << ",";
        }
    }
    ofs.close();
    return true;
}

void update_map ()
{
    ROS_INFO("Updating map");
    std::vector<int> x(1);
    std::vector<int> y(1);
    ROS_INFO("Vectors defined");
    //Bound robot within map dimensions
    int robot_x = int(round((MAP_RESOLUTION/MAP_D)*ips_x));
    int robot_y = int(round((MAP_RESOLUTION/MAP_D)*ips_y));
    ROS_INFO("Robot position defined");

    for (int i=0; i<ranges.size(); i++)
    {
        ROS_INFO("%d",i);
        double endpoint_x = robot_x + ranges[i]*cos(ips_yaw+(angle_min+angle_inc*i));
        double endpoint_y = robot_y + ranges[i]*sin(ips_yaw+(angle_min+angle_inc*i));
        ROS_INFO("Endpoint defined");
        //Bound endpoint within map dimensions
        endpoint_x = round((MAP_RESOLUTION/MAP_D)*endpoint_x);
        endpoint_y = round((MAP_RESOLUTION/MAP_D)*endpoint_y);
        ROS_INFO("Endpoint rounded");
        if (abs(endpoint_x)>int(MAP_RESOLUTION/2))
        {
          endpoint_x = int(MAP_RESOLUTION/2);
        }
        if (abs(endpoint_y)>int(MAP_RESOLUTION/2))
        {
          endpoint_y = int(MAP_RESOLUTION/2);
        }
        ROS_INFO("%d, %d, %d, %d", robot_x, robot_y,int(endpoint_x), int(endpoint_y));
        bresenham(robot_x, robot_y, int(endpoint_x), int(endpoint_y), x, y);
        ROS_INFO("bresenham run");
        //Calculated updated log odds for points defined in x and y vectors
        double e_l_map_data;
        for (int j=0; j<x.size(); j++)
        {
          if (j==(x.size()-1) && ranges[i] < range_max)
          {
            l_map_data[map_data_origin+(x[j]-(y[j]*MAP_RESOLUTION))] =
            l_map_data[map_data_origin+(x[j]-(y[j]*MAP_RESOLUTION))] + l_p_occ - l_p_0;
          }
          else
          {
            l_map_data[map_data_origin+(x[j]-(y[j]*MAP_RESOLUTION))] =
            l_map_data[map_data_origin+(x[j]-(y[j]*MAP_RESOLUTION))] + l_p_free - l_p_0;
          }
          e_l_map_data = exp(l_map_data[map_data_origin+(x[j]-(y[j]*MAP_RESOLUTION))]);
          map.data[map_data_origin+(x[j]-(y[j]*MAP_RESOLUTION))] = int(round(100*(e_l_map_data/(1+e_l_map_data))));
        }
        ROS_INFO("map.data updated");
    }
    ROS_INFO("Updated map");
}

int main(int argc, char **argv)
{
	  //Initialize the ROS framework
    ros::init(argc,argv,"main_control");
    ros::NodeHandle n;

    //Subscribe to the desired topics and assign callbacks
    ros::Subscriber pose_sub = n.subscribe("/gazebo/model_states", 1, pose_callback);
    ros::Subscriber laser_sub = n.subscribe("/scan", 1, laser_callback);

    //Setup topics that this node will Publish to
    ros::Publisher map_publisher = n.advertise<nav_msgs::OccupancyGrid>("/map",1);

    //Initialize our empty map grid
    build_map();
    bool res = save_map("1");
    // print_map(&map, map.info.width);

    //Set the loop rate
    ros::Rate loop_rate(20);    //20Hz update rate

    while (ros::ok())
    {
        ROS_INFO("ATTEMPTING TO BEGIN");
        loop_rate.sleep(); //Maintain the loop rate
        ROS_INFO("ATTEMPTING TO RECEIVE");
        ros::spinOnce();   //Check for new messages

        //Main loop code goes here:
        if (got_laser_ranges==true)
        {
          update_map();
        }
        map_publisher.publish(map);
        ROS_INFO("PUBLISHING");
    }

    return 0;
}
